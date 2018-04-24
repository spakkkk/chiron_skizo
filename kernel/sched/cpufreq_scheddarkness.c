/*
 * CPUFreq governor based on scheduler-provided CPU utilization data.
 *
 * Copyright (C) 2016, Intel Corporation
 * Author: Rafael J. Wysocki <rafael.j.wysocki@intel.com>
 *
 * Copyright (C) 2017, Alucard24@XDA
 * Author: Alucard24 <dmbaoh2@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/cpufreq.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#include <trace/events/power.h>

#include "sched.h"
#include "tune.h"
#ifdef CONFIG_STATE_NOTIFIER
#include <linux/state_notifier.h>
#endif

unsigned long boosted_cpu_util(int cpu);

/* Stub out fast switch routines present on mainline to reduce the backport
 * overhead. */
#define cpufreq_driver_fast_switch(x, y) 0
#define cpufreq_enable_fast_switch(x)
#define cpufreq_disable_fast_switch(x)
#define LATENCY_MULTIPLIER			(1000)
#define DKGOV_KTHREAD_PRIORITY	50

#define FREQ_RESPONSIVENESS			1113600
#define BOOST_PERC					0
#ifdef CONFIG_STATE_NOTIFIER
#define DEFAULT_RATE_LIMIT_SUSP_NS ((s64)(80000 * NSEC_PER_USEC))
#endif

struct dkgov_tunables {
	struct gov_attr_set attr_set;
	unsigned int up_rate_limit_us;
	unsigned int down_rate_limit_us;
	/*
	 * CPUs frequency scaling
	 */
	int freq_responsiveness;
	bool freq_responsiveness_jump;
	unsigned int boost_perc;
	bool iowait_boost_enable;
	int eval_busy_for_freq;
};

struct dkgov_policy {
	struct cpufreq_policy *policy;

	struct dkgov_tunables *tunables;
	struct list_head tunables_hook;

	raw_spinlock_t update_lock;  /* For shared policies */
	u64 last_freq_update_time;
	s64 min_rate_limit_ns;
	s64 up_rate_delay_ns;
	s64 down_rate_delay_ns;
	unsigned int next_freq;

	/* The next fields are only needed if fast switch cannot be used. */
	struct irq_work irq_work;
	struct kthread_work work;
	struct mutex work_lock;
	struct kthread_worker worker;
	struct task_struct *thread;
	bool work_in_progress;

	bool need_freq_update;
};

struct dkgov_cpu {
	struct update_util_data update_util;
	struct dkgov_policy *sg_policy;

	unsigned long iowait_boost;
	unsigned long iowait_boost_max;
	u64 last_update;

	/* The fields below are only needed when sharing a policy. */
	unsigned long util;
	unsigned long max;
	unsigned int flags;

	/* The field below is for single-CPU policies only. */
#ifdef CONFIG_NO_HZ_COMMON
	unsigned long saved_idle_calls;
#endif
};

static DEFINE_PER_CPU(struct dkgov_cpu, dkgov_cpu);
static DEFINE_PER_CPU(struct dkgov_tunables, cached_tunables);

#define LITTLE_NFREQS				16
#define BIG_NFREQS					25
static unsigned long little_capacity[LITTLE_NFREQS] = {
	0,
	149,
	205,
	229,
	253,
	296,
	350,
	406,
	469,
	491,
	527,
	572,
	584,
	630,
	666,
	711
};

static unsigned long big_capacity[BIG_NFREQS] = {
	0,
	149,
	197,
	229,
	253,
	296,
	350,
	372,
	400,
	445,
	495,
	527,
	572,
	598,
	630,
	666,
	711,
	743,
	780,
	812,
	850,
	868,
	914,
	961,
	988
};

/************************ Governor internals ***********************/

static bool dkgov_should_update_freq(struct dkgov_policy *sg_policy, u64 time)
{
	s64 delta_ns;

	if (sg_policy->work_in_progress)
		return false;

	if (unlikely(sg_policy->need_freq_update)) {
		sg_policy->need_freq_update = false;
		/*
		 * This happens when limits change, so forget the previous
		 * next_freq value and force an update.
		 */
		sg_policy->next_freq = UINT_MAX;
		return true;
	}

	delta_ns = time - sg_policy->last_freq_update_time;

	/* No need to recalculate next freq for min_rate_limit_us at least */
	return delta_ns >= sg_policy->min_rate_limit_ns;
}

static bool dkgov_up_down_rate_limit(struct dkgov_policy *sg_policy, u64 time,
				     unsigned int next_freq)
{
	s64 delta_ns;

	delta_ns = time - sg_policy->last_freq_update_time;
#ifdef CONFIG_STATE_NOTIFIER
	if (state_suspended) {
		if (delta_ns < DEFAULT_RATE_LIMIT_SUSP_NS)
			return true;
	}
#endif
	if (next_freq > sg_policy->next_freq &&
	    delta_ns < sg_policy->up_rate_delay_ns)
			return true;

	if (next_freq < sg_policy->next_freq &&
	    delta_ns < sg_policy->down_rate_delay_ns)
			return true;

	return false;
}

static void dkgov_update_commit(struct dkgov_policy *sg_policy, u64 time,
				unsigned int next_freq)
{
	struct cpufreq_policy *policy = sg_policy->policy;

	if (dkgov_up_down_rate_limit(sg_policy, time, next_freq))
		return;

	if (next_freq == CPUFREQ_ENTRY_INVALID || policy->cur == next_freq)
		return;

	sg_policy->next_freq = next_freq;
	sg_policy->last_freq_update_time = time;

	if (policy->fast_switch_enabled) {
		next_freq = cpufreq_driver_fast_switch(policy, next_freq);
		if (next_freq == CPUFREQ_ENTRY_INVALID)
			return;

		policy->cur = next_freq;
		trace_cpu_frequency(next_freq, smp_processor_id());
	} else {
		sg_policy->work_in_progress = true;
		irq_work_queue(&sg_policy->irq_work);
	}
}

static unsigned int resolve_target_freq(struct cpufreq_policy *policy,
					unsigned long util)
{
	struct cpufreq_frequency_table *table;
	unsigned int target_freq = policy->cur;
	int i = 0;

	if (!policy)
		return CPUFREQ_ENTRY_INVALID;

	table = policy->freq_table;
	if (policy->cpu < 2) {
		for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
			if (table[i].frequency == CPUFREQ_ENTRY_INVALID
				|| i >= LITTLE_NFREQS)
				continue;
			if (util < little_capacity[i])
				break;
			target_freq = table[i].frequency;
		}
	} else {
		for (i = 0; (table[i].frequency != CPUFREQ_TABLE_END); i++) {
			if (table[i].frequency == CPUFREQ_ENTRY_INVALID
				|| i >= BIG_NFREQS)
				continue;
			if (util < big_capacity[i])
				break;
			target_freq = table[i].frequency;
		}
	}
	return target_freq;
}

/**
 * get_next_freq - Compute a new frequency for a given cpufreq policy.
 * @sg_policy: scheddarkness policy object to compute the new frequency for.
 * @util: Current CPU utilization.
 * @max: CPU capacity.
 *
 * The lowest driver-supported frequency which is equal or greater than the raw
 * next_freq (as calculated above) is returned, subject to policy min/max and
 * cpufreq driver limitations.
 */
static unsigned int get_next_freq(struct dkgov_policy *sg_policy, unsigned long util,
				  unsigned long max)
{
	struct cpufreq_policy *policy = sg_policy->policy;
	struct dkgov_tunables *tunables = sg_policy->tunables;
	unsigned long cur_util =
			util + ((util * tunables->boost_perc) / 100);

	return resolve_target_freq(policy, cur_util);
}

static inline bool use_pelt(void)
{
#ifdef CONFIG_SCHED_WALT
	return (!sysctl_sched_use_walt_cpu_util || walt_disabled);
#else
	return true;
#endif
}

static void dkgov_get_util(unsigned long *util, unsigned long *max, u64 time)
{
	int cpu = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);
	unsigned long max_cap, rt;
	s64 delta;

	max_cap = arch_scale_cpu_capacity(NULL, cpu);

	sched_avg_update(rq);
	delta = time - rq->age_stamp;
	if (unlikely(delta < 0))
		delta = 0;
	rt = div64_u64(rq->rt_avg, sched_avg_period() + delta);
	rt = (rt * max_cap) >> SCHED_CAPACITY_SHIFT;

	*util = boosted_cpu_util(cpu);
	if (likely(use_pelt()))
		*util = min((*util + rt), max_cap);

	*max = max_cap;
}

static void dkgov_set_iowait_boost(struct dkgov_cpu *sg_cpu, u64 time,
				   unsigned int flags)
{
	struct dkgov_policy *sg_policy = sg_cpu->sg_policy;

	if (!sg_policy->tunables->iowait_boost_enable)
		return;

	if (flags & SCHED_CPUFREQ_IOWAIT) {
		sg_cpu->iowait_boost = sg_cpu->iowait_boost_max;
	} else if (sg_cpu->iowait_boost) {
		s64 delta_ns = time - sg_cpu->last_update;

		/* Clear iowait_boost if the CPU apprears to have been idle. */
		if (delta_ns > TICK_NSEC)
			sg_cpu->iowait_boost = 0;
	}
}

static void dkgov_iowait_boost(struct dkgov_cpu *sg_cpu, unsigned long *util,
			       unsigned long *max)
{
	unsigned long boost_util = sg_cpu->iowait_boost;
	unsigned long boost_max = sg_cpu->iowait_boost_max;

	if (!boost_util)
		return;

	if (*util * boost_max < *max * boost_util) {
		*util = boost_util;
		*max = boost_max;
	}
	sg_cpu->iowait_boost >>= 1;
}

#ifdef CONFIG_NO_HZ_COMMON
static bool dkgov_cpu_is_busy(struct dkgov_cpu *sg_cpu)
{
	unsigned long idle_calls = tick_nohz_get_idle_calls();
	bool ret = idle_calls == sg_cpu->saved_idle_calls;

	sg_cpu->saved_idle_calls = idle_calls;
	return ret;
}
#else
static inline bool dkgov_cpu_is_busy(struct dkgov_cpu *sg_cpu) { return false; }
#endif /* CONFIG_NO_HZ_COMMON */

static void dkgov_update_single(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct dkgov_cpu *sg_cpu = container_of(hook, struct dkgov_cpu, update_util);
	struct dkgov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util, max;
	unsigned int next_f;
	unsigned int freq_responsiveness =
		sg_policy->tunables->freq_responsiveness;
	unsigned int busy_freq = freq_responsiveness;
	int eval_busy = sg_policy->tunables->eval_busy_for_freq;
	bool busy = false;

	dkgov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (!dkgov_should_update_freq(sg_policy, time))
		return;

	if (eval_busy) {
		busy = dkgov_cpu_is_busy(sg_cpu);
		if (eval_busy > 1)
			busy_freq = policy->cur;
	}

	if (flags & SCHED_CPUFREQ_DL) {
		next_f = policy->cpuinfo.max_freq;
	} else {
		dkgov_get_util(&util, &max, time);
		dkgov_iowait_boost(sg_cpu, &util, &max);
		next_f = get_next_freq(sg_policy, util, max);
		/*
		 * Jump directly to the frequency responsiveness if the next 
		 * CPU frequency is greater than current CPU frequency and less
		 * than the frequency responsiveness.
		 */
		if (sg_policy->tunables->freq_responsiveness_jump) {
			if (next_f > policy->cur 
				&& next_f < freq_responsiveness
				&& sg_policy->next_freq != UINT_MAX)
				next_f = freq_responsiveness;
		}
		/*
		 * Do not reduce the frequency if the CPU has not been idle
		 * recently, as the reduction is likely to be premature then.
		 */
		if (busy && next_f < busy_freq
			&& sg_policy->next_freq != UINT_MAX)
			next_f = policy->cur;
	}
	dkgov_update_commit(sg_policy, time, next_f);
}

static unsigned int dkgov_next_freq_shared(struct dkgov_cpu *sg_cpu, u64 time)
{
	struct dkgov_policy *sg_policy = sg_cpu->sg_policy;
	struct cpufreq_policy *policy = sg_policy->policy;
	unsigned long util = 0, max = 1;
	unsigned int j;

	for_each_cpu(j, policy->cpus) {
		struct dkgov_cpu *j_sg_cpu = &per_cpu(dkgov_cpu, j);
		unsigned long j_util, j_max;
		s64 delta_ns;

		/*
		 * If the CPU utilization was last updated before the previous
		 * frequency update and the time elapsed between the last update
		 * of the CPU utilization and the last frequency update is long
		 * enough, don't take the CPU into account as it probably is
		 * idle now (and clear iowait_boost for it).
		 */
		delta_ns = time - j_sg_cpu->last_update;
		if (delta_ns > TICK_NSEC) {
			j_sg_cpu->iowait_boost = 0;
			continue;
		}
		if (j_sg_cpu->flags & SCHED_CPUFREQ_DL)
			return policy->cpuinfo.max_freq;

		j_util = j_sg_cpu->util;
		j_max = j_sg_cpu->max;
		if (j_util * max > j_max * util) {
			util = j_util;
			max = j_max;
		}

		dkgov_iowait_boost(j_sg_cpu, &util, &max);
	}

	return get_next_freq(sg_policy, util, max);
}

static void dkgov_update_shared(struct update_util_data *hook, u64 time,
				unsigned int flags)
{
	struct dkgov_cpu *sg_cpu = container_of(hook, struct dkgov_cpu, update_util);
	struct dkgov_policy *sg_policy = sg_cpu->sg_policy;
	unsigned long util, max;
	unsigned int next_f;

	dkgov_get_util(&util, &max, time);

	raw_spin_lock(&sg_policy->update_lock);

	sg_cpu->util = util;
	sg_cpu->max = max;
	sg_cpu->flags = flags;

	dkgov_set_iowait_boost(sg_cpu, time, flags);
	sg_cpu->last_update = time;

	if (dkgov_should_update_freq(sg_policy, time)) {
		if (flags & SCHED_CPUFREQ_DL)
			next_f = sg_policy->policy->cpuinfo.max_freq;
		else
			next_f = dkgov_next_freq_shared(sg_cpu, time);

		dkgov_update_commit(sg_policy, time, next_f);
	}

	raw_spin_unlock(&sg_policy->update_lock);
}

static void dkgov_work(struct kthread_work *work)
{
	struct dkgov_policy *sg_policy = container_of(work, struct dkgov_policy, work);

	mutex_lock(&sg_policy->work_lock);
	__cpufreq_driver_target(sg_policy->policy, sg_policy->next_freq,
				CPUFREQ_RELATION_L);
	mutex_unlock(&sg_policy->work_lock);

	sg_policy->work_in_progress = false;
}

static void dkgov_irq_work(struct irq_work *irq_work)
{
	struct dkgov_policy *sg_policy;

	sg_policy = container_of(irq_work, struct dkgov_policy, irq_work);

	/*
	 * For RT and deadline tasks, the darknessched governor shoots the
	 * frequency to maximum. Special care must be taken to ensure that this
	 * kthread doesn't result in the same behavior.
	 *
	 * This is (mostly) guaranteed by the work_in_progress flag. The flag is
	 * updated only at the end of the dkgov_work() function and before that
	 * the darknessched governor rejects all other frequency scaling requests.
	 *
	 * There is a very rare case though, where the RT thread yields right
	 * after the work_in_progress flag is cleared. The effects of that are
	 * neglected for now.
	 */
	queue_kthread_work(&sg_policy->worker, &sg_policy->work);
}

/************************** sysfs interface ************************/

static struct dkgov_tunables *global_tunables;
static DEFINE_MUTEX(global_tunables_lock);

static inline struct dkgov_tunables *to_dkgov_tunables(struct gov_attr_set *attr_set)
{
	return container_of(attr_set, struct dkgov_tunables, attr_set);
}

static DEFINE_MUTEX(min_rate_lock);

static void update_min_rate_limit_us(struct dkgov_policy *sg_policy)
{
	mutex_lock(&min_rate_lock);
	sg_policy->min_rate_limit_ns = min(sg_policy->up_rate_delay_ns,
					   sg_policy->down_rate_delay_ns);
	mutex_unlock(&min_rate_lock);
}

/* up_rate_limit_us */
static ssize_t up_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->up_rate_limit_us);
}

/* down_rate_limit_us */
static ssize_t down_rate_limit_us_show(struct gov_attr_set *attr_set, char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->down_rate_limit_us);
}

/* freq_responsiveness */
static ssize_t freq_responsiveness_show(struct gov_attr_set *attr_set, char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%d\n", tunables->freq_responsiveness);
}

/* freq_responsiveness_jump */
static ssize_t freq_responsiveness_jump_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->freq_responsiveness_jump);
}

/* boost_perc */
static ssize_t boost_perc_show(struct gov_attr_set *attr_set, char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->boost_perc);
}

/* iowait_boost_enable */
static ssize_t iowait_boost_enable_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->iowait_boost_enable);
}

/* eval_busy_for_freq */
static ssize_t eval_busy_for_freq_show(struct gov_attr_set *attr_set,
					char *buf)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);

	return sprintf(buf, "%u\n", tunables->eval_busy_for_freq);
}

/* up_rate_limit_us */
static ssize_t up_rate_limit_us_store(struct gov_attr_set *attr_set,
				      const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	struct dkgov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->up_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->up_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_us(sg_policy);
	}

	return count;
}

/* down_rate_limit_us */
static ssize_t down_rate_limit_us_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	struct dkgov_policy *sg_policy;
	unsigned int rate_limit_us;

	if (kstrtouint(buf, 10, &rate_limit_us))
		return -EINVAL;

	tunables->down_rate_limit_us = rate_limit_us;

	list_for_each_entry(sg_policy, &attr_set->policy_list, tunables_hook) {
		sg_policy->down_rate_delay_ns = rate_limit_us * NSEC_PER_USEC;
		update_min_rate_limit_us(sg_policy);
	}

	return count;
}

/* freq_responsiveness */
static ssize_t freq_responsiveness_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	int input;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	if (input == tunables->freq_responsiveness)
		return count;

	tunables->freq_responsiveness = input;

	return count;
}

/* freq_responsiveness_jump */
static ssize_t freq_responsiveness_jump_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	bool enable;

	if (strtobool(buf, &enable))
		return -EINVAL;

	tunables->freq_responsiveness_jump = enable;

	return count;
}

/* boost_perc */
static ssize_t boost_perc_store(struct gov_attr_set *attr_set,
					const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	int input;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	input = min(max(0, input), 20);

	if (input == tunables->boost_perc)
		return count;

	tunables->boost_perc = input;

	return count;
}

/* iowait_boost_enable */
static ssize_t iowait_boost_enable_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	bool enable;

	if (strtobool(buf, &enable))
		return -EINVAL;

	tunables->iowait_boost_enable = enable;

	return count;
}

/* eval_busy_for_freq */
static ssize_t eval_busy_for_freq_store(struct gov_attr_set *attr_set,
					 const char *buf, size_t count)
{
	struct dkgov_tunables *tunables = to_dkgov_tunables(attr_set);
	int input;

	if (kstrtouint(buf, 10, &input))
		return -EINVAL;

	input = min(max(0, input), 2);

	if (input == tunables->eval_busy_for_freq)
		return count;

	tunables->eval_busy_for_freq = input;

	return count;
}

static struct governor_attr up_rate_limit_us = __ATTR_RW(up_rate_limit_us);
static struct governor_attr down_rate_limit_us = __ATTR_RW(down_rate_limit_us);
static struct governor_attr freq_responsiveness = __ATTR_RW(freq_responsiveness);
static struct governor_attr freq_responsiveness_jump = __ATTR_RW(freq_responsiveness_jump);
static struct governor_attr boost_perc = __ATTR_RW(boost_perc);
static struct governor_attr iowait_boost_enable = __ATTR_RW(iowait_boost_enable);
static struct governor_attr eval_busy_for_freq = __ATTR_RW(eval_busy_for_freq);

static struct attribute *dkgov_attributes[] = {
	&up_rate_limit_us.attr,
	&down_rate_limit_us.attr,
	&freq_responsiveness.attr,
	&freq_responsiveness_jump.attr,
	&boost_perc.attr,
	&iowait_boost_enable.attr,
	&eval_busy_for_freq.attr,
	NULL
};

static struct kobj_type dkgov_tunables_ktype = {
	.default_attrs = dkgov_attributes,
	.sysfs_ops = &governor_sysfs_ops,
};

/********************** cpufreq governor interface *********************/
#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDDARKNESS
static
#endif
struct cpufreq_governor cpufreq_gov_scheddarkness;

static struct dkgov_policy *dkgov_policy_alloc(struct cpufreq_policy *policy)
{
	struct dkgov_policy *sg_policy;

	sg_policy = kzalloc(sizeof(*sg_policy), GFP_KERNEL);
	if (!sg_policy)
		return NULL;

	sg_policy->policy = policy;
	raw_spin_lock_init(&sg_policy->update_lock);
	return sg_policy;
}

static void dkgov_policy_free(struct dkgov_policy *sg_policy)
{
	kfree(sg_policy);
}

static int dkgov_kthread_create(struct dkgov_policy *sg_policy)
{
	struct task_struct *thread;
	struct sched_param param = { .sched_priority = MAX_USER_RT_PRIO / 2 };
	struct cpufreq_policy *policy = sg_policy->policy;
	int ret;

	/* kthread only required for slow path */
	if (policy->fast_switch_enabled)
		return 0;

	init_kthread_work(&sg_policy->work, dkgov_work);
	init_kthread_worker(&sg_policy->worker);
	thread = kthread_create(kthread_worker_fn, &sg_policy->worker,
				"dkgov:%d",
				cpumask_first(policy->related_cpus));
	if (IS_ERR(thread)) {
		pr_err("failed to create dkgov thread: %ld\n", PTR_ERR(thread));
		return PTR_ERR(thread);
	}

	ret = sched_setscheduler_nocheck(thread, SCHED_FIFO, &param);
	if (ret) {
		kthread_stop(thread);
		pr_warn("%s: failed to set SCHED_FIFO\n", __func__);
		return ret;
	}

	sg_policy->thread = thread;
	kthread_bind_mask(thread, policy->related_cpus);
	init_irq_work(&sg_policy->irq_work, dkgov_irq_work);
	mutex_init(&sg_policy->work_lock);

	wake_up_process(thread);

	return 0;
}

static void dkgov_kthread_stop(struct dkgov_policy *sg_policy)
{
	/* kthread only required for slow path */
	if (sg_policy->policy->fast_switch_enabled)
		return;

	flush_kthread_worker(&sg_policy->worker);
	kthread_stop(sg_policy->thread);
	mutex_destroy(&sg_policy->work_lock);
}

static struct dkgov_tunables *dkgov_tunables_alloc(struct dkgov_policy *sg_policy)
{
	struct dkgov_tunables *tunables;

	tunables = kzalloc(sizeof(*tunables), GFP_KERNEL);
	if (tunables) {
		gov_attr_set_init(&tunables->attr_set, &sg_policy->tunables_hook);
		if (!have_governor_per_policy())
			global_tunables = tunables;
	}
	return tunables;
}

static void dkgov_tunables_free(struct dkgov_tunables *tunables)
{
	if (!have_governor_per_policy())
		global_tunables = NULL;

	kfree(tunables);
}

static void store_tunables_data(struct dkgov_tunables *tunables,
		struct cpufreq_policy *policy)
{
	struct dkgov_tunables *ptunables;
	unsigned int cpu = cpumask_first(policy->related_cpus);

	ptunables = &per_cpu(cached_tunables, cpu);
	if (!ptunables)
		return;
	ptunables->up_rate_limit_us = tunables->up_rate_limit_us;
	ptunables->down_rate_limit_us = tunables->down_rate_limit_us;
	ptunables->freq_responsiveness = tunables->freq_responsiveness;
	ptunables->freq_responsiveness_jump = tunables->freq_responsiveness_jump;
	ptunables->boost_perc = tunables->boost_perc;
	ptunables->iowait_boost_enable = tunables->iowait_boost_enable;
	ptunables->eval_busy_for_freq = tunables->eval_busy_for_freq;
	pr_debug("tunables data saved for cpu[%u]\n", cpu);
}

static void get_tunables_data(struct dkgov_tunables *tunables,
		struct cpufreq_policy *policy)
{
	struct dkgov_tunables *ptunables;
	unsigned int lat;
	unsigned int cpu = cpumask_first(policy->related_cpus);

	ptunables = &per_cpu(cached_tunables, cpu);
	if (!ptunables)
		goto initialize;

	if (ptunables->up_rate_limit_us > 0) {
		tunables->up_rate_limit_us = ptunables->up_rate_limit_us;
		tunables->down_rate_limit_us = ptunables->down_rate_limit_us;
		tunables->freq_responsiveness = ptunables->freq_responsiveness;
		tunables->freq_responsiveness_jump = ptunables->freq_responsiveness_jump;
		tunables->boost_perc = ptunables->boost_perc;
		tunables->iowait_boost_enable = ptunables->iowait_boost_enable;
		tunables->eval_busy_for_freq = ptunables->eval_busy_for_freq;
		pr_debug("tunables data restored for cpu[%u]\n", cpu);
		goto out;
	}

initialize:
	tunables->up_rate_limit_us = LATENCY_MULTIPLIER;
	tunables->down_rate_limit_us = LATENCY_MULTIPLIER;
	lat = policy->cpuinfo.transition_latency / NSEC_PER_USEC;
	if (lat) {
		tunables->up_rate_limit_us *= lat;
		tunables->down_rate_limit_us *= lat;
	}
	tunables->freq_responsiveness = FREQ_RESPONSIVENESS;
	tunables->freq_responsiveness_jump = true;
	tunables->boost_perc = BOOST_PERC;
	tunables->eval_busy_for_freq = 0;
	pr_debug("tunables data initialized for cpu[%u]\n", cpu);
out:
	return;
}

static int dkgov_init(struct cpufreq_policy *policy)
{
	struct dkgov_policy *sg_policy;
	struct dkgov_tunables *tunables;
	int ret = 0;

	/* State should be equivalent to EXIT */
	if (policy->governor_data)
		return -EBUSY;

	cpufreq_enable_fast_switch(policy);

	sg_policy = dkgov_policy_alloc(policy);
	if (!sg_policy) {
		ret = -ENOMEM;
		goto disable_fast_switch;
	}

	ret = dkgov_kthread_create(sg_policy);
	if (ret)
		goto free_sg_policy;

	mutex_lock(&global_tunables_lock);

	if (global_tunables) {
		if (WARN_ON(have_governor_per_policy())) {
			ret = -EINVAL;
			goto stop_kthread;
		}
		policy->governor_data = sg_policy;
		sg_policy->tunables = global_tunables;

		gov_attr_set_get(&global_tunables->attr_set, &sg_policy->tunables_hook);
		goto out;
	}

	tunables = dkgov_tunables_alloc(sg_policy);
	if (!tunables) {
		ret = -ENOMEM;
		goto stop_kthread;
	}

	get_tunables_data(tunables, policy);
	policy->governor_data = sg_policy;
	sg_policy->tunables = tunables;

	ret = kobject_init_and_add(&tunables->attr_set.kobj, &dkgov_tunables_ktype,
				   get_governor_parent_kobj(policy), "%s",
				   cpufreq_gov_scheddarkness.name);
	if (ret)
		goto fail;

out:
	mutex_unlock(&global_tunables_lock);
	return 0;

fail:
	policy->governor_data = NULL;
	dkgov_tunables_free(tunables);

stop_kthread:
	dkgov_kthread_stop(sg_policy);

free_sg_policy:
	mutex_unlock(&global_tunables_lock);

	dkgov_policy_free(sg_policy);

disable_fast_switch:
	cpufreq_disable_fast_switch(policy);

	pr_err("initialization failed (error %d)\n", ret);
	return ret;
}

static int dkgov_exit(struct cpufreq_policy *policy)
{
	struct dkgov_policy *sg_policy = policy->governor_data;
	struct dkgov_tunables *tunables = sg_policy->tunables;
	unsigned int count;

	mutex_lock(&global_tunables_lock);

	store_tunables_data(sg_policy->tunables, policy);
	count = gov_attr_set_put(&tunables->attr_set, &sg_policy->tunables_hook);
	policy->governor_data = NULL;
	if (!count)
		dkgov_tunables_free(tunables);

	mutex_unlock(&global_tunables_lock);

	dkgov_kthread_stop(sg_policy);
	dkgov_policy_free(sg_policy);
	cpufreq_disable_fast_switch(policy);

	return 0;
}

static int dkgov_start(struct cpufreq_policy *policy)
{
	struct dkgov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	sg_policy->up_rate_delay_ns =
		sg_policy->tunables->up_rate_limit_us * NSEC_PER_USEC;
	sg_policy->down_rate_delay_ns =
		sg_policy->tunables->down_rate_limit_us * NSEC_PER_USEC;
	update_min_rate_limit_us(sg_policy);
	sg_policy->last_freq_update_time = 0;
	sg_policy->next_freq = policy->cur;
	sg_policy->work_in_progress = false;
	sg_policy->need_freq_update = false;

	for_each_cpu(cpu, policy->cpus) {
		struct dkgov_cpu *sg_cpu = &per_cpu(dkgov_cpu, cpu);

		memset(sg_cpu, 0, sizeof(*sg_cpu));
		sg_cpu->sg_policy = sg_policy;
		sg_cpu->flags = SCHED_CPUFREQ_DL;
		sg_cpu->iowait_boost_max = policy->cpuinfo.max_freq;
		cpufreq_add_update_util_hook(cpu, &sg_cpu->update_util,
					     policy_is_shared(policy) ?
							dkgov_update_shared :
							dkgov_update_single);
	}
	return 0;
}

static int dkgov_stop(struct cpufreq_policy *policy)
{
	struct dkgov_policy *sg_policy = policy->governor_data;
	unsigned int cpu;

	for_each_cpu(cpu, policy->cpus)
		cpufreq_remove_update_util_hook(cpu);

	synchronize_sched();

	if (!policy->fast_switch_enabled) {
		irq_work_sync(&sg_policy->irq_work);
		kthread_cancel_work_sync(&sg_policy->work);
	}

	return 0;
}

static int dkgov_limits(struct cpufreq_policy *policy)
{
	struct dkgov_policy *sg_policy = policy->governor_data;

	if (!policy->fast_switch_enabled) {
		mutex_lock(&sg_policy->work_lock);
		cpufreq_policy_apply_limits(policy);
		mutex_unlock(&sg_policy->work_lock);
	}

	sg_policy->need_freq_update = true;

	return 0;
}

static int cpufreq_scheddarkness_cb(struct cpufreq_policy *policy,
				unsigned int event)
{
	switch(event) {
	case CPUFREQ_GOV_POLICY_INIT:
		return dkgov_init(policy);
	case CPUFREQ_GOV_POLICY_EXIT:
		return dkgov_exit(policy);
	case CPUFREQ_GOV_START:
		return dkgov_start(policy);
	case CPUFREQ_GOV_STOP:
		return dkgov_stop(policy);
	case CPUFREQ_GOV_LIMITS:
		return dkgov_limits(policy);
	default:
		BUG();
	}
}

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_SCHEDDARKNESS
static
#endif
struct cpufreq_governor cpufreq_gov_scheddarkness = {
	.name = "scheddarkness",
	.governor = cpufreq_scheddarkness_cb,
	.owner = THIS_MODULE,
};

static int __init dkgov_register(void)
{
	return cpufreq_register_governor(&cpufreq_gov_scheddarkness);
}
fs_initcall(dkgov_register);
