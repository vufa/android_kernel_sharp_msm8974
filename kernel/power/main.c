/*
 * kernel/power/main.c - PM subsystem core functionality.
 *
 * Copyright (c) 2003 Patrick Mochel
 * Copyright (c) 2003 Open Source Development Lab
 *
 * This file is released under the GPLv2
 *
 */

#include <linux/export.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/resume-trace.h>
#include <linux/workqueue.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/hrtimer.h>

#include "power.h"

#ifdef CONFIG_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
#include <mach/perflock.h>
#include <linux/pm_qos.h>
#include <mach/cpuidle.h>
DEFINE_MUTEX(user_lock_mutex);
#endif
#else
#include <mach/perflock.h>
#endif
#endif

#define MAX_BUF 100

DEFINE_MUTEX(pm_mutex);

#ifdef CONFIG_PM_SLEEP

/* Routines for PM-transition notifications */

static BLOCKING_NOTIFIER_HEAD(pm_chain_head);

static void touch_event_fn(struct work_struct *work);
static DECLARE_WORK(touch_event_struct, touch_event_fn);

static struct hrtimer tc_ev_timer;
static int tc_ev_processed;
static ktime_t touch_evt_timer_val;

int register_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(register_pm_notifier);

int unregister_pm_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&pm_chain_head, nb);
}
EXPORT_SYMBOL_GPL(unregister_pm_notifier);

int pm_notifier_call_chain(unsigned long val)
{
	int ret = blocking_notifier_call_chain(&pm_chain_head, val, NULL);

	return notifier_to_errno(ret);
}

/* If set, devices may be suspended and resumed asynchronously. */
int pm_async_enabled = 1;

static ssize_t pm_async_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_async_enabled);
}

static ssize_t pm_async_store(struct kobject *kobj, struct kobj_attribute *attr,
			      const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val > 1)
		return -EINVAL;

	pm_async_enabled = val;
	return n;
}

power_attr(pm_async);

static ssize_t
touch_event_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	if (tc_ev_processed == 0)
		return snprintf(buf, strnlen("touch_event", MAX_BUF) + 1,
				"touch_event");
	else
		return snprintf(buf, strnlen("null", MAX_BUF) + 1,
				"null");
}

static ssize_t
touch_event_store(struct kobject *kobj,
		  struct kobj_attribute *attr,
		  const char *buf, size_t n)
{

	hrtimer_cancel(&tc_ev_timer);
	tc_ev_processed = 0;

	/* set a timer to notify the userspace to stop processing
	 * touch event
	 */
	hrtimer_start(&tc_ev_timer, touch_evt_timer_val, HRTIMER_MODE_REL);

	/* wakeup the userspace poll */
	sysfs_notify(kobj, NULL, "touch_event");

	return n;
}

power_attr(touch_event);

static ssize_t
touch_event_timer_show(struct kobject *kobj,
		 struct kobj_attribute *attr, char *buf)
{
	return snprintf(buf, MAX_BUF, "%lld", touch_evt_timer_val.tv64);
}

static ssize_t
touch_event_timer_store(struct kobject *kobj,
			struct kobj_attribute *attr,
			const char *buf, size_t n)
{
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	touch_evt_timer_val = ktime_set(0, val*1000);

	return n;
}

power_attr(touch_event_timer);

static void touch_event_fn(struct work_struct *work)
{
	/* wakeup the userspace poll */
	tc_ev_processed = 1;
	sysfs_notify(power_kobj, NULL, "touch_event");

	return;
}

static enum hrtimer_restart tc_ev_stop(struct hrtimer *hrtimer)
{

	schedule_work(&touch_event_struct);

	return HRTIMER_NORESTART;
}

#ifdef CONFIG_PM_DEBUG
int pm_test_level = TEST_NONE;

static const char * const pm_tests[__TEST_AFTER_LAST] = {
	[TEST_NONE] = "none",
	[TEST_CORE] = "core",
	[TEST_CPUS] = "processors",
	[TEST_PLATFORM] = "platform",
	[TEST_DEVICES] = "devices",
	[TEST_FREEZER] = "freezer",
};

static ssize_t pm_test_show(struct kobject *kobj, struct kobj_attribute *attr,
				char *buf)
{
	char *s = buf;
	int level;

	for (level = TEST_FIRST; level <= TEST_MAX; level++)
		if (pm_tests[level]) {
			if (level == pm_test_level)
				s += sprintf(s, "[%s] ", pm_tests[level]);
			else
				s += sprintf(s, "%s ", pm_tests[level]);
		}

	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';

	return (s - buf);
}

static ssize_t pm_test_store(struct kobject *kobj, struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	const char * const *s;
	int level;
	char *p;
	int len;
	int error = -EINVAL;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	lock_system_sleep();

	level = TEST_FIRST;
	for (s = &pm_tests[level]; level <= TEST_MAX; s++, level++)
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len)) {
			pm_test_level = level;
			error = 0;
			break;
		}

	unlock_system_sleep();

	return error ? error : n;
}

power_attr(pm_test);
#endif /* CONFIG_PM_DEBUG */

#ifdef CONFIG_DEBUG_FS
static char *suspend_step_name(enum suspend_stat_step step)
{
	switch (step) {
	case SUSPEND_FREEZE:
		return "freeze";
	case SUSPEND_PREPARE:
		return "prepare";
	case SUSPEND_SUSPEND:
		return "suspend";
	case SUSPEND_SUSPEND_NOIRQ:
		return "suspend_noirq";
	case SUSPEND_RESUME_NOIRQ:
		return "resume_noirq";
	case SUSPEND_RESUME:
		return "resume";
	default:
		return "";
	}
}

static int suspend_stats_show(struct seq_file *s, void *unused)
{
	int i, index, last_dev, last_errno, last_step;

	last_dev = suspend_stats.last_failed_dev + REC_FAILED_NUM - 1;
	last_dev %= REC_FAILED_NUM;
	last_errno = suspend_stats.last_failed_errno + REC_FAILED_NUM - 1;
	last_errno %= REC_FAILED_NUM;
	last_step = suspend_stats.last_failed_step + REC_FAILED_NUM - 1;
	last_step %= REC_FAILED_NUM;
	seq_printf(s, "%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n"
			"%s: %d\n%s: %d\n%s: %d\n%s: %d\n%s: %d\n",
			"success", suspend_stats.success,
			"fail", suspend_stats.fail,
			"failed_freeze", suspend_stats.failed_freeze,
			"failed_prepare", suspend_stats.failed_prepare,
			"failed_suspend", suspend_stats.failed_suspend,
			"failed_suspend_late",
				suspend_stats.failed_suspend_late,
			"failed_suspend_noirq",
				suspend_stats.failed_suspend_noirq,
			"failed_resume", suspend_stats.failed_resume,
			"failed_resume_early",
				suspend_stats.failed_resume_early,
			"failed_resume_noirq",
				suspend_stats.failed_resume_noirq);
	seq_printf(s,	"failures:\n  last_failed_dev:\t%-s\n",
			suspend_stats.failed_devs[last_dev]);
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_dev + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		seq_printf(s, "\t\t\t%-s\n",
			suspend_stats.failed_devs[index]);
	}
	seq_printf(s,	"  last_failed_errno:\t%-d\n",
			suspend_stats.errno[last_errno]);
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_errno + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		seq_printf(s, "\t\t\t%-d\n",
			suspend_stats.errno[index]);
	}
	seq_printf(s,	"  last_failed_step:\t%-s\n",
			suspend_step_name(
				suspend_stats.failed_steps[last_step]));
	for (i = 1; i < REC_FAILED_NUM; i++) {
		index = last_step + REC_FAILED_NUM - i;
		index %= REC_FAILED_NUM;
		seq_printf(s, "\t\t\t%-s\n",
			suspend_step_name(
				suspend_stats.failed_steps[index]));
	}

	return 0;
}

static int suspend_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, suspend_stats_show, NULL);
}

static const struct file_operations suspend_stats_operations = {
	.open           = suspend_stats_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static int __init pm_debugfs_init(void)
{
	debugfs_create_file("suspend_stats", S_IFREG | S_IRUGO,
			NULL, NULL, &suspend_stats_operations);
	return 0;
}

late_initcall(pm_debugfs_init);
#endif /* CONFIG_DEBUG_FS */

#endif /* CONFIG_PM_SLEEP */

struct kobject *power_kobj;

/**
 *	state - control system power state.
 *
 *	show() returns what states are supported, which is hard-coded to
 *	'standby' (Power-On Suspend), 'mem' (Suspend-to-RAM), and
 *	'disk' (Suspend-to-Disk).
 *
 *	store() accepts one of those strings, translates it into the
 *	proper enumerated value, and initiates a suspend transition.
 */
static ssize_t state_show(struct kobject *kobj, struct kobj_attribute *attr,
			  char *buf)
{
	char *s = buf;
#ifdef CONFIG_SUSPEND
	int i;

	for (i = 0; i < PM_SUSPEND_MAX; i++) {
		if (pm_states[i] && valid_state(i))
			s += sprintf(s,"%s ", pm_states[i]);
	}
#endif
#ifdef CONFIG_HIBERNATION
	s += sprintf(s, "%s\n", "disk");
#else
	if (s != buf)
		/* convert the last space to a newline */
		*(s-1) = '\n';
#endif
	return (s - buf);
}

static suspend_state_t decode_state(const char *buf, size_t n)
{
#ifdef CONFIG_SUSPEND
#ifdef CONFIG_EARLYSUSPEND
	suspend_state_t state = PM_SUSPEND_ON;
#else
	suspend_state_t state = PM_SUSPEND_STANDBY;
#endif
	const char * const *s;
#endif
	char *p;
	int len;

	p = memchr(buf, '\n', n);
	len = p ? p - buf : n;

	/* Check hibernation first. */
	if (len == 4 && !strncmp(buf, "disk", len))
		return PM_SUSPEND_MAX;

#ifdef CONFIG_SUSPEND
	for (s = &pm_states[state]; state < PM_SUSPEND_MAX; s++, state++)
		if (*s && len == strlen(*s) && !strncmp(buf, *s, len))
			return state;
#endif

	return PM_SUSPEND_ON;
}

static ssize_t state_store(struct kobject *kobj, struct kobj_attribute *attr,
			   const char *buf, size_t n)
{
	suspend_state_t state;
	int error;

	error = pm_autosleep_lock();
	if (error)
		return error;

	if (pm_autosleep_state() > PM_SUSPEND_ON) {
		error = -EBUSY;
		goto out;
	}

	state = decode_state(buf, n);
	if (state < PM_SUSPEND_MAX)
		error = pm_suspend(state);
	else if (state == PM_SUSPEND_MAX)
		error = hibernate();
	else
		error = -EINVAL;

 out:
	pm_autosleep_unlock();
	return error ? error : n;
}

power_attr(state);

#ifdef CONFIG_PM_SLEEP
/*
 * The 'wakeup_count' attribute, along with the functions defined in
 * drivers/base/power/wakeup.c, provides a means by which wakeup events can be
 * handled in a non-racy way.
 *
 * If a wakeup event occurs when the system is in a sleep state, it simply is
 * woken up.  In turn, if an event that would wake the system up from a sleep
 * state occurs when it is undergoing a transition to that sleep state, the
 * transition should be aborted.  Moreover, if such an event occurs when the
 * system is in the working state, an attempt to start a transition to the
 * given sleep state should fail during certain period after the detection of
 * the event.  Using the 'state' attribute alone is not sufficient to satisfy
 * these requirements, because a wakeup event may occur exactly when 'state'
 * is being written to and may be delivered to user space right before it is
 * frozen, so the event will remain only partially processed until the system is
 * woken up by another event.  In particular, it won't cause the transition to
 * a sleep state to be aborted.
 *
 * This difficulty may be overcome if user space uses 'wakeup_count' before
 * writing to 'state'.  It first should read from 'wakeup_count' and store
 * the read value.  Then, after carrying out its own preparations for the system
 * transition to a sleep state, it should write the stored value to
 * 'wakeup_count'.  If that fails, at least one wakeup event has occurred since
 * 'wakeup_count' was read and 'state' should not be written to.  Otherwise, it
 * is allowed to write to 'state', but the transition will be aborted if there
 * are any wakeup events detected after 'wakeup_count' was written to.
 */

static ssize_t wakeup_count_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	unsigned int val;

	return pm_get_wakeup_count(&val, true) ?
		sprintf(buf, "%u\n", val) : -EINTR;
}

static ssize_t wakeup_count_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	unsigned int val;
	int error;

	error = pm_autosleep_lock();
	if (error)
		return error;

	if (pm_autosleep_state() > PM_SUSPEND_ON) {
		error = -EBUSY;
		goto out;
	}

	error = -EINVAL;
	if (sscanf(buf, "%u", &val) == 1) {
		if (pm_save_wakeup_count(val))
			error = n;
	}

 out:
	pm_autosleep_unlock();
	return error;
}

power_attr(wakeup_count);

#ifdef CONFIG_PM_AUTOSLEEP
static ssize_t autosleep_show(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      char *buf)
{
	suspend_state_t state = pm_autosleep_state();

	if (state == PM_SUSPEND_ON)
		return sprintf(buf, "off\n");

#ifdef CONFIG_SUSPEND
	if (state < PM_SUSPEND_MAX)
		return sprintf(buf, "%s\n", valid_state(state) ?
						pm_states[state] : "error");
#endif
#ifdef CONFIG_HIBERNATION
	return sprintf(buf, "disk\n");
#else
	return sprintf(buf, "error");
#endif
}

static ssize_t autosleep_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	suspend_state_t state = decode_state(buf, n);
	int error;

	if (state == PM_SUSPEND_ON
	    && strcmp(buf, "off") && strcmp(buf, "off\n"))
		return -EINVAL;

	error = pm_autosleep_set_state(state);

#ifdef CONFIG_SHSYS_CUST
	if(!error)
		sysfs_notify(kobj, NULL, "autosleep");
#endif /* CONFIG_SHSYS_CUST */

	return error ? error : n;
}

power_attr(autosleep);
#endif /* CONFIG_PM_AUTOSLEEP */

#ifdef CONFIG_PM_WAKELOCKS
static ssize_t wake_lock_show(struct kobject *kobj,
			      struct kobj_attribute *attr,
			      char *buf)
{
	return pm_show_wakelocks(buf, true);
}

static ssize_t wake_lock_store(struct kobject *kobj,
			       struct kobj_attribute *attr,
			       const char *buf, size_t n)
{
	int error = pm_wake_lock(buf);
	return error ? error : n;
}

power_attr(wake_lock);

static ssize_t wake_unlock_show(struct kobject *kobj,
				struct kobj_attribute *attr,
				char *buf)
{
	return pm_show_wakelocks(buf, false);
}

static ssize_t wake_unlock_store(struct kobject *kobj,
				 struct kobj_attribute *attr,
				 const char *buf, size_t n)
{
	int error = pm_wake_unlock(buf);
	return error ? error : n;
}

power_attr(wake_unlock);

#endif /* CONFIG_PM_WAKELOCKS */
#endif /* CONFIG_PM_SLEEP */

#ifdef CONFIG_PM_TRACE
int pm_trace_enabled;

static ssize_t pm_trace_show(struct kobject *kobj, struct kobj_attribute *attr,
			     char *buf)
{
	return sprintf(buf, "%d\n", pm_trace_enabled);
}

static ssize_t
pm_trace_store(struct kobject *kobj, struct kobj_attribute *attr,
	       const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) == 1) {
		pm_trace_enabled = !!val;
		return n;
	}
	return -EINVAL;
}

power_attr(pm_trace);

static ssize_t pm_trace_dev_match_show(struct kobject *kobj,
				       struct kobj_attribute *attr,
				       char *buf)
{
	return show_trace_dev_match(buf, PAGE_SIZE);
}

static ssize_t
pm_trace_dev_match_store(struct kobject *kobj, struct kobj_attribute *attr,
			 const char *buf, size_t n)
{
	return -EINVAL;
}

power_attr(pm_trace_dev_match);

#endif /* CONFIG_PM_TRACE */

#ifdef CONFIG_USER_WAKELOCK
power_attr(wake_lock);
power_attr(wake_unlock);
#endif

#ifdef CONFIG_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
#define SHSYS_PM_QOS_USER_PERFLOCK_LATENCY 34
static struct pm_qos_request user_perf_lock_qos;
static uint32_t user_perf_lock_num[PERF_LOCK_INVALID];
static uint32_t user_limit_lock_num[PERF_LOCK_INVALID];
static struct perf_lock user_perf_lock[PERF_LOCK_INVALID];
static struct perf_lock user_limit_lock[PERF_LOCK_INVALID];
#endif
#else
static struct perf_lock user_perf_lock;
static struct perf_lock user_cpufreq_ceiling[CEILING_LEVEL_INVALID];
#endif

#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
static ssize_t
perflock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	int i, perflock_enable = 0;
	for (i = 0; i < PERF_LOCK_INVALID; i++)
		if(is_perf_lock_active(&user_perf_lock[i]) != 0)
			perflock_enable |= (1 << i);

	return sprintf(buf, "%d\n", perflock_enable);
}
#endif
#else
static ssize_t
perflock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	return sprintf(buf, "%d\n", (is_perf_lock_active(&user_perf_lock) != 0));
}
#endif

#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
static inline void user_perflock(int level, int val)
{
	mutex_lock(&user_lock_mutex);
	if (val == 1) {
		if (!is_perf_locked())
			pm_qos_update_request(&user_perf_lock_qos, SHSYS_PM_QOS_USER_PERFLOCK_LATENCY);
		if (!is_perf_lock_active(&user_perf_lock[level]))
			perf_lock(&user_perf_lock[level]);
		user_perf_lock_num[level]++;
	}
	if (val == 0) {
		if(user_perf_lock_num[level] > 0)
			user_perf_lock_num[level]--;
		if (user_perf_lock_num[level] == 0 && is_perf_lock_active(&user_perf_lock[level]))
			perf_unlock(&user_perf_lock[level]);
		if (!is_perf_locked())
			pm_qos_update_request(&user_perf_lock_qos, PM_QOS_DEFAULT_VALUE);
	}
	mutex_unlock(&user_lock_mutex);
}

static ssize_t
perflock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	unsigned int input_rate;
	int input_mode, level;

	if (sscanf(buf, "%u %d", &input_rate, &input_mode) != 2)
		return -EINVAL;
	if (input_mode != 0 && input_mode != 1)
		return -EINVAL;

	for (level = 0; level < PERF_LOCK_INVALID; level++)
		if (input_rate == (get_perflock_acpu_table(level) / 1000)) {
			user_perflock(level, input_mode);
			return n;
		}

	return -EINVAL;
}
power_attr(perflock);
#endif
#else
static ssize_t
perflock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) > 0) {
		if (val == 1 && !is_perf_lock_active(&user_perf_lock))
			perf_lock(&user_perf_lock);
		if (val == 0 && is_perf_lock_active(&user_perf_lock))
			perf_unlock(&user_perf_lock);
		return n;
	}
	return -EINVAL;
}
power_attr(perflock);
#endif

#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
static ssize_t
limitlock_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	/* bit[0] = low, bit[1] = medium, bit[2] = high */
	int i, ceiling_enable = 0;

	for (i = 0; i < PERF_LOCK_INVALID; i++)
		if(is_perf_lock_active(&user_limit_lock[i]) != 0)
			ceiling_enable |= (1 << i);

	return sprintf(buf, "%d\n", ceiling_enable);
}
#endif
#else
static ssize_t
cpufreq_ceiling_show(struct kobject *kobj, struct kobj_attribute *attr,
		char *buf)
{
	/* bit[0] = low, bit[1] = medium, bit[2] = high */
	int i, ceiling_enable = 0;

	for (i = 0; i < CEILING_LEVEL_INVALID; i++)
		if(is_perf_lock_active(&user_cpufreq_ceiling[i]) != 0)
			ceiling_enable |= (1 << i);

	return sprintf(buf, "%d\n", ceiling_enable);
}
#endif

#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
static inline void user_cpufreq_ceiling_lock(int level, int val)
{
	mutex_lock(&user_lock_mutex);
	if (val == 1) {
		if (!is_perf_lock_active(&user_limit_lock[level]))
			limit_lock(&user_limit_lock[level]);
		user_limit_lock_num[level]++;
	}
	if (val == 0) {
		if(user_limit_lock_num[level] > 0)
			user_limit_lock_num[level]--;
		if (user_limit_lock_num[level] == 0 && is_perf_lock_active(&user_limit_lock[level]))
			limit_unlock(&user_limit_lock[level]);
	}
	mutex_unlock(&user_lock_mutex);
}

static ssize_t
limitlock_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	unsigned int input_rate;
	int input_mode, level;

	if (sscanf(buf, "%u %d", &input_rate, &input_mode) != 2)
		return -EINVAL;
	if (input_mode != 0 && input_mode != 1)
		return -EINVAL;

	for (level = 0; level < PERF_LOCK_INVALID; level++)
		if (input_rate == (get_limitlock_acpu_table(level) / 1000)) {
			user_cpufreq_ceiling_lock(level, input_mode);
			return n;
		}

	return -EINVAL;
}
power_attr(limitlock);
#endif
#else
static inline void user_cpufreq_ceiling_lock(int level, int val)
{
	if (val == 1 && !is_perf_lock_active(&user_cpufreq_ceiling[level]))
		perf_lock(&user_cpufreq_ceiling[level]);
	if (val == 0 && is_perf_lock_active(&user_cpufreq_ceiling[level]))
		perf_unlock(&user_cpufreq_ceiling[level]);
}

#define ceiling_level_wrapper(off, on, level) \
	case off:\
		user_cpufreq_ceiling_lock(level, 0);\
		break;\
	case on:\
		user_cpufreq_ceiling_lock(level, 1);\
		break;

static ssize_t
cpufreq_ceiling_store(struct kobject *kobj, struct kobj_attribute *attr,
		const char *buf, size_t n)
{
	int val;

	if (sscanf(buf, "%d", &val) > 0) {
		switch (val){
		ceiling_level_wrapper(0, 1, CEILING_LEVEL_HIGHEST);
		ceiling_level_wrapper(2, 3, CEILING_LEVEL_HIGH);
		ceiling_level_wrapper(4, 5, CEILING_LEVEL_MEDIUM);
		default:
			/* no matching level found */
			break;
		}
		return n;
	}

	return -EINVAL;
}
power_attr(cpufreq_ceiling);
#endif
#endif

static struct attribute *g[] = {
	&state_attr.attr,
#ifdef CONFIG_PM_TRACE
	&pm_trace_attr.attr,
	&pm_trace_dev_match_attr.attr,
#endif
#ifdef CONFIG_PM_SLEEP
	&pm_async_attr.attr,
	&wakeup_count_attr.attr,
#ifdef CONFIG_PM_AUTOSLEEP
	&autosleep_attr.attr,
#endif
#ifdef CONFIG_PM_WAKELOCKS
	&wake_lock_attr.attr,
	&wake_unlock_attr.attr,
#endif
	&touch_event_attr.attr,
	&touch_event_timer_attr.attr,
#ifdef CONFIG_PM_DEBUG
	&pm_test_attr.attr,
#endif
#ifdef CONFIG_USER_WAKELOCK
	&wake_lock_attr.attr,
	&wake_unlock_attr.attr,
#endif
#ifdef CONFIG_PM_SLEEP_DEBUG
	&pm_print_times_attr.attr,
#endif
#endif
#ifdef CONFIG_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
	&perflock_attr.attr,
	&limitlock_attr.attr,
#endif
#else
	&perflock_attr.attr,
	&cpufreq_ceiling_attr.attr,
#endif
#endif
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = g,
};

#ifdef CONFIG_PM_RUNTIME
struct workqueue_struct *pm_wq;
EXPORT_SYMBOL_GPL(pm_wq);

static int __init pm_start_workqueue(void)
{
	pm_wq = alloc_workqueue("pm", WQ_FREEZABLE, 0);

	return pm_wq ? 0 : -ENOMEM;
}
#else
static inline int pm_start_workqueue(void) { return 0; }
#endif

static int __init pm_init(void)
{
	int error = pm_start_workqueue();
#ifdef CONFIG_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
	int i;
	static char perflock_name[PERF_LOCK_INVALID][25];
	static char limitlock_name[PERF_LOCK_INVALID][26];
#endif
#else
	int i;
	char buf[38];
#endif
#endif
	if (error)
		return error;
	hibernate_image_size_init();
	hibernate_reserved_size_init();

	touch_evt_timer_val = ktime_set(2, 0);
	hrtimer_init(&tc_ev_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	tc_ev_timer.function = &tc_ev_stop;
	tc_ev_processed = 1;

#ifdef CONFIG_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
#ifdef CONFIG_SHSYS_CUST_USER_PERFLOCK
	pm_qos_add_request(&user_perf_lock_qos, PM_QOS_CPU_DMA_LATENCY, PM_QOS_DEFAULT_VALUE);
	for (i = 0; i < PERF_LOCK_INVALID; i++) {
		snprintf(perflock_name[i], 24, "User Perflock level(%d)", i);
		perflock_name[i][24] = '\0';
		perf_lock_init(&user_perf_lock[i], i, perflock_name[i]);
		snprintf(limitlock_name[i], 25, "User Limitlock level(%d)", i);
		limitlock_name[i][25] = '\0';
		limit_lock_init(&user_limit_lock[i], i, limitlock_name[i]);
	}
#endif
#else
	perf_lock_init(&user_perf_lock, PERF_LOCK_HIGHEST, "User Perflock");
	for (i = 0; i < CEILING_LEVEL_INVALID; i++) {
		snprintf(buf, 37, "User cpufreq_ceiling lock level(%d)", i);
		buf[37] = '\0';
		perf_lock_init_v2(&user_cpufreq_ceiling[i], i, buf);
	}
#endif
#endif

	power_kobj = kobject_create_and_add("power", NULL);
	if (!power_kobj)
		return -ENOMEM;
	error = sysfs_create_group(power_kobj, &attr_group);
	if (error)
		return error;
	return pm_autosleep_init();
}

core_initcall(pm_init);
