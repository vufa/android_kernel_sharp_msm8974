/*
 * kernel/power/autosleep.c
 *
 * Opportunistic sleep support.
 *
 * Copyright (C) 2012 Rafael J. Wysocki <rjw@sisk.pl>
 */

#include <linux/device.h>
#include <linux/mutex.h>
#include <linux/pm_wakeup.h>

#include "power.h"

#ifdef CONFIG_SHSYS_CUST_DEBUG
#include <linux/rtc.h>
#include <linux/module.h>
enum {
	SH_DEBUG_SUSPEND = 1U << 0,
};

static int sh_debug_mask = 0;
module_param_named(
	sh_debug_mask, sh_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP
);
#endif /* CONFIG_SHSYS_CUST_DEBUG */

#ifdef CONFIG_SH_SLEEP_LOG
#include <sharp/sh_sleeplog.h>
#endif

static suspend_state_t autosleep_state;
static struct workqueue_struct *autosleep_wq;
/*
 * Note: it is only safe to mutex_lock(&autosleep_lock) if a wakeup_source
 * is active, otherwise a deadlock with try_to_suspend() is possible.
 * Alternatively mutex_lock_interruptible() can be used.  This will then fail
 * if an auto_sleep cycle tries to freeze processes.
 */
static DEFINE_MUTEX(autosleep_lock);
static struct wakeup_source *autosleep_ws;

static void try_to_suspend(struct work_struct *work)
{
	unsigned int initial_count, final_count;

	if (!pm_get_wakeup_count(&initial_count, true))
		goto out;

	mutex_lock(&autosleep_lock);

	if (!pm_save_wakeup_count(initial_count)) {
		mutex_unlock(&autosleep_lock);
		goto out;
	}

	if (autosleep_state == PM_SUSPEND_ON) {
		mutex_unlock(&autosleep_lock);
		return;
	}
	if (autosleep_state >= PM_SUSPEND_MAX)
		hibernate();
	else
		pm_suspend(autosleep_state);

	mutex_unlock(&autosleep_lock);

	if (!pm_get_wakeup_count(&final_count, false))
		goto out;

	/*
	 * If the wakeup occured for an unknown reason, wait to prevent the
	 * system from trying to suspend and waking up in a tight loop.
	 */
	if (final_count == initial_count)
#ifdef CONFIG_SHSYS_CUST_DEBUG
	{
		if (sh_debug_mask & SH_DEBUG_SUSPEND)
			pr_info("%s: pm_suspend returned with no event\n", __func__);
#endif /* CONFIG_SHSYS_CUST_DEBUG */
		schedule_timeout_uninterruptible(HZ / 2);
#ifdef CONFIG_SHSYS_CUST_DEBUG
	}
#endif /* CONFIG_SHSYS_CUST_DEBUG */

 out:
	queue_up_suspend_work();
}

static DECLARE_WORK(suspend_work, try_to_suspend);

void queue_up_suspend_work(void)
{
	if (!work_pending(&suspend_work) && autosleep_state > PM_SUSPEND_ON)
		queue_work(autosleep_wq, &suspend_work);
}

suspend_state_t pm_autosleep_state(void)
{
	return autosleep_state;
}

int pm_autosleep_lock(void)
{
	return mutex_lock_interruptible(&autosleep_lock);
}

void pm_autosleep_unlock(void)
{
	mutex_unlock(&autosleep_lock);
}

int pm_autosleep_set_state(suspend_state_t state)
{
#if defined(CONFIG_SH_SLEEP_LOG) || defined(CONFIG_SHSYS_CUST_DEBUG)
	struct timespec ts;
#endif
#ifndef CONFIG_HIBERNATION
	if (state >= PM_SUSPEND_MAX)
		return -EINVAL;
#endif

	__pm_stay_awake(autosleep_ws);

	mutex_lock(&autosleep_lock);

#if defined(CONFIG_SH_SLEEP_LOG) || defined(CONFIG_SHSYS_CUST_DEBUG)
	getnstimeofday(&ts);
#endif
#ifdef CONFIG_SH_SLEEP_LOG
	sh_set_screen_state(ts, state);
#endif
#ifdef CONFIG_SHSYS_CUST_DEBUG
	if (sh_debug_mask & SH_DEBUG_SUSPEND) {
		struct rtc_time tm;
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			state > PM_SUSPEND_ON ? "sleep" : "wakeup",
			autosleep_state, state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
	}
#endif /* CONFIG_SHSYS_CUST_DEBUG */
	autosleep_state = state;

	__pm_relax(autosleep_ws);

	if (state > PM_SUSPEND_ON) {
#ifdef CONFIG_SHSYS_CUST_DEBUG
		print_active_locks();
#endif /* CONFIG_SHSYS_CUST_DEBUG */
		pm_wakep_autosleep_enabled(true);
		queue_up_suspend_work();
	} else {
		pm_wakep_autosleep_enabled(false);
	}

	mutex_unlock(&autosleep_lock);
	return 0;
}

int __init pm_autosleep_init(void)
{
	autosleep_ws = wakeup_source_register("autosleep");
	if (!autosleep_ws)
		return -ENOMEM;

	autosleep_wq = alloc_ordered_workqueue("autosleep", 0);
	if (autosleep_wq)
		return 0;

	wakeup_source_unregister(autosleep_ws);
	return -ENOMEM;
}
