/* arch/arm/mach-msm/perflock.h
 *
 * MSM performance lock driver header
 *
 * Copyright (C) 2008 HTC Corporation
 * Author: Eiven Peng <eiven_peng@htc.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ARCH_ARM_MACH_PERF_LOCK_H
#define __ARCH_ARM_MACH_PERF_LOCK_H

#include <linux/list.h>
#include <linux/cpufreq.h>

/*
 * Performance level determine differnt EBI1 rate
 */

enum {
	TYPE_PERF_LOCK = 0,	/* default performance lock*/
	TYPE_CPUFREQ_CEILING,	/* cpufreq ceiling lock */
};

#ifdef CONFIG_SHSYS_CUST_PERFLOCK
enum {
	PERF_LOCK_300000KHz,	/* PERF_LEVEL0 */
	PERF_LOCK_422400KHz,	/* PERF_LEVEL1 */
	PERF_LOCK_652800KHz,	/* PERF_LEVEL2 */
	PERF_LOCK_729600KHz,	/* PERF_LEVEL3 */
	PERF_LOCK_883200KHz,	/* PERF_LEVEL4 */
	PERF_LOCK_960000KHz,	/* PERF_LEVEL5 */
	PERF_LOCK_1036800KHz,	/* PERF_LEVEL6 */
	PERF_LOCK_1190400KHz,	/* PERF_LEVEL7 */
	PERF_LOCK_1267200KHz,	/* PERF_LEVEL8 */
	PERF_LOCK_1497600KHz,	/* PERF_LEVEL9 */
	PERF_LOCK_1574400KHz,	/* PERF_LEVEL10 */
	PERF_LOCK_1728000KHz,	/* PERF_LEVEL11 */
	PERF_LOCK_1958400KHz,	/* PERF_LEVEL12 */
	PERF_LOCK_2150400KHz,	/* PERF_LEVEL13 */
	PERF_LOCK_HIGHEST = PERF_LOCK_2150400KHz,
	PERF_LOCK_INVALID,	/* INVALID */
};
#else
enum {
	PERF_LOCK_LOWEST,	/* Lowest performance */
	PERF_LOCK_LOW,	/* Low performance */
	PERF_LOCK_MEDIUM,	/* Medium performance */
	PERF_LOCK_HIGH,	/* High performance */
	PERF_LOCK_HIGHEST,	/* Highest performance */
	PERF_LOCK_INVALID,
};

enum {
	CEILING_LEVEL_MEDIUM,	/* Medium ceiling level */
	CEILING_LEVEL_HIGH,	/* High ceiling level */
	CEILING_LEVEL_HIGHEST,	/* Highest ceiling level */
	CEILING_LEVEL_INVALID,
};
#endif

struct perf_lock {
	struct list_head link;
	unsigned int flags;
	unsigned int level;
	const char *name;
	unsigned int type;
};

struct perflock_platform_data {
	unsigned int *perf_acpu_table;
	unsigned int table_size;
};

#ifndef CONFIG_PERFLOCK
#ifndef CONFIG_SHSYS_CUST_PERFLOCK
static inline void __init perflock_init(
	struct perflock_platform_data *pdata) { return; }
static inline void __init cpufreq_ceiling_init(
	struct perflock_platform_data *pdata) { return; }
#endif
static inline void perf_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name) { return; }
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
static inline void limit_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name) { return; }
static inline void limit_lock(struct perf_lock *lock) { return; }
static inline void limit_unlock(struct perf_lock *lock) { return; }
#else
static inline void perf_lock_init_v2(struct perf_lock *lock,
	unsigned int level, const char *name) { return; }
#endif
static inline void perf_lock(struct perf_lock *lock) { return; }
static inline void perf_unlock(struct perf_lock *lock) { return; }
static inline int is_perf_lock_active(struct perf_lock *lock) { return 0; }
static inline int is_perf_locked(void) { return 0; }
static inline void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu) { return; }
static inline void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu) { return; }
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
static inline unsigned int get_perflock_acpu_table(unsigned int level) { return 0; }
static inline unsigned int get_limitlock_acpu_table(unsigned int level) { return 0; }
#else
static inline void htc_print_active_perf_locks(void) { return; }
#endif
static inline int perflock_override(const struct cpufreq_policy *policy) { return 0; }
#else
#ifndef CONFIG_SHSYS_CUST_PERFLOCK
extern void __init perflock_init(struct perflock_platform_data *pdata);
extern void __init cpufreq_ceiling_init(struct perflock_platform_data *pdata);
#endif
extern void perf_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name);
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
extern void limit_lock_init(struct perf_lock *lock,
	unsigned int level, const char *name);
extern void limit_lock(struct perf_lock *lock);
extern void limit_unlock(struct perf_lock *lock);
#else
extern void perf_lock_init_v2(struct perf_lock *lock,
	unsigned int level, const char *name);
#endif
extern void perf_lock(struct perf_lock *lock);
extern void perf_unlock(struct perf_lock *lock);
extern int is_perf_lock_active(struct perf_lock *lock);
extern int is_perf_locked(void);
extern void perflock_scaling_max_freq(unsigned int freq, unsigned int cpu);
extern void perflock_scaling_min_freq(unsigned int freq, unsigned int cpu);
extern int perflock_override(const struct cpufreq_policy *policy, const unsigned int new_freq);
#ifdef CONFIG_SHSYS_CUST_PERFLOCK
extern unsigned int get_perflock_acpu_table(unsigned int level);
extern unsigned int get_limitlock_acpu_table(unsigned int level);
#else
extern void htc_print_active_perf_locks(void);
#endif
#endif

#ifdef CONFIG_PERFLOCK_SUSPEND_LOCK
extern void shsys_enter_state_perf_lock(void);
extern void shsys_enter_state_perf_unlock(void);
#endif

#endif

