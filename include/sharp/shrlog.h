/* include/sharp/shrlog.h
 *
 * Copyright (C) 2010 Sharp Corporation
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

#ifndef _SHRLOG_H_
#define _SHRLOG_H_

#include <linux/sched.h>

/* 20120727 for get system info from smem */
#define SHRLOG_FIXED_MAGIC_NUM (0x88990011)
#define SHRLOG_FIXED_APPS_ADDRESS   (0xFA400000 + 0x001FFFF0)
struct shrlog_ram_fixed_T {
    unsigned long *shrlog_ram_fixed_addr;
    unsigned long magic_num;
};

typedef struct
{
    unsigned long init_task_addr;
    unsigned long xtime_addr;
    unsigned long __log_buf_addr;
    unsigned long log_end_addr;
    unsigned long _buf_log_main_addr;
    unsigned long _text_addr;
    unsigned long _stext_addr;
    unsigned long _etext_addr;
    unsigned long __start_unwind_idx_addr;
    unsigned long __origin_unwind_idx_addr;
    unsigned long __stop_unwind_idx_addr;
    unsigned long _buf_log_events_addr;
    unsigned long latest_process_addr;
    unsigned long stack_offset;
    unsigned long tasks_offset;
    unsigned long pid_offset;
    unsigned long thread_group_offset;
    unsigned long comm_offset;
    unsigned long memory_hole_end;
    unsigned long memory_hole_offset;
    unsigned long memory_hole_align;
} shrlog_fixed_apps_info;

extern int rlog_app_start( struct task_struct *tsk, unsigned long addr,
                           unsigned int fsr, unsigned int sig, int code,
                           struct pt_regs *regs );
extern int  rlog_sys_write( const char *attr_name, char *buf, int size );
extern int  rlog_uevent( void );
extern void rlog_fault_init( void );

#endif /* _SHRLOG_H_ */
