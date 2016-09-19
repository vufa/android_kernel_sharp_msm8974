/* drivers/sharp/shlog/shrlog.c
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

/*==============================================================================
    Includes
==============================================================================*/
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/gfp.h>
#include <linux/slab.h>
#include <sharp/shrlog.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <asm-generic/sections.h>
#include <asm/unwind.h>

/*==============================================================================
    Constants
==============================================================================*/
#define PINFO_KALLOC_SIZE 256
#define PMAPS_KALLOC_SIZE 512
#define PMEM_KALLOC_SIZE  320
#define STACK_KALLOC_SIZE 2048

static int pid_hist[2] = {0, 0};
static struct semaphore rlog_fautl_sem;
static int rlog_fautl_sem_flag = 0;
static shrlog_fixed_apps_info fixed_info;
static struct shrlog_ram_fixed_T *shrlog_ram_fixed;

extern unsigned long get_xtime_addr(void);
extern unsigned long get_log_buf_addr(void);
extern unsigned long get_log_end_addr(void);
extern unsigned long get_buf_log_main_addr(void);
extern unsigned long get_buf_log_events_addr(void);
extern unsigned long get_origin_unwind_addr(void);
extern struct unwind_idx __start_unwind_idx[];
extern struct unwind_idx __stop_unwind_idx[];
extern struct task_struct *latest_process[];
/*==============================================================================
    Private Prototype Definitions
==============================================================================*/
static void rlog_reginfo_to_user( struct pt_regs *regs );
static int  rlog_sys_read( const char *path, char *buff, int size );
static int  rlog_pinfo_to_user( pid_t pid, pid_t tgid, unsigned int fsr,
                                unsigned int sig, int code, unsigned long addr );
static void rlog_pmaps_to_user( pid_t tgid, long lr, long pc, unsigned long addr );
static void rlog_check_pc_lr( char *buff, long lr, long pc, unsigned long addr );
static void rlog_stack_to_user( long sp );
static void signame_get( unsigned int sig, char *signame );

/*==============================================================================
[Function]
    rlog_app_start
==============================================================================*/
int rlog_app_start( struct task_struct *tsk, unsigned long addr,
                    unsigned int fsr, unsigned int sig, int code,
                    struct pt_regs *regs )
{
    int ret = -1;
    int tmp_pid;
    char dummy = 'a';
    char pid[8];

    if( !rlog_fautl_sem_flag ){
        return ret;
    }

    if( down_interruptible(&rlog_fautl_sem) ){
        printk( "%s down_interruptible for write failed\n", __FUNCTION__ );
        return ret;
    }

    if( tsk->pid != pid_hist[0] && tsk->pid != pid_hist[1] ){
        pid_hist[0] = pid_hist[1];
        pid_hist[1] = tsk->pid;
    }
    else if( tsk->pid == pid_hist[1] ){
        tmp_pid = pid_hist[0];
        pid_hist[0] = pid_hist[1];
        pid_hist[1] = tmp_pid;
        up( &rlog_fautl_sem );
        return ret;
    }
    else {
        up( &rlog_fautl_sem );
        return ret;
    }

    up( &rlog_fautl_sem );

    rlog_sys_write( "init", &dummy, 1 );
    memset( pid, 0x00, sizeof(pid) );
    snprintf( pid, 7, "%d", tsk->tgid );
    rlog_sys_write( "pid", pid, strlen(pid) );
    rlog_reginfo_to_user( regs );
    ret = rlog_pinfo_to_user( tsk->pid, tsk->tgid, fsr, sig, code, addr );
    rlog_pmaps_to_user( tsk->tgid, regs->uregs[14], regs->uregs[15], addr );
    rlog_stack_to_user( regs->uregs[13] );
    if( !ret ){
        ret = rlog_uevent();
    }

    return ret;
}

/*==============================================================================
[Function]
    rlog_reginfo_to_user
==============================================================================*/
static void rlog_reginfo_to_user( struct pt_regs *regs )
{
    char *buff;
    int i, ret_count, count = 0;

    if( regs == NULL ){
        printk( "[RLOG]pt_regs is NULL\n" );
        return;
    }

    buff = kzalloc( PMEM_KALLOC_SIZE, GFP_KERNEL );
    if( !buff ){
        printk( "[RLOG]kzalloc failure\n" );
        return;
    }

    for( i = 0; i < 17; i++ ){
        switch( i ){
        case 10:
            ret_count = sprintf( &buff[count], "%d:0x%08lx\n", i, regs->uregs[i] );
            break;
        case 11:
            ret_count = sprintf( &buff[count], "fp:0x%08lx\n", regs->uregs[i] );
            break;
        case 12:
            ret_count = sprintf( &buff[count], "ip:0x%08lx\n", regs->uregs[i] );
            break;
        case 13:
            ret_count = sprintf( &buff[count], "sp:0x%08lx\n", regs->uregs[i] );
            break;
        case 14:
            ret_count = sprintf( &buff[count], "lr:0x%08lx\n", regs->uregs[i] );
            break;
        case 15:
            ret_count = sprintf( &buff[count], "pc:0x%08lx\n", regs->uregs[i] );
            break;
        case 16:
            ret_count = sprintf( &buff[count], "cpsr:0x%08lx\n", regs->uregs[i] );
            break;
        default :
            ret_count = sprintf( &buff[count], "r%d:0x%08lx\n", i, regs->uregs[i] );
            break;
        }
        count += ret_count;
    }

    rlog_sys_write( "regs", buff, strlen(buff) );
    kfree( buff );
}

/*==============================================================================
[Function]
    rlog_pinfo_to_user
==============================================================================*/
static int rlog_pinfo_to_user( pid_t pid, pid_t tgid, unsigned int fsr,
                               unsigned int sig, int code, unsigned long addr )
{
    char *buff;
    char path[24];
    char signame[16];
    int size;
    int ret = 0;

    memset( signame, 0x00, sizeof(signame) );
    signame_get( sig, signame );

    buff = kzalloc( PINFO_KALLOC_SIZE, GFP_KERNEL );
    if( !buff ){
        printk( "[RLOG]kzalloc failure\n" );
        return ret;
    }
    size = sprintf( buff, "fault address:0x%08lx sig:%s\npid:%d tgid:%d\n",
                    addr, signame, pid, tgid );

    memset( path, 0x00, sizeof(path) );
    snprintf( path, 23, "/proc/%d/cmdline", pid );
    rlog_sys_read( path, &buff[size], PINFO_KALLOC_SIZE - size - 1 );
    if( !strncmp(&buff[size], "rlog", 4) ){
        ret = 1;
    }

    rlog_sys_write( "info", buff, strlen(buff) );
    kfree( buff );

    return ret;
}

/*==============================================================================
[Function]
    rlog_pmaps_to_user
==============================================================================*/
static void rlog_pmaps_to_user( pid_t tgid, long lr, long pc, unsigned long addr )
{
    struct file *filp;
    mm_segment_t fs;
    char *buff = NULL;
    char *line_buff = NULL;
    char path[24];
    int i;
    int read_size = 0;
    int line_size = 0;
    int pre_size = 0;

    memset( path, 0x00, sizeof(path) );
    snprintf( path, 23, "/proc/%d/maps", tgid );

    filp = filp_open( path, O_RDONLY , 0 );
    if( IS_ERR(filp) ){
        printk( "[RLOG]can't open %s\n", path );
        return;
    }

    fs = get_fs();
    set_fs( get_ds() );

    buff = kzalloc( PMAPS_KALLOC_SIZE, GFP_KERNEL );
    if( !buff ){
        printk( "[RLOG]kzalloc failure\n" );
        return;
    }
    line_buff = kzalloc( 512, GFP_KERNEL );
    if( !line_buff ){
        printk( "[RLOG]kzalloc failure\n" );
        kfree( buff );
        return;
    }

    do{
        memset( buff, 0x00, PMAPS_KALLOC_SIZE );
        read_size = filp->f_op->read( filp, buff, PMAPS_KALLOC_SIZE, &filp->f_pos );
        if( read_size <= 0 ){
            break;
        }
        for( i = 0; i < read_size; i++ ){
            if( buff[i] == '\n' ){
                memcpy( &line_buff[pre_size], &buff[i-line_size], line_size - pre_size + 1 );
                rlog_check_pc_lr( line_buff, lr, pc, addr );
                line_size = 0;
                pre_size = 0;
                memset( line_buff, 0x00, 512 );
            }
            else {
                if( line_size < 250 ){
                    line_size++;
                }
            }
        }
        if( read_size == PMAPS_KALLOC_SIZE &&
            line_size != 0                 ){
            memcpy( line_buff, &buff[i-line_size], line_size );
            pre_size = line_size;
            line_size = 0;
        }
    }while( read_size == PMAPS_KALLOC_SIZE );

    set_fs( fs );
    filp_close( filp, NULL );
    kfree( line_buff );
    kfree( buff );
}

/*==============================================================================
[Function]
    rlog_check_pc_lr
==============================================================================*/
static void rlog_check_pc_lr( char *buff, long lr, long pc, unsigned long addr )
{
    unsigned long before, after;
    char str[9];

    /* /proc/[pid]/maps */
    /* address           perms offset  dev   inode      pathname                */
    /* 00008000-00009000 r-xp 00000000 1f:04 404        /system/bin/app_process */
    /* 012345678901234567890123456789012345678901234567890123456789012345678901 */

    if( strlen(buff) < 16 ){
        return;
    }

    memset( str, 0x00, sizeof(str) );
    memcpy( str, buff, 8 );
    before = simple_strtoul( str, NULL, 16 );

    memset( str, 0x00, sizeof(str) );
    memcpy( str, &buff[9], 8 );
    after = simple_strtoul( str, NULL, 16 );

    if( before <= (unsigned long)pc && after >= (unsigned long)pc ){
        rlog_sys_write( "maps", buff, strlen(buff) );
    }
    else if( before <= (unsigned long)lr && after >= (unsigned long)lr ){
        rlog_sys_write( "maps", buff, strlen(buff) );
    }
    else if( before <= addr && after >= addr ){
        rlog_sys_write( "maps", buff, strlen(buff) );
    }
}

/*==============================================================================
[Function]
    rlog_stack_to_user
==============================================================================*/
static void rlog_stack_to_user( long sp )
{
    char *buff;
    int i;
    unsigned long *p;
    unsigned long val[4];
    int size = 0;
    int couner = 0;

    buff = kzalloc( STACK_KALLOC_SIZE, GFP_KERNEL );
    if( !buff ){
        printk( "[RLOG]kzalloc failure\n" );
        return;
    }
    if( (unsigned long )sp < 48 ){
        sprintf( &buff[0], "no stack\n" );
    }
    else{
        p = (unsigned long *)sp;

        for( i = 0; i < 16; i++ ){
            if( get_user(val[0], (unsigned long __user *)(p))   ||
                get_user(val[1], (unsigned long __user *)(p+1)) ||
                get_user(val[2], (unsigned long __user *)(p+2)) ||
                get_user(val[3], (unsigned long __user *)(p+3)) ){
                break;
            }
            size = sprintf( &buff[couner], "%p %08lx %08lx %08lx %08lx\n",
                            p, val[0], val[1], val[2], val[3] );
            p += 4;
            couner += size;
        }
        if( couner == 0 ){
            sprintf( &buff[0], "stack ptr err\n" );
        }
    }
    rlog_sys_write( "stack", buff, strlen(buff) );

    kfree( buff );
}

/*==============================================================================
[Function]
    rlog_sys_read
==============================================================================*/
static int rlog_sys_read( const char *path, char *buff, int size )
{
    struct file *filp;
    mm_segment_t fs;
    int read_size;

    if( path == NULL || buff == NULL ){
        printk( "[RLOG]path or buff is NULL\n" );
        return -1;
    }

    filp = filp_open( path, O_RDONLY, 0 );
    if( IS_ERR(filp) ){
        printk( "[RLOG]can't open %s\n", path );
        return -1;
    }
    fs = get_fs();
    set_fs( get_ds() );
    read_size = filp->f_op->read( filp, buff, size, &filp->f_pos );

    set_fs( fs );
    filp_close( filp, NULL );

    return read_size;
}

/*==============================================================================
[Function]
    signame_get
==============================================================================*/
static void signame_get( unsigned int sig, char *signame )
{
    switch( sig ){
    case SIGILL:
        strcpy( signame, "SIGILL" );
        break;
    case SIGABRT:
        strcpy( signame, "SIGABRT" );
        break;
    case SIGBUS:
        strcpy( signame, "SIGBUS" );
        break;
    case SIGFPE:
        strcpy( signame, "SIGFPE" );
        break;
    case SIGSEGV:
        strcpy( signame, "SIGSEGV" );
        break;
    case SIGSTKFLT:
        strcpy( signame, "SIGSTKFLT" );
        break;
    default:
        sprintf( signame, "%d", sig );
        break;
    }
}

void rlog_fault_init( void )
{
    sema_init( &rlog_fautl_sem, 1 );
    rlog_fautl_sem_flag = 1;
}

/*==============================================================================
[Function]
    shrlog_set_fixed_info
==============================================================================*/
static int __init shrlog_set_fixed_info( void )
{
    fixed_info.init_task_addr          = (unsigned long)(&init_task);
    fixed_info.xtime_addr              = get_xtime_addr();
    fixed_info.__log_buf_addr          = get_log_buf_addr();
    fixed_info.log_end_addr            = get_log_end_addr();
    fixed_info._buf_log_main_addr      = get_buf_log_main_addr();
    fixed_info._text_addr              = (unsigned long)(_text);
    fixed_info._stext_addr             = (unsigned long)(_stext);
    fixed_info._etext_addr             = (unsigned long)(_etext);
    fixed_info.__start_unwind_idx_addr = (unsigned long)__start_unwind_idx;
    fixed_info.__origin_unwind_idx_addr= get_origin_unwind_addr();
    fixed_info.__stop_unwind_idx_addr  = (unsigned long)__stop_unwind_idx;
    fixed_info._buf_log_events_addr    = get_buf_log_events_addr();
    fixed_info.latest_process_addr     = (unsigned long)latest_process;
    fixed_info.stack_offset        = offsetof(struct task_struct, stack);
    fixed_info.tasks_offset        = offsetof(struct task_struct, tasks);
    fixed_info.pid_offset          = offsetof(struct task_struct, pid);
    fixed_info.thread_group_offset = offsetof(struct task_struct, thread_group);
    fixed_info.comm_offset         = offsetof(struct task_struct, comm);
    fixed_info.memory_hole_end         = (unsigned long)memory_hole_end;
    fixed_info.memory_hole_offset      = (unsigned long)memory_hole_offset;
    fixed_info.memory_hole_align       = memory_hole_align;

    shrlog_ram_fixed = (struct shrlog_ram_fixed_T *)SHRLOG_FIXED_APPS_ADDRESS;
    shrlog_ram_fixed->shrlog_ram_fixed_addr = (unsigned long*)(&fixed_info);
    shrlog_ram_fixed->magic_num = SHRLOG_FIXED_MAGIC_NUM; // magic number
    return 0;
}
late_initcall(shrlog_set_fixed_info);
