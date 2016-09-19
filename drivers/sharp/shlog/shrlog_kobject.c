/* drivers/sharp/shrlog/shrlog_kobject.c
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
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <sharp/shrlog.h>

/*==============================================================================
    Constants
==============================================================================*/
#define MAX_REGS_BUFF_SIZE 320
#define MAX_INFO_BUFF_SIZE 128
#define MAX_MAPS_BUFF_SIZE 384
#define MAX_STACK_BUFF_SIZE 1024
#define MAX_PID_BUFF_SIZE 8

static char regs_log[MAX_REGS_BUFF_SIZE];
static char info_log[MAX_INFO_BUFF_SIZE];
static char maps_log[MAX_MAPS_BUFF_SIZE];
static char stack_log[MAX_STACK_BUFF_SIZE];
static char pid[MAX_PID_BUFF_SIZE];

static int regs_log_counter = 0;
static int info_log_counter = 0;
static int maps_log_counter = 0;
static int stack_log_counter = 0;

static void rlog_release( struct kobject *kobj );
static ssize_t rlog_sysfs_show( struct kobject *kobj, struct attribute *attr, char *buf );
static ssize_t rlog_sysfs_store( struct kobject *kobj, struct attribute *attr, const char *buf, size_t count );

typedef struct {
    const char *name;
    struct kobject *kobj;
    struct kobj_type *ktype;
} rlog_data;

static struct kset *rlog_kset;
static struct kobject rlog_kobj;

static struct sysfs_ops rlog_sysfs_ops = {
    .show  = rlog_sysfs_show,
    .store = rlog_sysfs_store,
};

static struct kobj_attribute init_attribute =
    __ATTR( init, 0600, NULL, NULL );

static struct kobj_attribute regs_attribute =
    __ATTR( regs, 0644, NULL, NULL );

static struct kobj_attribute info_attribute =
    __ATTR( info, 0644, NULL, NULL );

static struct kobj_attribute maps_attribute =
    __ATTR( maps, 0644, NULL, NULL );

static struct kobj_attribute stack_attribute =
    __ATTR( stack, 0644, NULL, NULL );

static struct kobj_attribute pid_attribute =
    __ATTR( pid, 0644, NULL, NULL );

static struct attribute *rlog_attrs[] = {
    &init_attribute.attr,
    &regs_attribute.attr,
    &info_attribute.attr,
    &maps_attribute.attr,
    &stack_attribute.attr,
    &pid_attribute.attr,
    NULL,
};

static struct kobj_type rlog_ktype = {
    .release = rlog_release,
    .sysfs_ops = &rlog_sysfs_ops,
    .default_attrs = rlog_attrs,
};
static rlog_data data = {
    .name  = "data",
    .kobj  = &rlog_kobj,
    .ktype = &rlog_ktype,
};

/*==============================================================================
    Fanctions
==============================================================================*/
static void rlog_release( struct kobject *kobj )
{
    kfree( kobj );
}

static ssize_t rlog_sysfs_show( struct kobject *kobj, struct attribute *attr,
                                char *buf )
{
    int ret = -1;
    if( buf == NULL ){
        return ret;
    }

    if( strcmp(attr->name, "regs") == 0 ){
        ret = snprintf( buf, MAX_REGS_BUFF_SIZE, "%s", regs_log );
    }
    else if( strcmp(attr->name, "info") == 0 ){
        ret = snprintf( buf, MAX_INFO_BUFF_SIZE, "%s", info_log );
    }
    else if( strcmp(attr->name, "maps") == 0 ){
        ret = snprintf( buf, MAX_MAPS_BUFF_SIZE, "%s", maps_log );
    }
    else if( strcmp(attr->name, "stack") == 0 ){
        ret = snprintf( buf, MAX_STACK_BUFF_SIZE, "%s", stack_log );
    }
    else if( strcmp(attr->name, "pid") == 0 ){
        ret = snprintf( buf, MAX_PID_BUFF_SIZE, "%s", pid );
    }

    return ret;
}

static ssize_t rlog_sysfs_store( struct kobject *kobj, struct attribute *attr,
                                 const char *buf, size_t count )
{
    return 0;
}

/*==============================================================================
[Function]
    rlog_sys_write
==============================================================================*/
int rlog_sys_write( const char *attr_name, char *buf, int size )
{
    int ret_size = -1;

    if( attr_name == NULL || buf == NULL ){
        printk( "path or buff is NULL\n" );
        return ret_size;
    }

    if( buf == NULL ){
        return ret_size;
    }

    if( strcmp(attr_name, "regs") == 0 ){
        if( strlen( buf ) <= MAX_REGS_BUFF_SIZE - regs_log_counter ){
            ret_size = snprintf( &regs_log[regs_log_counter], MAX_REGS_BUFF_SIZE, "%s", buf );
            regs_log_counter += ret_size;
            return strlen(buf);
        }
    }
    else if( strcmp(attr_name, "info") == 0 ){
        if( strlen( buf ) <= MAX_INFO_BUFF_SIZE - info_log_counter ){
            ret_size = snprintf( &info_log[info_log_counter], MAX_INFO_BUFF_SIZE, "%s", buf );
            info_log_counter += ret_size;
            return strlen(buf);
        }
    }
    else if( strcmp(attr_name, "maps") == 0 ){
        if( strlen( buf ) <= MAX_MAPS_BUFF_SIZE - maps_log_counter ){
            ret_size = snprintf( &maps_log[maps_log_counter], MAX_MAPS_BUFF_SIZE, "%s", buf );
            maps_log_counter += ret_size;
            return strlen(buf);
        }
    }
    else if( strcmp(attr_name, "stack") == 0 ){
        if( strlen( buf ) <= MAX_STACK_BUFF_SIZE - stack_log_counter ){
            ret_size = snprintf( &stack_log[stack_log_counter], MAX_STACK_BUFF_SIZE, "%s", buf );
            stack_log_counter += ret_size;
            return strlen(buf);
        }
    }
    else if( strcmp(attr_name, "pid") == 0 ){
        ret_size = snprintf( pid, MAX_PID_BUFF_SIZE, "%s", buf );
        return strlen(buf);
    }
    else if( strcmp(attr_name, "init") == 0 ){
        regs_log_counter = 0;
        info_log_counter = 0;
        maps_log_counter = 0;
        stack_log_counter = 0;
        memset( regs_log, 0x00, sizeof(regs_log) );
        memset( info_log, 0x00, sizeof(info_log) );
        memset( maps_log, 0x00, sizeof(maps_log) );
        memset( stack_log, 0x00, sizeof(stack_log) );
        memset( pid, 0x00, sizeof(pid) );
        ret_size = 0;
    }

    return ret_size;
}

int rlog_uevent( void )
{
    return kobject_uevent( data.kobj, KOBJ_CHANGE );
}

static int __init rlog_init( void )
{
    int ret;

    /* Create a kset with the name of "rlog" */
    /* located under /sys/kernel/ */
    rlog_kset = kset_create_and_add( "rlog", NULL, kernel_kobj );
    if( !rlog_kset ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        return -ENOMEM;
    }

    data.kobj->kset = rlog_kset;
    ret = kobject_init_and_add( data.kobj, data.ktype, NULL, "%s", data.name );
    if( ret ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        kobject_put( data.kobj );
    }

    memset( regs_log, 0x00, sizeof(regs_log) );
    memset( info_log, 0x00, sizeof(info_log) );
    memset( maps_log, 0x00, sizeof(maps_log) );
    memset( stack_log, 0x00, sizeof(stack_log) );
    memset( pid, 0x00, sizeof(pid) );
    regs_log_counter = 0;
    info_log_counter = 0;
    maps_log_counter = 0;
    stack_log_counter = 0;

    rlog_fault_init();

    return ret;
}

static void __exit rlog_exit( void )
{
    kset_unregister( rlog_kset );
}

module_init( rlog_init );
module_exit( rlog_exit );
MODULE_LICENSE("GPL");
