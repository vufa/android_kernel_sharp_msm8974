/* drivers/sharp/vold/shproc_kobject.c
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

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/kernel.h>
#include <linux/kobject.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/stat.h>
#include <linux/file.h>
#include <linux/fs_struct.h>
#include <linux/mm.h>
#include <linux/dirent.h>
#include <linux/unistd.h>
#include <asm/uaccess.h>
#include <linux/sched.h>
#include <linux/fdtable.h>
#include <linux/string.h>

#define SHPROC_BUFF_SIZE 96

static void shproc_release( struct kobject *kobj );
static ssize_t shproc_sysfs_show( struct kobject *kobj, struct attribute *attr, char *buf );
static ssize_t shproc_sysfs_store( struct kobject *kobj, struct attribute *attr, const char *buf, size_t count );

typedef struct {
    const char *name;
    struct kobject *kobj;
    struct kobj_type *ktype;
} shproc_data;

static struct kset *shproc_kset;
static struct kobject shproc_kobj;

static struct sysfs_ops shproc_sysfs_ops = {
    .show  = shproc_sysfs_show,
    .store = shproc_sysfs_store,
};

static struct kobj_attribute shproc_attribute =
    __ATTR( sd, 0600, NULL, NULL );
static struct kobj_attribute shproccwd_attribute =
    __ATTR( sdcwd, 0600, NULL, NULL );

static struct attribute *shproc_attrs[] = {
    &shproc_attribute.attr,
    &shproccwd_attribute.attr,
    NULL,
};

static struct kobj_type shproc_ktype = {
    .release = shproc_release,
    .sysfs_ops = &shproc_sysfs_ops,
    .default_attrs = shproc_attrs,
};
static shproc_data data = {
    .name  = "data",
    .kobj  = &shproc_kobj,
    .ktype = &shproc_ktype,
};

static void shproc_release( struct kobject *kobj )
{
    kfree( kobj );
}

static ssize_t shproc_sysfs_show( struct kobject *kobj, struct attribute *attr,
                                  char *buf )
{
    return 0;
}

static ssize_t shproc_sysfs_store( struct kobject *kobj, struct attribute *attr,
                                   const char *buf, size_t count )
{
    struct task_struct *p;
    int pid, len;
    unsigned int i;
    int ret = -1;
    char *buff = NULL;
    char *pathname;
    char *cmpstr = NULL;
    char from_user[SHPROC_BUFF_SIZE] = {0};
    int n;
    int len_cmp = 0;
    unsigned int max_fds;

    if( buf == NULL                    ||
        count > (SHPROC_BUFF_SIZE - 1) ){
        printk( "[vold] error\n" );
        return ret;
    }

    if( !memcpy( from_user, buf, count ) ){
        printk( "[vold]memcpy error\n" );
        return -EFAULT;
    }

    for( n = 0; n < count; n++ ){
        if( from_user[n] == ':' ){
            from_user[n] = 0x00;
            cmpstr = from_user + n + 1;
            break;
        }
    }
    if( cmpstr == NULL ){
        printk( "[vold]arg error\n" );
        return ret;
    }

    pid = simple_strtol( from_user, NULL, 10 );
    if( pid <= 0 ){
        printk( "[vold]pid(%d) error\n", pid );
        return ret;
    }
    len_cmp = strnlen( cmpstr, SHPROC_BUFF_SIZE - 10 );

    buff = kmalloc( PAGE_SIZE, GFP_KERNEL );
    if( !buff ){
        printk( "[vold]kmalloc error\n" );
        return ret;
    }
    memset( buff, 0x00, PAGE_SIZE );
    read_lock(&tasklist_lock);

    p = find_task_by_vpid( (pid_t)pid );
    if( !p ){
        read_unlock(&tasklist_lock);
        kfree( buff );
        printk( "[vold]task_struct get failed\n" );
        return ret;
    }

    /* for checking cwd symbolic link */
    if ( !strncmp(attr->name, "sdcwd", strlen("sdcwd")) ) {
        struct path path;

        if( p->fs == NULL         ||
            IS_ERR( p->fs ) ) {
            goto out;
        }

        get_fs_pwd(p->fs, &path);

        if ( path.dentry == NULL ||
             IS_ERR(path.dentry) ){
            path_put(&path);
            goto out;
        }

        pathname = d_path( &(path), buff, PAGE_SIZE );
        path_put(&path);

        ret = PTR_ERR(pathname);

        if( IS_ERR(pathname) ){
            goto out;
        }
        if (len_cmp < 1 ) {
            goto out;
        }

        if( !strncmp(pathname, cmpstr, len_cmp) ){
            printk( "[vold]find pid:%d, %s\n", pid, cmpstr);
            ret = len_cmp;
        }
    } else {

    if( p->files == NULL         ||
        IS_ERR( p->files )       ||
        p->files->fdt == NULL    ||
        IS_ERR( p->files->fdt )  ){
        read_unlock(&tasklist_lock);
        kfree( buff );
        printk( "[vold]task_struct NULL : task_struct get failed\n" );
        return ret;
    }

    max_fds = p->files->fdt->max_fds;
    for( i = 0; i < max_fds; i++ ){
        if( p->files == NULL                            ||
            IS_ERR(p->files)                            ||
            p->files->fdt == NULL                       ||
            IS_ERR(p->files->fdt)                       ||
            p->files->fdt->fd[i] == NULL                ||
            IS_ERR(p->files->fdt->fd[i])                ||
            p->files->fdt->fd[i]->f_path.dentry == NULL ||
            IS_ERR(p->files->fdt->fd[i]->f_path.dentry) ){
            continue;
        }
        pathname = d_path( &(p->files->fdt->fd[i]->f_path), buff, PAGE_SIZE );
        ret = PTR_ERR(pathname);
        if( IS_ERR(pathname) ){
            break;
        }
        len = strlen( pathname );
        if( len > len_cmp ){
            len = len_cmp;
        }
        if( !strncmp(pathname, cmpstr, len) ){
            printk( "[vold]find pid:%d, %s\n", pid, cmpstr);
            ret = len;
            break;
        }
        memset( buff, 0x00, PAGE_SIZE );
    }

    }

    read_unlock(&tasklist_lock);
    kfree( buff );
    return ret;

out:
    read_unlock(&tasklist_lock);
    kfree( buff );
    return ret;
}

static int __init shproc_init( void )
{
    int ret;

    /* Create a kset with the name of "shproc" */
    /* located under /sys/kernel/ */
    shproc_kset = kset_create_and_add( "shproc", NULL, kernel_kobj );
    if( !shproc_kset ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        return -ENOMEM;
    }

    data.kobj->kset = shproc_kset;
    ret = kobject_init_and_add( data.kobj, data.ktype, NULL, "%s", data.name );
    if( ret ){
        printk( "%s : line %d error\n", __FUNCTION__, __LINE__ );
        kobject_put( data.kobj );
    }

    return ret;
}

static void __exit shproc_exit( void )
{
    kset_unregister( shproc_kset );
}

module_init( shproc_init );
module_exit( shproc_exit );

