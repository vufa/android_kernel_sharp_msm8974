/*
 * MIYABI LSM module
 *
 * based on root_plug.c
 * Copyright (C) 2002 Greg Kroah-Hartman <greg@kroah.com>
 *
 * _xx_is_valid(), _xx_encode(), _xx_realpath_from_path()
 * is ported from security/tomoyo/realpath.c in linux-2.6.32 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/security.h>
#include <linux/moduleparam.h>
#include <linux/mount.h>
#include <linux/mnt_namespace.h>
#include <linux/fs_struct.h>
#include <linux/magic.h>
#include <linux/net.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#include <linux/binfmts.h>
#include <net/sock.h>

#include "miyabi_sandbox.h"

#if 1 
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <crypto/sha.h>
#endif

#include "../capability.c"

#if 1
static const unsigned char miyabi_scp0[SHA256_DIGEST_SIZE] = {0x62, 0xc3, 0x45, 0x19, 0x14, 0x89, 0x77, 0xc3, 0x4f, 0x26, 0x4d, 0x71, 0xb4, 0x68, 0x0d, 0x47, 0x38, 0x77, 0x5b, 0x63, 0xd1, 0x49, 0xa2, 0xd9, 0xcb, 0x24, 0x43, 0x2f, 0x84, 0x0b, 0xb1, 0x4d};
static const unsigned char miyabi_scp1[SHA256_DIGEST_SIZE] = {0x04, 0xa1, 0xbd, 0x25, 0xd5, 0xf2, 0xe4, 0x10, 0x78, 0xc8, 0xe9, 0xbc, 0x0b, 0x84, 0x7e, 0xfc, 0xc6, 0x0b, 0xfa, 0x69, 0xd7, 0x2f, 0xe6, 0xfe, 0x3e, 0xea, 0xe9, 0xe4, 0x83, 0x9b, 0x10, 0x59};
#endif

static void record_rooted(void)
{
	printk("!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
	printk("!!        rooted        !!\n");
	printk("!!!!!!!!!!!!!!!!!!!!!!!!!!\n");

	return;
}

static int miyabi_ptrace_access_check(struct task_struct *child, unsigned int mode)
{
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	return 0;
#else /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
#if 1
	int ret = -EPERM;

	char* pathbuff = NULL;
	char* path = NULL;
	struct crypto_shash *shash = NULL;
	struct shash_desc *desc = NULL;
	unsigned char hash[SHA256_DIGEST_SIZE];

	do
	{
		pathbuff = kmalloc(PATH_MAX + 1, GFP_KERNEL);

		if(pathbuff == NULL)
		{
			ret = -ENOMEM;

			break;
		}

		shash = crypto_alloc_shash("sha256", 0, 0);

		if(IS_ERR(shash))
		{
			ret = -ENOMEM;

			break;
		}

		desc = kmalloc(sizeof(*desc) + crypto_shash_descsize(shash), GFP_KERNEL);

		if(desc == NULL)
		{
			ret = -ENOMEM;

			break;
		}

		memset(pathbuff, 0, PATH_MAX + 1);
		memset(desc, 0, sizeof(*desc) + crypto_shash_descsize(shash));

		if(current->mm != NULL && current->mm->exe_file != NULL)
		{
			path = d_path(&current->mm->exe_file->f_path, pathbuff, PATH_MAX);
		}
	
		if(path == NULL || (long)path == ENAMETOOLONG) break;

		desc->tfm = shash;
		desc->flags = CRYPTO_TFM_REQ_MAY_SLEEP;

		crypto_shash_init(desc);
		crypto_shash_update(desc, path, strlen(path));
		crypto_shash_final(desc, hash);

		if(memcmp(hash, miyabi_scp0, SHA256_DIGEST_SIZE) == 0)
		{
			ret = 0;

			break;
		}
		else if(memcmp(hash, miyabi_scp1, SHA256_DIGEST_SIZE) == 0)
		{
			extern int mnhelper_get_uid(void);

			int uid = mnhelper_get_uid();

			if(uid > 1 && current_uid() == uid)
			{
				ret = 0;

				break;
			}
		}
	}
	while(0);

	if(desc != NULL)
	{
		kfree(desc);

		desc = NULL;
	}

	if(shash != NULL)
	{
		crypto_free_shash(shash);

		shash = NULL;
	}

	if(pathbuff != NULL)
	{
		kfree(pathbuff);

		pathbuff = NULL;
	}

	return ret;
#else
	return -EPERM;
#endif
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
}

static int miyabi_ptrace_traceme(struct task_struct *parent)
{
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	return 0;
#else /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	return -EPERM;
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
}

static struct vec
{
	const char *prefix;
}
wh[] =
{
	{ "/system/bin/run-as" },
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
	{ "/system/xbin/procrank" },
	{ "/system/xbin/librank" },
	{ "/system/xbin/su" },
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	{ 0 },
};

static int permitted_prog(const char *path)
{
	int len;
	struct vec *w = wh;

	for(; w->prefix; w++)
	{
		len = strlen(w->prefix);
		if(strlen(path) == len && !strncmp(w->prefix, path, len))
		{
			return 1;
		}
	}

	return 0;
}

static int miyabi_bprm_set_creds(struct linux_binprm *bprm)
{
	struct cred *new = bprm->cred;

	if((new->uid != 0 && new->euid == 0) ||
	   (new->gid != 0 && new->egid == 0))
	{
		if(!permitted_prog(bprm->filename))
		{
			record_rooted();
			return -EPERM;
		}
	}

	return cap_bprm_set_creds(bprm);
}

static char* _xx_encode(const char *str)
{
	int len = 0;
	const char *p = str;
	char *cp;
	char *cp0;

	if (!p)
		return NULL;
	while (*p) {
		const unsigned char c = *p++;
		if (c == '\\')
			len += 2;
		else if (c > ' ' && c < 127)
			len++;
		else
			len += 4;
	}
	len++;
	/* Reserve space for appending "/". */
	cp = kzalloc(len + 10, GFP_NOFS);
	if (!cp)
		return NULL;
	cp0 = cp;
	p = str;
	while (*p) {
		const unsigned char c = *p++;

		if (c == '\\') {
			*cp++ = '\\';
			*cp++ = '\\';
		} else if (c > ' ' && c < 127) {
			*cp++ = c;
		} else {
			*cp++ = '\\';
			*cp++ = (c >> 6) + '0';
			*cp++ = ((c >> 3) & 7) + '0';
			*cp++ = (c & 7) + '0';
		}
	}
	return cp0;
}

static char* _xx_realpath_from_path(struct path *path)
{
	char *buf = NULL;
	char *name = NULL;
	unsigned int buf_len = PAGE_SIZE / 2;
	struct dentry *dentry = path->dentry;
	bool is_dir;
	if (!dentry)
		return NULL;
	is_dir = dentry->d_inode && S_ISDIR(dentry->d_inode->i_mode);
	while (1) {
		char *pos;
		buf_len <<= 1;
		kfree(buf);
		buf = kmalloc(buf_len, GFP_NOFS);
		if (!buf)
			break;
		/* Get better name for socket. */
		if (dentry->d_sb && dentry->d_sb->s_magic == SOCKFS_MAGIC) {
			struct inode *inode = dentry->d_inode;
			struct socket *sock = inode ? SOCKET_I(inode) : NULL;
			struct sock *sk = sock ? sock->sk : NULL;
			if (sk) {
				snprintf(buf, buf_len - 1, "socket:[family=%u:"
					 "type=%u:protocol=%u]", sk->sk_family,
					 sk->sk_type, sk->sk_protocol);
			} else {
				snprintf(buf, buf_len - 1, "socket:[unknown]");
			}
			name = _xx_encode(buf);
			break;
		}
		/* For "socket:[\$]" and "pipe:[\$]". */
		if (dentry->d_op && dentry->d_op->d_dname) {
			pos = dentry->d_op->d_dname(dentry, buf, buf_len - 1);
			if (IS_ERR(pos))
				continue;
			name = _xx_encode(pos);
			break;
		}
		/* If we don't have a vfsmount, we can't calculate. */
		if (!path->mnt)
			break;
		pos = d_absolute_path(path, buf, buf_len - 1);
		/* If path is disconnected, use "[unknown]" instead. */
		if (pos == ERR_PTR(-EINVAL)) {
			name = _xx_encode("[unknown]");
			break;
		}
		/* Prepend "/proc" prefix if using internal proc vfs mount. */
		if (!IS_ERR(pos) && (path->mnt->mnt_flags & MNT_INTERNAL) &&
		    (path->mnt->mnt_sb->s_magic == PROC_SUPER_MAGIC)) {
			pos -= 5;
			if (pos >= buf)
				memcpy(pos, "/proc", 5);
			else
				pos = ERR_PTR(-ENOMEM);
		}
		if (IS_ERR(pos))
			continue;
		name = _xx_encode(pos);
		break;
	}
	kfree(buf);
	if (is_dir && name != NULL && *name) {
		/* Append trailing '/' if dentry is a directory. */
		char *pos = name + strlen(name) - 1;
		if (*pos != '/')
			/*
			 * This is OK because tomoyo_encode() reserves space
			 * for appending "/".
			 */
			*++pos = '/';
	}
	return name;
}


#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH
#define CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH "/system/"
#endif

#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR
#define CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR (179)
#endif /* ! CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR */

#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR
#define CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR (12)
#endif /* ! CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR */

#ifndef CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH
#define CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH "/factory/"
#endif

#ifndef CONFIG_SECURITY_MIYABI_FACTORY_DEV_MAJOR
#define CONFIG_SECURITY_MIYABI_FACTORY_DEV_MAJOR (179)
#endif /* ! CONFIG_SECURITY_MIYABI_FACTORY_DEV_MAJOR */

#ifndef CONFIG_SECURITY_MIYABI_FACTORY_DEV_MINOR
#define CONFIG_SECURITY_MIYABI_FACTORY_DEV_MINOR (2)
#endif /* ! CONFIG_SECURITY_MIYABI_FACTORY_DEV_MINOR */

#ifndef CONFIG_SECURITY_MIYABI_CACHE_DIR_PATH
#define CONFIG_SECURITY_MIYABI_CACHE_DIR_PATH "/chache/"
#endif

#ifndef CONFIG_SECURITY_MIYABI_CACHE_DEV_MAJOR
#define CONFIG_SECURITY_MIYABI_CACHE_DEV_MAJOR (179)
#endif /* ! CONFIG_SECURITY_MIYABI_CACHE_DEV_MAJOR */

#ifndef CONFIG_SECURITY_MIYABI_CACHE_DEV_MINOR
#define CONFIG_SECURITY_MIYABI_CACHE_DEV_MINOR (17)
#endif /* ! CONFIG_SECURITY_MIYABI_CACHE_DEV_MINOR */

static int miyabi_sb_mount(char *dev_name, struct path *path,
			    char *type, unsigned long flags, void *data)
{
#ifndef CONFIG_ANDROID_ENGINEERING
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	char* realpath;
	struct block_device* bdev;
	dev_t major, minor;
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */

	if(dev_name != NULL)
	{
		if(strncmp(dev_name, "rootfs", 6) == 0)
		{
			if ((flags & MS_REMOUNT) && (!(flags & MS_RDONLY)))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s cannot remount as writable\n", __FUNCTION__, dev_name);

				return -EPERM;
			}
		}
	}

#ifndef CONFIG_ANDROID_ENGINEERING
/* enable debugfs for systrace (in compliance with Android 4.2 CDD) */
/*
	if(type != NULL)
	{
		if(strncmp(type, "debugfs", 7) == 0)
		{
			printk(KERN_ERR "%s: REJECT dev_name=%s cannot mount %s\n", __FUNCTION__, dev_name, type);

			return -EPERM;
		}
	}
*/

#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	realpath = _xx_realpath_from_path(path);
	if(realpath == NULL)
	{
		return -ENOMEM;
	}


	if (strncmp(realpath, CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH,
		strlen(CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH)) == 0)
	{
		if (strcmp(realpath, CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH) == 0)
		{
			if ((flags & MS_REMOUNT) && (!(flags & MS_RDONLY)))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s ro remount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			if (flags & MS_BIND)
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s loopback mount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			bdev = lookup_bdev((const char*)dev_name);

			if( bdev == NULL )
			{
				printk("cannot lookup\n");

				kfree(realpath);
				return -EPERM;
			}

			major = MAJOR(bdev->bd_dev);
			minor = MINOR(bdev->bd_dev);

			bdput(bdev);

			if((major != CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) || (minor != CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s mismatch major or minor\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}
		}
		else
		{
			printk(KERN_ERR "%s: REJECT realpath=%s\n",
				__FUNCTION__, realpath);

			kfree(realpath);
			return -EPERM;
		}
	}

	if (strncmp(realpath, CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH,
		strlen(CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH)) == 0)
	{
		if (strcmp(realpath, CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH) == 0)
		{
			if (!(flags & MS_RDONLY))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s rw mount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			if ((flags & MS_REMOUNT) && (!(flags & MS_RDONLY)))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s ro remount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			if (flags & MS_BIND)
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s loopback mount\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}

			bdev = lookup_bdev((const char*)dev_name);

			if( bdev == NULL )
			{
				printk("cannot lookup\n");

				kfree(realpath);
				return -EPERM;
			}

			major = MAJOR(bdev->bd_dev);
			minor = MINOR(bdev->bd_dev);

			bdput(bdev);

			if((major != CONFIG_SECURITY_MIYABI_FACTORY_DEV_MAJOR) || (minor != CONFIG_SECURITY_MIYABI_FACTORY_DEV_MINOR))
			{
				printk(KERN_ERR "%s: REJECT dev_name=%s realpath=%s mismatch major or minor\n",
					__FUNCTION__, dev_name, realpath);

				kfree(realpath);
				return -EPERM;
			}
		}
		else
		{
			printk(KERN_ERR "%s: REJECT realpath=%s\n",
				__FUNCTION__, realpath);

			kfree(realpath);
			return -EPERM;
		}
	}

	kfree(realpath);
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */
	return 0;
}

#ifndef CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT
#define CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT "system"
#endif

#ifndef CONFIG_SECURITY_MIYABI_FACTORY_MOUNT_POINT
#define CONFIG_SECURITY_MIYABI_FACTORY_MOUNT_POINT "factory"
#endif

static int miyabi_sb_umount(struct vfsmount *mnt, int flags)
{
#ifndef CONFIG_ANDROID_ENGINEERING
#ifndef CONFIG_ANDROID_RECOVERY_BUILD

	dev_t major, minor;

	(void)flags;

	if(mnt && mnt->mnt_sb && mnt->mnt_sb->s_bdev)
	{
		major = MAJOR(mnt->mnt_sb->s_bdev->bd_dev);
		minor = MINOR(mnt->mnt_sb->s_bdev->bd_dev);

		if((major == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR))
		{
			printk(KERN_ERR "%s: REJECT umount=%d,%d\n", __FUNCTION__, major, minor);

			return -EPERM;
		}
	}
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */
	return 0;
}

static int miyabi_sb_pivotroot(struct path *old_path, struct path *new_path)
{
	char* old_realpath;
	char* new_realpath;

	old_realpath = _xx_realpath_from_path(old_path);
	if(old_realpath == NULL)
	{
		return -ENOMEM;
	}

	new_realpath = _xx_realpath_from_path(new_path);
	if(new_realpath == NULL)
	{
		kfree(old_realpath);
		return -ENOMEM;
	}

	printk(KERN_ERR "%s: REJECT old_path=%s new_path=%s\n",
	       __FUNCTION__, old_realpath, new_realpath);


	kfree(old_realpath);
	kfree(new_realpath);

	return -EPERM;
}

static int miyabi_path_symlink(struct path *dir, struct dentry *dentry, const char *old_name)
{
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	char* realdir;

	realdir = _xx_realpath_from_path(dir);

	if (realdir == NULL) return -ENOMEM;

	if(strncmp(realdir, CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH,
		    strlen(CONFIG_SECURITY_MIYABI_SYSTEM_DIR_PATH)) == 0)
	{
		printk(KERN_ERR "%s: REJECT dir=%s\n", __FUNCTION__, realdir);

		kfree(realdir);
		return -EPERM;
	}

	if((strcmp(realdir, "/") == 0) && 
	    (strncmp(dentry->d_name.name, CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT, 
	            strlen(CONFIG_SECURITY_MIYABI_SYSTEM_MOUNT_POINT)) == 0))
	{
		printk(KERN_ERR "%s: REJECT dir + dentry=%s%s\n", __FUNCTION__, realdir, dentry->d_name.name);

		kfree(realdir);
		return -EPERM;
	}

	if(strncmp(realdir, CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH,
		    strlen(CONFIG_SECURITY_MIYABI_FACTORY_DIR_PATH)) == 0)
	{
		printk(KERN_ERR "%s: REJECT dir=%s\n", __FUNCTION__, realdir);

		kfree(realdir);
		return -EPERM;
	}

	if((strcmp(realdir, "/") == 0) && 
	    (strncmp(dentry->d_name.name, CONFIG_SECURITY_MIYABI_FACTORY_MOUNT_POINT, 
	            strlen(CONFIG_SECURITY_MIYABI_FACTORY_MOUNT_POINT)) == 0))
	{
		printk(KERN_ERR "%s: REJECT dir + dentry=%s%s\n", __FUNCTION__, realdir, dentry->d_name.name);

		kfree(realdir);
		return -EPERM;
	}

	kfree(realdir);
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */

	return 0;
}

static int miyabi_path_link(struct dentry *old_dentry, struct path *new_dir, struct dentry *new_dentry)
{
	return -EPERM;
}

static int miyabi_path_chmod(struct path* path, umode_t mode)
{
#ifndef CONFIG_ANDROID_ENGINEERING
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
	struct inode* inode;
	const struct cred* cred = current_cred();

	if(path == NULL) return -EFAULT;
	if(path->dentry == NULL) return -EFAULT;
	if(path->dentry->d_inode == NULL) return -EFAULT;

	inode = path->dentry->d_inode;

	if(mode & S_ISUID)
	{
		if(inode->i_uid == 0 && cred->euid == 0)
		{
			record_rooted();
			return -EPERM;
		}
	}
	if(mode & S_ISGID)
	{
		if(inode->i_gid == 0 && cred->egid == 0)
		{
			record_rooted();
			return -EPERM;
		}
	}
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
#endif /* ! CONFIG_ANDROID_ENGINEERING */
	return 0;
}

#if 0
#ifndef CONFIG_SECURITY_MIYABI_CHROOT_PATH
#define CONFIG_SECURITY_MIYABI_CHROOT_PATH ""
#endif
#endif

#ifdef CONFIG_SECURITY_PATH
static int miyabi_path_chroot(struct path *path)
{
#ifdef CONFIG_SECURITY_MIYABI_CHROOT_PATH
	char* realpath;
	char* tmp;
	char *p, *p2;

	tmp = kmalloc(PATH_MAX + 1, GFP_KERNEL);

	if(tmp == NULL) return -ENOMEM;

	memset(tmp, 0, PATH_MAX + 1);

	realpath = _xx_realpath_from_path(path);

	if (realpath == NULL)
	{
		kfree(tmp);
		return -ENOMEM;
	}

	p = CONFIG_SECURITY_MIYABI_CHROOT_PATH;

	while (*p)
	{
		p2 = strchr(p, ':');
		if (p2)
		{
			strncpy(tmp, p, (p2 - p));
			tmp[p2 - p] = 0;
		}
		else
		{
			strncpy(tmp, p, PATH_MAX);
		}

		if (strcmp(tmp, realpath) == 0)
		{
			kfree(realpath);
			kfree(tmp);

			return 0;
		}

		if (p2)
		{
			p = p2 + 1;
		}
		else
		{
			p += strlen(p);
		}
	}

	kfree(realpath);
	kfree(tmp);

	return -EPERM;
#else
	return 0;
#endif /* CONFIG_SECURITY_MIYABI_CHROOT_PATH */
}
#endif /* CONFIG_SECURITY_PATH */

static int miyabi_task_fix_setuid(struct cred *new, const struct cred *old, int flags)
{
	if(((new->uid == 0 || new->euid == 0 || new->suid == 0) 
						&& old->uid != 0) ||
	   ((new->gid == 0 || new->egid == 0 || new->sgid == 0) 
						&& old->gid != 0))
	{
#ifdef CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD
#else /* ! CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
		return -EPERM;
#endif /* CONFIG_SECURITY_MIYABI_ENGINEERING_BUILD */
	}

	return cap_task_fix_setuid(new, old, flags);
}

#ifdef CONFIG_SECURITY_NETWORK

#define LOCAL_MIYABI_IPTABLE_PATH "/system/bin/iptables"
#define LOCAL_MIYABI_IP6TABLE_PATH "/system/bin/ip6tables"
#define LOCAL_MIYABI_IPTABLE_PERMIT_PATH_0 "/system/bin/rild"
#define LOCAL_MIYABI_IPTABLE_PERMIT_PATH_1 "/system/bin/netd"

static int miyabi_socket_setsockopt(struct socket *sock, int level, int optname)
{
	if(optname == IPT_SO_SET_REPLACE || optname == IPT_SO_SET_ADD_COUNTERS)
	{
		char* pathbuff = NULL;
		char* path = NULL;

		pathbuff = kmalloc(PATH_MAX + 1, GFP_KERNEL);

		if(pathbuff == NULL) return -ENOMEM;

		memset(pathbuff, 0, PATH_MAX + 1);

		if(current->mm != NULL && current->mm->exe_file != NULL)
		{
			path = d_path(&current->mm->exe_file->f_path, pathbuff, PATH_MAX);
		}

		if(path == NULL || (long)path == ENAMETOOLONG)
		{
			kfree(pathbuff);
			return -EPERM;
		}

		if(
			!(strlen(path) == strlen(LOCAL_MIYABI_IPTABLE_PATH) && strcmp(path, LOCAL_MIYABI_IPTABLE_PATH) == 0) &&
			!(strlen(path) == strlen(LOCAL_MIYABI_IP6TABLE_PATH) && strcmp(path, LOCAL_MIYABI_IP6TABLE_PATH) == 0)
		)
		{
			kfree(pathbuff);
			return -EPERM;
		}
		
		if(current->parent == NULL)
		{
			kfree(pathbuff);
			return -EPERM;
		}

		path = NULL;
		memset(pathbuff, 0, PATH_MAX + 1);

		if(current->parent->mm != NULL && current->parent->mm->exe_file != NULL)
		{
			path = d_path(&current->parent->mm->exe_file->f_path, pathbuff, PATH_MAX);
		}

		if(path == NULL || (long)path == ENAMETOOLONG)
		{
			kfree(pathbuff);
			return -EPERM;
		}

		if((strlen(path) == strlen(LOCAL_MIYABI_IPTABLE_PERMIT_PATH_0)) && (strcmp(path, LOCAL_MIYABI_IPTABLE_PERMIT_PATH_0) == 0))
		{
			kfree(pathbuff);
			return 0;
		}

		if((strlen(path) == strlen(LOCAL_MIYABI_IPTABLE_PERMIT_PATH_1)) && (strcmp(path, LOCAL_MIYABI_IPTABLE_PERMIT_PATH_1) == 0))
		{
			kfree(pathbuff);
			return 0;
		}

		kfree(pathbuff);
		return -EPERM;
	}
	else
	{
		return 0;
	}
}
#endif /* CONFIG_SECURITY_NETWORK */

static int miyabi_dentry_open(struct file *file, const struct cred *cred)
{
#ifndef LOCAL_MIYABI_FBUILD
	dev_t major;
	dev_t minor;

	if(S_ISBLK(file->f_dentry->d_inode->i_mode))
	{
		major = MAJOR(file->f_dentry->d_inode->i_rdev);
		minor = MINOR(file->f_dentry->d_inode->i_rdev);

		if(	((major == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR)) ||
			((major == CONFIG_SECURITY_MIYABI_FACTORY_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_FACTORY_DEV_MINOR))
		)
		{
			printk("reject opening device %d / %d\n", major, minor);

			return -EPERM;
		}
	}
	else if(S_ISREG(file->f_dentry->d_inode->i_mode) || S_ISDIR(file->f_dentry->d_inode->i_mode))
	{
#ifndef CONFIG_ANDROID_RECOVERY_BUILD
		major = MAJOR(file->f_vfsmnt->mnt_sb->s_dev);
		minor = MINOR(file->f_vfsmnt->mnt_sb->s_dev);

		if((major == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_SYSTEM_DEV_MINOR))
		{
			return miyabi_sandbox_dentry_open(file, cred, D_MIYABISANDBOX_SYSTEM);
		}
		else if((major == CONFIG_SECURITY_MIYABI_FACTORY_DEV_MAJOR) && (minor == CONFIG_SECURITY_MIYABI_FACTORY_DEV_MINOR))
		{
			return miyabi_sandbox_dentry_open(file, cred, D_MIYABISANDBOX_FACTORY);
		}
#endif /* ! CONFIG_ANDROID_RECOVERY_BUILD */
	}
#endif /* ! LOCAL_MIYABI_FBUILD */
	return 0;
}

const struct security_operations miyabi_security_ops = {
	"miyabi",

	miyabi_ptrace_access_check,
	miyabi_ptrace_traceme,
	cap_capget,
	cap_capset,
	cap_capable,
	cap_quotactl,
	cap_quota_on,
	cap_syslog,
	cap_settime,
	cap_vm_enough_memory,
	miyabi_bprm_set_creds,
	cap_bprm_check_security,
	cap_bprm_secureexec,
	cap_bprm_committing_creds,
	cap_bprm_committed_creds,
	cap_sb_alloc_security,
	cap_sb_free_security,
	cap_sb_copy_data,
	cap_sb_remount,
	cap_sb_kern_mount,
	cap_sb_show_options,
	cap_sb_statfs,
	miyabi_sb_mount,
	miyabi_sb_umount,
	miyabi_sb_pivotroot,
	cap_sb_set_mnt_opts,
	cap_sb_clone_mnt_opts,
	cap_sb_parse_opts_str,
#ifdef CONFIG_SECURITY_PATH
	cap_path_unlink,
	cap_path_mkdir,
	cap_path_rmdir,
	cap_path_mknod,
	cap_path_truncate,
	miyabi_path_symlink,
	miyabi_path_link,
	cap_path_rename,
	miyabi_path_chmod,
	cap_path_chown,
	miyabi_path_chroot,
#endif	/* CONFIG_SECURITY_PATH */
	cap_inode_alloc_security,
	cap_inode_free_security,
	cap_inode_init_security,
	cap_inode_create,
	cap_inode_link,
	cap_inode_unlink,
	cap_inode_symlink,
	cap_inode_mkdir,
	cap_inode_rmdir,
	cap_inode_mknod,
	cap_inode_rename,
	cap_inode_readlink,
	cap_inode_follow_link,
	cap_inode_permission,
	cap_inode_setattr,
	cap_inode_getattr,
	cap_inode_setxattr,
	cap_inode_post_setxattr,
	cap_inode_getxattr,
	cap_inode_listxattr,
	cap_inode_removexattr,
	cap_inode_need_killpriv,
	cap_inode_killpriv,
	cap_inode_getsecurity,
	cap_inode_setsecurity,
	cap_inode_listsecurity,
	cap_inode_getsecid,
	cap_file_permission,
	cap_file_alloc_security,
	cap_file_free_security,
	cap_file_ioctl,
	cap_file_mmap,
	cap_file_mprotect,
	cap_file_lock,
	cap_file_fcntl,
	cap_file_set_fowner,
	cap_file_send_sigiotask,
	cap_file_receive,
	miyabi_dentry_open,
	cap_task_create,
	cap_task_free,
	cap_cred_alloc_blank,
	cap_cred_free,
	cap_cred_prepare,
	cap_cred_transfer,
	cap_kernel_act_as,
	cap_kernel_create_files_as,
	cap_kernel_module_request,
	miyabi_task_fix_setuid,
	cap_task_setpgid,
	cap_task_getpgid,
	cap_task_getsid,
	cap_task_getsecid,
	cap_task_setnice,
	cap_task_setioprio,
	cap_task_getioprio,
	cap_task_setrlimit,
	cap_task_setscheduler,
	cap_task_getscheduler,
	cap_task_movememory,
	cap_task_kill,
	cap_task_wait,
	cap_task_prctl,
	cap_task_to_inode,
	cap_ipc_permission,
	cap_ipc_getsecid,
	cap_msg_msg_alloc_security,
	cap_msg_msg_free_security,
	cap_msg_queue_alloc_security,
	cap_msg_queue_free_security,
	cap_msg_queue_associate,
	cap_msg_queue_msgctl,
	cap_msg_queue_msgsnd,
	cap_msg_queue_msgrcv,
	cap_shm_alloc_security,
	cap_shm_free_security,
	cap_shm_associate,
	cap_shm_shmctl,
	cap_shm_shmat,
	cap_sem_alloc_security,
	cap_sem_free_security,
	cap_sem_associate,
	cap_sem_semctl,
	cap_sem_semop,
	cap_netlink_send,
	cap_d_instantiate,
	cap_getprocattr,
	cap_setprocattr,
	cap_secid_to_secctx,
	cap_secctx_to_secid,
	cap_release_secctx,
	cap_inode_notifysecctx,
	cap_inode_setsecctx,
	cap_inode_getsecctx,
#ifdef CONFIG_SECURITY_NETWORK
	cap_unix_stream_connect,
	cap_unix_may_send,
	cap_socket_create,
	cap_socket_post_create,
	cap_socket_bind,
	cap_socket_connect,
	cap_socket_listen,
	cap_socket_accept,
	cap_socket_sendmsg,
	cap_socket_recvmsg,
	cap_socket_getsockname,
	cap_socket_getpeername,
	cap_socket_getsockopt,
	miyabi_socket_setsockopt,
	cap_socket_shutdown,
	cap_socket_sock_rcv_skb,
	cap_socket_getpeersec_stream,
	cap_socket_getpeersec_dgram,
	cap_sk_alloc_security,
	cap_sk_free_security,
	cap_sk_clone_security,
	cap_sk_getsecid,
	cap_sock_graft,
	cap_inet_conn_request,
	cap_inet_csk_clone,
	cap_inet_conn_established,
	cap_secmark_relabel_packet,
	cap_secmark_refcount_inc,
	cap_secmark_refcount_dec,
	cap_req_classify_flow,
	cap_tun_dev_create,
	cap_tun_dev_post_create,
	cap_tun_dev_attach,
#endif	/* CONFIG_SECURITY_NETWORK */
#ifdef CONFIG_SECURITY_NETWORK_XFRM
	cap_xfrm_policy_alloc_security,
	cap_xfrm_policy_clone_security,
	cap_xfrm_policy_free_security,
	cap_xfrm_policy_delete_security,
	cap_xfrm_state_alloc_security,
	cap_xfrm_state_free_security,
	cap_xfrm_state_delete_security,
	cap_xfrm_policy_lookup,
	cap_xfrm_state_pol_flow_match,
	cap_xfrm_decode_session,
#endif	/* CONFIG_SECURITY_NETWORK_XFRM */
#ifdef CONFIG_KEYS
	cap_key_alloc,
	cap_key_free,
	cap_key_permission,
	cap_key_getsecurity,
#endif	/* CONFIG_KEYS */
#ifdef CONFIG_AUDIT
	cap_audit_rule_init,
	cap_audit_rule_known,
	cap_audit_rule_match,
	cap_audit_rule_free,
#endif /* CONFIG_AUDIT */
};

