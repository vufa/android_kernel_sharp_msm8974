/*
 * Copyright (C) 2013 Sharp.
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

/*
 * include
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/proc_fs.h>
#include <linux/crypto.h>
#include <crypto/hash.h>
#include <crypto/sha.h>

/*
 * define
 */

#define CREATE_KOBJECT(dir, file)										\
	static void dir##_release(struct kobject *kobj);					\
	static ssize_t dir##_show(struct kobject *kobj, struct attribute *attr, char *buf); \
	static ssize_t dir##_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len);	\
	static struct kobject dir##_kobj;									\
	static struct kobj_attribute dir##_attribute = __ATTR(file, 0200, NULL, NULL);	\
	static struct attribute *dir##_default_attrs[] = {					\
		&dir##_attribute.attr,											\
		NULL,	/* need to NULL terminate the list of attributes */		\
	};																	\
	static struct sysfs_ops dir##_sysfs_ops = {						\
		.show = dir##_show,											\
		.store = dir##_store,											\
	};																	\
	static struct kobj_type dir##_ktype = {							\
		.release = dir##_release,										\
		.sysfs_ops = &dir##_sysfs_ops,									\
		.default_attrs = dir##_default_attrs,							\
	};																	\
	static kobj_data dir##_data = {										\
		.name = #dir,													\
		.kobj = &dir##_kobj,											\
		.ktype = &dir##_ktype,											\
	};																	\

/*
 * types
 */

typedef struct
{
	const char *name;
	struct kobject *kobj;
	struct kobj_type *ktype;
} kobj_data;

/*
 * static
 */
static struct kset *mnhelper_kset;
static int permited_uid = 1;
static const unsigned char mnhelper_scp0[SHA256_DIGEST_SIZE] = {0x04, 0xa1, 0xbd, 0x25, 0xd5, 0xf2, 0xe4, 0x10, 0x78, 0xc8, 0xe9, 0xbc, 0x0b, 0x84, 0x7e, 0xfc, 0xc6, 0x0b, 0xfa, 0x69, 0xd7, 0x2f, 0xe6, 0xfe, 0x3e, 0xea, 0xe9, 0xe4, 0x83, 0x9b, 0x10, 0x59};
static const unsigned char mnhelper_scp1[SHA256_DIGEST_SIZE] = {0xfe, 0x0c, 0x87, 0xdc, 0x8f, 0x7f, 0x30, 0x45, 0xe1, 0xf4, 0x2c, 0x2e, 0xae, 0xfb, 0x5e, 0xac, 0x8a, 0xf9, 0x26, 0x6e, 0x62, 0xa2, 0xfc, 0x53, 0xc2, 0x6e, 0xb0, 0x82, 0x8b, 0x1f, 0x54, 0x8d};
static const unsigned char mnhelper_scp2[SHA256_DIGEST_SIZE] = {0x99, 0xea, 0x9c, 0x9c, 0x84, 0xd4, 0x39, 0x33, 0x7d, 0x27, 0x24, 0x52, 0xc3, 0xd0, 0xf8, 0xc8, 0x07, 0x18, 0x5f, 0xca, 0x89, 0xbe, 0x19, 0x56, 0xbb, 0xb5, 0x40, 0x59, 0x6a, 0x33, 0xfc, 0xbd};

/*
 * object
 */
CREATE_KOBJECT(sender, value);

/*
 * extern
 */
int mnhelper_get_uid(void)
{
	return permited_uid;
}

/*
 * function
 */
static int mnhelper_atoi(const char* a)
{
	int ans = 0;
 
	while (*a != '\0')
	{
		ans = ans * 10 + (*a++) - '0';
	}
 
	return ans;
}

static char* mnhelper_detect_package(struct task_struct* t, char* buffer)
{
	char* package = NULL;

	int res = 0;
	unsigned int len;
	struct mm_struct *mm = get_task_mm(t);
	if (!mm)
		goto out;
	if (!mm->arg_end)
		goto out_mm;

 	len = mm->arg_end - mm->arg_start;
 
	if (len > PAGE_SIZE)
		len = PAGE_SIZE;
 
	res = access_process_vm(t, mm->arg_start, buffer, len, 0);

	if (res > 0 && buffer[res-1] != '\0' && len < PAGE_SIZE) {
		len = strnlen(buffer, res);
		if (len < res) {
		    res = len;
		} else {
			len = mm->env_end - mm->env_start;
			if (len > PAGE_SIZE - res)
				len = PAGE_SIZE - res;
			res += access_process_vm(t, mm->env_start, buffer+res, len, 0);
			res = strnlen(buffer, res);
		}
	}
out_mm:
	mmput(mm);
out:
	if(res > 0) package = buffer;

	return package;
}

static void sender_release(struct kobject *kobj)
{
	kfree(kobj);
}

static ssize_t sender_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
	return 0;
}

static ssize_t sender_store(struct kobject *kobj, struct attribute *attr, const char *buf, size_t len)
{
	char* pathbuff = NULL;
	char* path = NULL;
	char* pos = NULL;
	int uid = 1;
	struct crypto_shash *shash = NULL;
	struct shash_desc *desc = NULL;
	unsigned char hash[SHA256_DIGEST_SIZE];

	do
	{
		if(len >= PATH_MAX) break;

		if(current_uid() != 1000) break;

		pathbuff = kmalloc(PATH_MAX + 1, GFP_KERNEL);

		if(pathbuff == NULL) break;

		shash = crypto_alloc_shash("sha256", 0, 0);

		if(IS_ERR(shash)) break;

		desc = kmalloc(sizeof(*desc) + crypto_shash_descsize(shash), GFP_KERNEL);

		if(desc == NULL) break;

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

		if(memcmp(hash, mnhelper_scp0, SHA256_DIGEST_SIZE) != 0) break;

		memset(pathbuff, 0, PATH_MAX + 1);

		path = mnhelper_detect_package(current, pathbuff);
		
		if(path == NULL) break;

		crypto_shash_init(desc);
		crypto_shash_update(desc, path, strlen(path));
		crypto_shash_final(desc, hash);

		if(memcmp(hash, mnhelper_scp1, SHA256_DIGEST_SIZE) != 0) break;

		memset(pathbuff, 0, PATH_MAX + 1);
		memcpy(pathbuff, buf, len);

		pos = strchr(pathbuff, ':');

		if(pos == NULL) break;

		*pos = 0x00;

		uid = mnhelper_atoi(pos + 1);

		crypto_shash_init(desc);
		crypto_shash_update(desc, pathbuff, strlen(pathbuff));
		crypto_shash_final(desc, hash);

		if(memcmp(hash, mnhelper_scp2, SHA256_DIGEST_SIZE) == 0)
		{
			permited_uid = uid;
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

	return len;
}

static int mnhelper_create_kobject(kobj_data *data)
{
	int ret;

	/*
	 * As we have a kset for this kobject, we need to set it before calling
	 * the kobject core.
	 */
	data->kobj->kset = mnhelper_kset;

	/*
	 * Initialize and add the kobject to the kernel.  All the default files
	 * will be created here.  As we have already specified a kset for this
	 * kobject, we don't have to set a parent for the kobject, the kobject
	 * will be placed beneath that kset automatically.
	 */
	ret = kobject_init_and_add(data->kobj, data->ktype, NULL, "%s", data->name);
	if(ret) kobject_put(data->kobj);

	/*
	 * We are always responsible for sending the uevent that the kobject
	 * was added to the system.
	 */
	if(!ret) kobject_uevent(data->kobj, KOBJ_ADD);

	return ret;
}

static int __init mnhelper_init(void)
{
	int ret;

	/* Create a simple kobject with the name of "mnhelper" located under /sys/kernel/ */
	mnhelper_kset = kset_create_and_add("mnhelper", NULL, kernel_kobj);
	if(!mnhelper_kset) return -ENOMEM;

	ret = mnhelper_create_kobject(&sender_data);

	return ret;
}

static void __exit mnhelper_exit(void)
{
	kset_unregister(mnhelper_kset);
}

module_init(mnhelper_init);
module_exit(mnhelper_exit);
MODULE_LICENSE("GPL2");
MODULE_AUTHOR("SHARP");
