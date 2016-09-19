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
/* -------------------------------------------------------------------- */
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/fdtable.h>
#include <linux/slab.h>

#include "miyabi_common.h"
#include "miyabi_sandbox.h"
/* -------------------------------------------------------------------- */
#define LOCAL_MIYABI_DIR_SYSTEMSANDBOX	"/system/sandbox/"
#define LOCAL_MIYABI_DIR_FACTORYSANDBOX	"/factory/sandbox/"
#define LOCAL_MIYABI_PATH_SYSTEMSANDBOX	"/system/sandbox"
#define LOCAL_MIYABI_PATH_FACTORYSANDBOX	"/factory/sandbox"

#define LOCAL_MIYABI_FMODE_MASK 		(F_MIYABISANDBOX_RDONLY | F_MIYABISANDBOX_WRONLY)

#define miyabi_printk			if(0)printk
/* -------------------------------------------------------------------- */
struct miyabi_allow_deny
{
	uint32_t direction;
	const char guid[36 + 1];
	const char* prog_name;
	uint32_t flags;
	uint32_t policies;
};
/* -------------------------------------------------------------------- */
static const struct miyabi_allow_deny miyabi_allow_deny_list[] =
{
#include "miyabi_sandbox.lst"
};
/* -------------------------------------------------------------------- */
int miyabi_sandbox_dentry_open(struct file *file, const struct cred *cred, uint32_t direction)
{
	int ret = -EPERM;
	char *binbuf = NULL, *bin = NULL;
	char *pathbuf = NULL, *path = NULL;
	int i;

	do
	{
		pathbuf = kmalloc(PATH_MAX, GFP_KERNEL);

		if(pathbuf == NULL)
		{
			ret = -ENOMEM;

			break;
		}

		memset(pathbuf, 0, PATH_MAX);

		path = d_path(&file->f_path, pathbuf, PATH_MAX);
		
		if(path == NULL)
		{
			ret = -ENOMEM;

			break;
		}

		if(direction == D_MIYABISANDBOX_SYSTEM)
		{
			if(strstr(path, LOCAL_MIYABI_PATH_SYSTEMSANDBOX) == path)
			{
				if(strlen(path) == strlen(LOCAL_MIYABI_PATH_SYSTEMSANDBOX))
				{
					ret = -EPERM;

					break;
				}
			}

			if(strstr(path, LOCAL_MIYABI_DIR_SYSTEMSANDBOX) == NULL)
			{
				ret = 0;

				break;
			}
		}
		else if(direction == D_MIYABISANDBOX_FACTORY)
		{
			if(strstr(path, LOCAL_MIYABI_PATH_FACTORYSANDBOX) == path)
			{
				if(strlen(path) == strlen(LOCAL_MIYABI_PATH_FACTORYSANDBOX))
				{
					ret = -EPERM;

					break;
				}
			}

			if(strstr(path, LOCAL_MIYABI_DIR_FACTORYSANDBOX) == NULL)
			{
				ret = 0;

				break;
			}
		}
		else
		{
			ret = -EPERM;

			break;
		}

		if((file->f_flags & O_CLOEXEC) == 0)
		{
			ret = -EPERM;

			break;
		}

		// ---

		binbuf = kmalloc(PATH_MAX, GFP_KERNEL);

		if(binbuf == NULL)
		{
			ret = -ENOMEM;

			break;
		}

		memset(binbuf, 0, PATH_MAX);

		read_lock(&tasklist_lock);

		bin = miyabi_detect_binary(current, binbuf);

		read_unlock(&tasklist_lock);

		if(bin == NULL)
		{
			break;
		}

		if(strcmp(bin, LOCAL_MIYABI_PATH_APPPROCESS) == 0)
		{
			break;
		}

		for(i = 0; i < sizeof(miyabi_allow_deny_list) / sizeof(struct miyabi_allow_deny); i++)
		{
			if(	(miyabi_allow_deny_list[i].direction == direction) &&
				(strncmp(path + strlen(LOCAL_MIYABI_DIR_SYSTEMSANDBOX), miyabi_allow_deny_list[i].guid, 36) == 0) && 
				(strcmp(bin, miyabi_allow_deny_list[i].prog_name) == 0) &&
				((miyabi_allow_deny_list[i].flags & LOCAL_MIYABI_FMODE_MASK) == (file->f_mode & LOCAL_MIYABI_FMODE_MASK)))
			{
				miyabi_printk("%s will open %s\n", bin, path);

				ret = 0;

				break;
			}
		}
	}
	while(0);

	if(pathbuf != NULL)
	{
		kfree(pathbuf);
		pathbuf = NULL;
	}

	if(binbuf != NULL)
	{
		kfree(binbuf);
		binbuf = NULL;
	}

	return ret;
}
/* -------------------------------------------------------------------- */

