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

#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mutex.h>
#include <linux/fdtable.h>

#include "miyabi_common.h"

char* miyabi_detect_binary(struct task_struct* t, char* pathbuf)
{
	char* path = NULL;

	if(pathbuf != NULL && t->mm != NULL && t->mm->exe_file != NULL)
	{
		path = d_path(&t->mm->exe_file->f_path, pathbuf, PATH_MAX);

		if(path != NULL && strcmp(path, LOCAL_MIYABI_PATH_APPPROCESS) == 0)
		{
			struct files_struct *files = t->files;
			struct fdtable *fdt;

			if(files != NULL)
			{
				spin_lock(&files->file_lock);

				fdt = files_fdtable(files);

				if(fdt != NULL)
				{
					int j;
					int found = 0;

					for(j = 0; j < fdt->max_fds; j++)
					{
						struct file* f = fdt->fd[j];

						if(f != NULL && f->f_dentry != NULL && f->f_dentry->d_inode != NULL && f->f_dentry->d_inode->i_mode & S_IFREG)
						{
							path = d_path(&f->f_path, pathbuf, PATH_MAX);

							if(path != NULL && strstr(path, LOCAL_MIYABI_DIR_FRAMEWORK) == NULL)
							{
								char* dot = strrchr(path, '.');

								if(dot != NULL && strstr(dot, LOCAL_MIYABI_SUFFIX_APK) != NULL)
								{
									found = 1;

									break;
								}
							}
						}
					}

					if(found == 0)
					{
						path = pathbuf;

						strncpy(path, LOCAL_MIYABI_PATH_APPPROCESS, PATH_MAX);
					}
				}
				else
				{
					path = NULL;
				}

				spin_unlock(&files->file_lock);
			}
			else
			{
				path = NULL;
			}
		}
	}
	
	return path;
}

char* miyabi_detect_package(struct task_struct* t, char* buffer)
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


