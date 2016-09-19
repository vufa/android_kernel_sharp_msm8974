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
#define D_MIYABISANDBOX_SYSTEM	(1)
#define D_MIYABISANDBOX_DATA	(2)
#define D_MIYABISANDBOX_DURABLE	(3)
#define D_MIYABISANDBOX_FACTORY	(4)

#define F_MIYABISANDBOX_RDONLY	(FMODE_READ)
#define F_MIYABISANDBOX_WRONLY	(FMODE_WRITE)
#define F_MIYABISANDBOX_RDWR	(FMODE_READ | FMODE_WRITE)

#define P_MIYABISANDBOX_SYSTEM	(0x01)
#define P_MIYABISANDBOX_SIGNATURE	(0x02)
#define P_MIYABISANDBOX_NOLIMIT	(0x10)
/* -------------------------------------------------------------------- */
int miyabi_sandbox_dentry_open(struct file *file, const struct cred *cred, uint32_t direction);

