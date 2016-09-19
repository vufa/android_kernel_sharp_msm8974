/*
 * Copyright (C) 2012 Sharp.
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
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/mman.h>
#include <linux/kernel.h>
#include <linux/module.h> 
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/pfn.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <asm/uaccess.h>
#include <asm/io.h>
/* -------------------------------------------------------------------- */
#define SHVQRDEV_KER_BASEMINOR	(0)
#define SHVQRDEV_KER_MINORCOUNT	(1)
#define SHVQRDEV_KER_DRVNAME	"shvqrdrv"
#define SHVQRDEV_CLASS_NAME		"cls_shvqrdrv"
#define SHVQRDEV_CHECK_PGOFF_A	(0x0FD80000 >> PAGE_SHIFT)
#define SHVQRDEV_CHECK_PGOFF_B	(0x8F700000 >> PAGE_SHIFT)
#define SHVQRDEV_CHECK_MAX_SIZE_A	(0x00180000)
#define SHVQRDEV_CHECK_MAX_SIZE_B	(8 * 1024)
#define SHVQRDEV_CHECK_PATH		"/system/bin/rmt_storage"
/* -------------------------------------------------------------------- */
typedef struct{
	int major;
	dev_t dev;
	struct cdev shvqrdrv_cdev;
	struct class* shvqrdrv_classp;
	int status;
}shvqrdrv_info_type;
shvqrdrv_info_type shvqrdrv_info;
/* -------------------------------------------------------------------- */
static int		shvqrdrv_open(struct inode* inode, struct file* filp);
static int		shvqrdrv_close(struct inode* inode, struct file* filp);
static int 		shvqrdrv_mmap(struct file *file, struct vm_area_struct *vma);
/* -------------------------------------------------------------------- */
static struct file_operations shvqrdrv_Ops = {
	.owner   = THIS_MODULE,
	.open    = shvqrdrv_open,
	.release = shvqrdrv_close,
	.mmap    = shvqrdrv_mmap,
};
/* -------------------------------------------------------------------- */
static pid_t shvqrdrv_client_pid = -1;
/* -------------------------------------------------------------------- */
static int __init shvqrdrv_ker_init(void)
{
	int sdResult;
	struct device* devp;

	shvqrdrv_info.major = -1;
	shvqrdrv_info.shvqrdrv_classp = NULL;
	shvqrdrv_info.status = 0;
	
	shvqrdrv_info.dev = MKDEV(shvqrdrv_info.major, 0);
	
	sdResult = alloc_chrdev_region( &shvqrdrv_info.dev, SHVQRDEV_KER_BASEMINOR, SHVQRDEV_KER_MINORCOUNT, SHVQRDEV_KER_DRVNAME );
	if( sdResult < 0 ){
		return -1;
	}
	shvqrdrv_info.major = sdResult;
	

	cdev_init( &shvqrdrv_info.shvqrdrv_cdev, &shvqrdrv_Ops );
	shvqrdrv_info.shvqrdrv_cdev.owner = THIS_MODULE;
	shvqrdrv_info.shvqrdrv_cdev.ops = &shvqrdrv_Ops;

	sdResult = cdev_add(&shvqrdrv_info.shvqrdrv_cdev, shvqrdrv_info.dev, SHVQRDEV_KER_MINORCOUNT);
	if( sdResult < 0 ){
		return -1;
	}

	
	shvqrdrv_info.shvqrdrv_classp = class_create( THIS_MODULE, SHVQRDEV_CLASS_NAME );
	if (IS_ERR(shvqrdrv_info.shvqrdrv_classp)){
		return -1;
	}

	devp = device_create( shvqrdrv_info.shvqrdrv_classp, NULL, shvqrdrv_info.dev, NULL, SHVQRDEV_KER_DRVNAME );
	sdResult = IS_ERR(devp) ? PTR_ERR(devp) : 0;
	if ( sdResult < 0 ){
		return -1;
	}
	
	return 0;
}
/* -------------------------------------------------------------------- */
static void __exit shvqrdrv_ker_term( void )
{
	if( shvqrdrv_info.major < 0){
		return;
	}
	
	device_destroy( shvqrdrv_info.shvqrdrv_classp, shvqrdrv_info.dev );
	class_destroy( shvqrdrv_info.shvqrdrv_classp );
	shvqrdrv_info.shvqrdrv_classp = NULL;

	cdev_del( &shvqrdrv_info.shvqrdrv_cdev );
	unregister_chrdev_region( shvqrdrv_info.dev, SHVQRDEV_KER_MINORCOUNT );
	shvqrdrv_info.major = -1;

	return;
}
/* -------------------------------------------------------------------- */
static char* shvqrdrv_guess_binary(struct task_struct* t, char* buf, int len)
{
	char* p = NULL;

	if(buf != NULL && t->mm != NULL && t->mm->exe_file != NULL)
	{
		p = d_path(&t->mm->exe_file->f_path, buf, len);

		if(p == NULL || (long)p == ENAMETOOLONG)
		{
			return NULL;
		}
	}
	
	return p;
}
/* -------------------------------------------------------------------- */
/*  */
/* -------------------------------------------------------------------- */
static int shvqrdrv_open(struct inode* inode, struct file* filp)
{
	char* buff = NULL;
	int ret = -EPERM;
	char* p = NULL;

	do
	{
		if(!capable(CAP_SYS_RAWIO)) break;

		if(shvqrdrv_client_pid != -1)
		{
			ret = -EPERM;

			break;
		}

		buff = kmalloc(PAGE_SIZE, GFP_KERNEL);

		if(buff == NULL)
		{
			ret = -ENOMEM;

			break;
		}

		memset(buff, 0, PAGE_SIZE);

		p = shvqrdrv_guess_binary(current, buff, PAGE_SIZE - 1);

		if(p == NULL)
		{
			ret = -EPERM;

			break;
		}

		if(strlen(p) != strlen(SHVQRDEV_CHECK_PATH))
		{
			ret = -EPERM;

			break;
		}

		if(strcmp(p, SHVQRDEV_CHECK_PATH) != 0)
		{
			ret = -EPERM;

			break;
		}
		
		shvqrdrv_client_pid = current->pid;

		ret = 0;
	}
	while(0);

	if(buff != NULL)
	{
		kfree(buff);

		buff = NULL;
	}

	return ret;
}
/* -------------------------------------------------------------------- */
static int shvqrdrv_close(struct inode* inode, struct file* filp)
{
	if(shvqrdrv_client_pid == -1) return -EPERM;

	if(shvqrdrv_client_pid != current->pid) return -EPERM;

	shvqrdrv_client_pid = -1;

	return 0;
}
/* -------------------------------------------------------------------- */
#ifndef __HAVE_PHYS_MEM_ACCESS_PROT

/*
 * Architectures vary in how they handle caching for addresses
 * outside of main memory.
 *
 */
#ifdef pgprot_noncached
static int uncached_access(struct file *file, unsigned long addr)
{
#if defined(CONFIG_IA64)
	/*
	 * On ia64, we ignore O_DSYNC because we cannot tolerate memory
	 * attribute aliases.
	 */
	return !(efi_mem_attributes(addr) & EFI_MEMORY_WB);
#elif defined(CONFIG_MIPS)
	{
		extern int __uncached_access(struct file *file,
					     unsigned long addr);

		return __uncached_access(file, addr);
	}
#else
	/*
	 * Accessing memory above the top the kernel knows about or through a
	 * file pointer
	 * that was marked O_DSYNC will be done non-cached.
	 */
	if (file->f_flags & O_DSYNC)
		return 1;
	return addr >= __pa(high_memory);
#endif
}
#endif
/* -------------------------------------------------------------------- */
static pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
				     unsigned long size, pgprot_t vma_prot)
{
#ifdef pgprot_noncached
	unsigned long offset = pfn << PAGE_SHIFT;

	if (uncached_access(file, offset))
		return pgprot_noncached(vma_prot);
#endif
	return vma_prot;
}
#endif

static const struct vm_operations_struct mmap_mem_ops = {
#ifdef CONFIG_HAVE_IOREMAP_PROT
	.access = generic_access_phys
#endif
};
/* -------------------------------------------------------------------- */
static int shvqrdrv_mmap(struct file *file, struct vm_area_struct *vma)
{
	size_t size = vma->vm_end - vma->vm_start;

	if (!valid_mmap_phys_addr_range(vma->vm_pgoff, size))
	{
		return -EINVAL;
	}

	if(vma->vm_pgoff != SHVQRDEV_CHECK_PGOFF_A && vma->vm_pgoff != SHVQRDEV_CHECK_PGOFF_B)
	{
		return -EINVAL;
	}

	if(size < 0)
	{
		return -EINVAL;
	}

	if(vma->vm_pgoff == SHVQRDEV_CHECK_PGOFF_A && size != SHVQRDEV_CHECK_MAX_SIZE_A)
	{
		return -EINVAL;
	}

	if(vma->vm_pgoff == SHVQRDEV_CHECK_PGOFF_B && size != SHVQRDEV_CHECK_MAX_SIZE_B)
	{
		return -EINVAL;
	}

	vma->vm_page_prot = phys_mem_access_prot(file, vma->vm_pgoff,
						 size,
						 vma->vm_page_prot);

	vma->vm_ops = &mmap_mem_ops;

	/* Remap-pfn-range will mark the range VM_IO and VM_RESERVED */
	if (remap_pfn_range(vma,
			    vma->vm_start,
			    vma->vm_pgoff,
			    size,
			    vma->vm_page_prot))
	{
		return -EAGAIN;
	}

	return 0;
}
/* -------------------------------------------------------------------- */
module_init( shvqrdrv_ker_init );
module_exit( shvqrdrv_ker_term );
/* -------------------------------------------------------------------- */
MODULE_AUTHOR("SHARP");
MODULE_DESCRIPTION("shvqrdrv device");
MODULE_LICENSE("GPL2");
/* -------------------------------------------------------------------- */

