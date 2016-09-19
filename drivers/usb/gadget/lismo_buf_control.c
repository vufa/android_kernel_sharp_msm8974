/* drivers/usb/gadget/lismo_buf_control.c
 *
 * lismo_buf_control.c -- buffer control for LISMO file transfer
 *
 * Copyright (C) 2013 SHARP CORPORATION
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


/* #define VERBOSE_DEBUG */
/* #define DUMP_MSGS */


#include <linux/blkdev.h>
#include <linux/completion.h>
#include <linux/dcache.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fcntl.h>
#include <linux/file.h>
#include <linux/fs.h>
#include <linux/kref.h>
#include <linux/kthread.h>
#include <linux/limits.h>
#include <linux/rwsem.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/freezer.h>
#include <linux/utsname.h>

#include <linux/platform_device.h>

#define LUN_FUNCTION_NAME		"usb_mass_storage"

/*------------------------------------------------------------------------*/

#define LISMO_BUFFER_DRIVER_DESC	"[LISMO] buffer control Function"
#define LISMO_BUFFER_DRIVER_VERSION	"2011/07/28"

#include "lismo_buf_control.h"

/*-------------------------------------------------------------------------*/

struct platform_device *lun_device = NULL;

/*-------------------------------------------------------------------------*/

/* setting notify change buffer */
static void buffer_notify_sysfs(struct work_struct *work)
{
	struct op_desc	*desc;
//printk("%s\n", __func__);
	desc = container_of(work, struct op_desc, work);
	sysfs_notify_dirent(desc->value_sd);
}

/* check vendor command code */
static int vendor_cmd_is_valid(unsigned cmd)
{
	if(cmd < SC_VENDOR_START)
		return 0;
	if(cmd > SC_VENDOR_END)
		return 0;
	return 1;
}

/* read vendor command buffer */
static ssize_t
vendor_cmd_read_buffer(struct file* f, struct kobject *kobj, struct bin_attribute *attr,
                char *buf, loff_t off, size_t count)
{
	ssize_t	status;
	struct op_desc	*desc = attr->private;

//printk("[lismo_buf_cntrol]%s: buf=%p off=%lx count=%x\n", __func__, buf, (unsigned long)off, count);
	mutex_lock(&sysfs_lock);

	if (!test_bit(FLAG_EXPORT, &desc->flags)){
		status = -EIO;
		printk("[lismo_buf_cntrol]%s not export\n", __func__);
	} else {
		size_t srclen, n;
		void *src;
		size_t nleft = count;
		src = desc->buffer;
		srclen = desc->len;

		if (off < srclen) {
			n = min(nleft, srclen - (size_t) off);
			memcpy(buf, src + off, n);
			nleft -= n;
			buf += n;
			off = 0;
//printk("[lismo_buf_cntrol]%s success\n", __func__);
		} else {
			off -= srclen;
			printk("[lismo_buf_cntrol]%s offset param err\n", __func__);
		}
		status = count - nleft;
	}

	mutex_unlock(&sysfs_lock);
	return status;
}

/* write vendor command buffer */
static ssize_t
vendor_cmd_write_buffer(struct file* f, struct kobject *kobj, struct bin_attribute *attr,
                char *buf, loff_t off, size_t count)
{
	ssize_t	status;
	struct op_desc	*desc = attr->private;

//printk("[lismo_buf_cntrol]%s: buf=%p off=%lx count=%x\n", __func__, buf, (unsigned long)off, count);
	mutex_lock(&sysfs_lock);

	if (!test_bit(FLAG_EXPORT, &desc->flags)){
		status = -EIO;
		printk("[lismo_buf_cntrol]%s not export\n", __func__);
	} else {
		size_t dstlen, n;
		size_t nleft = count;
		void *dst;

		dst = desc->buffer;
		dstlen = desc->len;

		if (off < dstlen) {
			n = min(nleft, dstlen - (size_t) off);
			memcpy(dst + off, buf, n);
			nleft -= n;
			buf += n;
			off = 0;
//printk("[lismo_buf_cntrol]%s success\n", __func__);
		} else {
			off -= dstlen;
			printk("[lismo_buf_cntrol]%s offset param err\n", __func__);
		}
		status = count - nleft;
	}

	desc->update = jiffies;

	mutex_unlock(&sysfs_lock);
	return status;
}

/* memory mapping vendor commn buffer */
static int
vendor_cmd_mmap_buffer(struct file *f, struct kobject *kobj, struct bin_attribute *attr,
		struct vm_area_struct *vma)
{
        int rc = -EINVAL;
	unsigned long pgoff, delta;
	ssize_t size = vma->vm_end - vma->vm_start;
	struct op_desc	*desc = attr->private;

//printk("[lismo_buf_cntrol]%s \n",__func__);
	mutex_lock(&sysfs_lock);

	if (vma->vm_pgoff != 0) {
		printk("[lismo_buf_cntrol]%s mmap failed: page offset %lx\n", __func__, vma->vm_pgoff);
		goto done;
	}

	pgoff = __pa(desc->buffer);
	delta = PAGE_ALIGN(pgoff) - pgoff;
//printk("[lismo_buf_cntrol]%s size=%x delta=%lx pgoff=%lx\n", __func__, size, delta, pgoff);

        if (size + delta > desc->len) {
		printk("[lismo_buf_cntrol]%s mmap failed: page offset %lx\n", __func__, vma->vm_pgoff);
                printk("mmap failed: size %d\n", size);
		goto done;
        }

        pgoff += delta;
        vma->vm_flags |= VM_RESERVED;

	rc = io_remap_pfn_range(vma, vma->vm_start, pgoff >> PAGE_SHIFT,
		size, vma->vm_page_prot);

	if (rc < 0)
		printk("[lismo_buf_cntrol]%s mmap failed rc=%d size=%x delta=%lx pgoff=%lx\n", __func__, rc, size, delta, pgoff);
done:
	mutex_unlock(&sysfs_lock);
	return rc;
}

/* set 'size'file */
static ssize_t vendor_size_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct op_desc	*desc = dev_to_desc(dev);
	ssize_t		status;
	pr_debug("[lismo_buf_cntrol]%s \n",__func__);

	mutex_lock(&sysfs_lock);

	if (!test_bit(FLAG_EXPORT, &desc->flags)){
		status = -EIO;
		printk("[lismo_buf_cntrol]%s not export\n", __func__);
	} else {
		status = sprintf(buf, "%d\n", desc->len);
	pr_debug("[lismo_buf_cntrol]%s success\n",__func__);
	}
	mutex_unlock(&sysfs_lock);
	return status;
}

/* when update 'size'file */
static ssize_t vendor_size_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	long len = 0;
	char* buffer;
	struct op_desc	*desc = dev_to_desc(dev);
	ssize_t		status;
	long cmd;
	char cmd_buf[16]="0x";
	struct fsg_lun	*curlun = fsg_lun_from_dev(&desc->dev);
	
	pr_debug("[lismo_buf_cntrol]%s desc->len=%08x ALLOC_INI_SIZE=%08x \n",__func__, desc->len, ALLOC_INI_SIZE);

	mutex_lock(&sysfs_lock);
	
	if (!test_bit(FLAG_EXPORT, &desc->flags)) {
		status = -EIO;
		printk("[lismo_buf_cntrol]%s not export\n", __func__);
	} else {
		struct bin_attribute* dev_bin_attr_buffer = &desc->dev_bin_attr_buffer;
		status = strict_strtol(buf, 0, &len);
		if (status < 0 || len <= 0) {
			status = -EINVAL;
			printk("[lismo_buf_cntrol]%s size param error \n", __func__);
			goto done;
		}
		if ( desc->len == len ) {
			status = 0;
			pr_debug("[lismo_buf_cntrol]%s same size param ,size not change\n", __func__);
			goto done;
		}
		
		status = strict_strtol(strcat(cmd_buf,dev_name(&desc->dev)+7), 0, &cmd);
		pr_debug("[lismo_buf_cntrol]%s cmd=0x%x old_size=0x%x new_size=0x%x \n", __func__, (unsigned int)cmd, (unsigned int)desc->len, (unsigned int)len);

		if ( cmd-SC_VENDOR_START < ALLOC_CMD_CNT && len == ALLOC_INI_SIZE){
			pr_debug("[lismo_buf_cntrol]%s buffer alreay malloc \n",__func__);
			buffer = curlun->reserve_buf[cmd-SC_VENDOR_START];
		} else {
			pr_debug("[lismo_buf_cntrol]%s malloc buffer \n",__func__);
			buffer = kzalloc(len, GFP_KERNEL);
			if(!buffer) {
				status = -ENOMEM;
				goto done;
			}
		}
		if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT || desc->len != ALLOC_INI_SIZE){
			pr_debug("[lismo_buf_cntrol]%s free old buffer \n",__func__);
			kfree(desc->buffer);
		}
		desc->len = len;
		desc->buffer = buffer;
		device_remove_bin_file(&desc->dev, dev_bin_attr_buffer);
		dev_bin_attr_buffer->size = len;
		status = device_create_bin_file(&desc->dev, dev_bin_attr_buffer);
		pr_debug("[lismo_buf_cntrol]%s success\n",__func__);
	}

done:
	mutex_unlock(&sysfs_lock);
	return status ? : size;
}
/* define 'size'file */
static DEVICE_ATTR(size, 0606, vendor_size_show, vendor_size_store);

/* set 'update'file */
static ssize_t vendor_update_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct op_desc	*desc = dev_to_desc(dev);
	ssize_t		status;
//printk("[lismo_buf_cntrol]%s \n",__func__);

	mutex_lock(&sysfs_lock);

	if (!test_bit(FLAG_EXPORT, &desc->flags)){
		status = -EIO;
		printk("[lismo_buf_cntrol]%s not export \n",__func__);
	} else {
		status = sprintf(buf, "%lu\n", desc->update);
//printk("[lismo_buf_cntrol]%s success\n",__func__);
	}

	mutex_unlock(&sysfs_lock);
	return status;
}

/* get 'update'file */
static ssize_t vendor_update_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t size)
{
	struct op_desc	*desc = dev_to_desc(dev);
	ssize_t		status;
//printk("[lismo_buf_cntrol]%s \n",__func__);

	mutex_lock(&sysfs_lock);

	if (!test_bit(FLAG_EXPORT, &desc->flags)){
		status = -EIO;
		printk("[lismo_buf_cntrol]%s not export\n",__func__);
	} else {
		status = 0;
		/* 'update' notify to select()*/
		desc->update = jiffies;
		schedule_work(&desc->work);
//printk("[lismo_buf_cntrol]%s success\n",__func__);
	}
	mutex_unlock(&sysfs_lock);

	return status;
}
/* define 'update'file */
static DEVICE_ATTR(update, 0606, vendor_update_show, vendor_update_store);

/* vendor command create */
static int vendor_cmd_export(struct device *dev, unsigned cmd, int init)
{
	struct fsg_lun	*curlun = fsg_lun_from_dev(dev);
	struct op_desc	*desc;
	int		status = -EINVAL;
	struct bin_attribute* dev_bin_attr_buffer;
	pr_debug("[lismo_buf_cntrol]%s start\n",__func__);

	desc = curlun->op_desc[cmd-SC_VENDOR_START];
	if (!desc) {
		desc = kzalloc(sizeof(struct op_desc), GFP_KERNEL);
		if(!desc) {
			status = -ENOMEM;
			printk("[lismo_buf_cntrol]%s desc alloc fail\n",__func__);
			goto done;
		}
		curlun->op_desc[cmd-SC_VENDOR_START] = desc;
	}

	status = 0;
	if (test_bit(FLAG_EXPORT, &desc->flags)) {
		pr_debug("[lismo_buf_cntrol]%s id:%02x alrdy created\n",__func__,cmd);
		goto done;
	}

	if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT ){
		desc->buffer = kzalloc(2048, GFP_KERNEL);
		pr_debug("[lismo_buf_cntrol]%s opcode:%02x bufalloc size:%08x \n", __func__, cmd, 2048);
		if(!desc->buffer) {
			status = -ENOMEM;
			goto done;
		}
		desc->len = 2048;
	}else{
		desc->buffer = curlun->reserve_buf[cmd-SC_VENDOR_START];
		pr_debug("[lismo_buf_cntrol]%s opcode:%02x bufcopy bufsize:%08x \n", __func__, cmd, ALLOC_INI_SIZE);
		desc->len = ALLOC_INI_SIZE;
	}

	dev_bin_attr_buffer = &desc->dev_bin_attr_buffer;
	desc->dev.release = op_release;
	desc->dev.parent = &curlun->dev;
	dev_set_drvdata(&desc->dev, curlun);
	dev_set_name(&desc->dev,"opcode-%02x", cmd);
	status = device_register(&desc->dev);
	if (status != 0) {
		printk("[lismo_buf_cntrol]%s failed to register opcode:%02x status:%d\n",__func__,cmd, status);
		goto done;
	}

	dev_bin_attr_buffer->attr.name = "buffer";
	if (init)
		dev_bin_attr_buffer->attr.mode = 0660;
	else
		dev_bin_attr_buffer->attr.mode = 0606;
	dev_bin_attr_buffer->read = vendor_cmd_read_buffer;
	dev_bin_attr_buffer->write = vendor_cmd_write_buffer;
	dev_bin_attr_buffer->mmap = vendor_cmd_mmap_buffer;
	if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT ){
		pr_debug("%s opcode:%02x bufsize:%08x \n", __func__, cmd, 2048);
		dev_bin_attr_buffer->size = 2048;
	}else{
		pr_debug("%s opcode:%02x bufsize:%08x \n", __func__, cmd, ALLOC_INI_SIZE);
		dev_bin_attr_buffer->size = ALLOC_INI_SIZE;
	}
	dev_bin_attr_buffer->private = desc;
	status = device_create_bin_file(&desc->dev, dev_bin_attr_buffer);

	if (status != 0) {
		printk("[lismo_buf_cntrol]%s failed to create buffer opcode:%02x status:%d\n",__func__,cmd, status);
		device_remove_bin_file(&desc->dev, dev_bin_attr_buffer);
		if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT )
			kfree(desc->buffer);
		desc->buffer = 0;
		desc->len = 0;
		device_unregister(&desc->dev);
		goto done;
	}

	if (init){
		dev_attr_size.attr.mode = 0660;
		dev_attr_update.attr.mode = 0660;
	} else {
		dev_attr_size.attr.mode = 0606;
		dev_attr_update.attr.mode = 0606;
	}

	status = device_create_file(&desc->dev, &dev_attr_size);
	if (status != 0) {
		printk("[lismo_buf_cntrol]%s failed to create size opcode:%02x status:%d\n",__func__,cmd, status);
		device_remove_bin_file(&desc->dev, dev_bin_attr_buffer);
		device_remove_file(&desc->dev, &dev_attr_size);
		if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT )
			kfree(desc->buffer);
		desc->buffer = 0;
		desc->len = 0;
		device_unregister(&desc->dev);
		goto done;
	}

	status = device_create_file(&desc->dev, &dev_attr_update);
	if (status != 0) {
		printk("[lismo_buf_cntrol]%s failed to create update opcode:%02x status:%d\n",__func__,cmd, status);
		device_remove_file(&desc->dev, &dev_attr_update);
		device_remove_file(&desc->dev, &dev_attr_size);
		device_remove_bin_file(&desc->dev, dev_bin_attr_buffer);
		if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT )
			kfree(desc->buffer);
		desc->buffer = 0;
		desc->len = 0;
		device_unregister(&desc->dev);
		goto done;
	}

	desc->value_sd = sysfs_get_dirent(desc->dev.kobj.sd, NULL, "update");
	INIT_WORK(&desc->work, buffer_notify_sysfs);
	
	if (status == 0)
		set_bit(FLAG_EXPORT, &desc->flags);

	desc->update = 0;
	pr_debug("[lismo_buf_cntrol]%s opcode:%02x success\n",__func__,cmd);

done:
	if (status)
		printk("[lismo_buf_cntrol]%s opcode%d status %d\n",__func__,cmd,status);
	return status;
}

/* vendor command delete */
static void vendor_cmd_unexport(struct device *dev, unsigned cmd)
{
	struct fsg_lun	*curlun = fsg_lun_from_dev(dev);
	struct op_desc *desc;
	int status = -EINVAL;
	pr_debug("[lismo_buf_cntrol]%s start \n",__func__);

	desc = curlun->op_desc[cmd-SC_VENDOR_START];
	if (!desc) {
		status = -ENODEV;
		printk("[lismo_buf_cntrol]%s desc not ready\n",__func__);
		goto done;
	}

	if (test_bit(FLAG_EXPORT, &desc->flags)) {
		struct bin_attribute* dev_bin_attr_buffer = &desc->dev_bin_attr_buffer;
		clear_bit(FLAG_EXPORT, &desc->flags);
		cancel_work_sync(&desc->work);
		device_remove_file(&desc->dev, &dev_attr_update);
		device_remove_file(&desc->dev, &dev_attr_size);
		device_remove_bin_file(&desc->dev, dev_bin_attr_buffer);
		if ( cmd-SC_VENDOR_START+1 > ALLOC_CMD_CNT || desc->len != ALLOC_INI_SIZE){
			kfree(desc->buffer);
			pr_debug("%s opcode:%02x free buff\n", __func__, cmd);
		}else{
			printk("%s opcode:%02x Not Free buff\n", __func__, cmd);
		}
		desc->buffer = 0;
		desc->len = 0;
		status = 0;
		device_unregister(&desc->dev);
		kfree(desc);
		curlun->op_desc[cmd-SC_VENDOR_START] = 0;
		pr_debug("[lismo_buf_cntrol]%s opcode:%02x success\n",__func__,cmd);
	} else {
		status = -ENODEV;
		printk("[lismo_buf_cntrol]%s not export opcode:%02x\n",__func__,cmd);
	}

done:
	if (status)
		printk("[lismo_buf_cntrol]%s opcode=%d status=%d\n",__func__,cmd,status);
}


/* when 'export'file update */
static ssize_t vendor_export_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
	long cmd;
	int status;
	pr_debug("[lismo_buf_cntrol]%s \n",__func__);

	status = strict_strtol(buf, 0, &cmd);
	if (status < 0){
		printk("[lismo_buf_cntrol]%s command-id invalid\n",__func__);
		goto done;
	}

	status = -EINVAL;

	if (!vendor_cmd_is_valid(cmd)){
		printk("[lismo_buf_cntrol]%s command-id out of range\n",__func__);
		goto done;
	}

	mutex_lock(&sysfs_lock);

	status = vendor_cmd_export(dev, cmd, 0);
	if (status < 0){
		vendor_cmd_unexport(dev, cmd);
		printk("[lismo_buf_cntrol]%s export fail\n",__func__);
	}

	mutex_unlock(&sysfs_lock);
done:
	if (status)
		pr_debug("%s: status %d\n", __func__, status);
	return status ? : len;
}

/* define 'export'file */
static DEVICE_ATTR(export, 0220, 0, vendor_export_store);

/* when 'unexport'file update */
static ssize_t vendor_unexport_store(struct device *dev,
                struct device_attribute *attr, const char *buf, size_t len)
{
	long cmd;
	int status;
	pr_debug("[lismo_buf_cntrol]%s \n",__func__);

	status = strict_strtol(buf, 0, &cmd);
	if (status < 0){
		printk("[lismo_buf_cntrol]%s command-id invalid\n",__func__);
		goto done;
	}

	status = -EINVAL;

	if (!vendor_cmd_is_valid(cmd)){
		printk("[lismo_buf_cntrol]%s command-id out of range\n",__func__);
		goto done;
	}

	mutex_lock(&sysfs_lock);

	status = 0;
	vendor_cmd_unexport(dev, cmd);

	mutex_unlock(&sysfs_lock);
done:
	if (status)
		pr_debug("%s: status %d\n", __func__, status);
	return status ? : len;
}
/* define 'unexport'file */
static DEVICE_ATTR(unexport, 0220, 0, vendor_unexport_store);

static void op_release(struct device *dev)
{
}

static void lun_release(struct device *dev)
{
	struct fsg_lun *curlun = fsg_lun_from_dev(dev);
	pr_debug("[lismo_buf_cntrol]%s \n",__func__);

	if (likely(curlun)) {
		unsigned j;
		/* In error recovery curlun may be zero. */
		for (j=SC_VENDOR_START; j < SC_VENDOR_END + 1; j++) {
			vendor_cmd_unexport(&curlun->dev, j);
			if ( j-SC_VENDOR_START < ALLOC_CMD_CNT ){
				pr_debug("%s kfree buf[%d]\n", __func__,j-SC_VENDOR_START);
				kfree(curlun->reserve_buf[j-SC_VENDOR_START]);
			}
		}
		device_remove_file(&curlun->dev, &dev_attr_export);
		device_remove_file(&curlun->dev, &dev_attr_unexport);
		device_unregister(&curlun->dev);

		kfree(curlun);
	}
}


static int lun_regist(struct platform_device *pdev)
{

	struct fsg_lun *curlun;
        int rc = -EINVAL;
	int i;
	
	pr_debug("[lismo_buf_cntrol]%s pdev.name:%s\n",__func__,dev_name(&pdev->dev));
#if 0
	if (!pdev->dev.platform_data){
		printk("[lismo_buf_cntrol]%s ERROR:pdev is null\n",__func__);
		return -1;
	}
#endif		
	/* Create the LUNs, open their backing files, and register the
	 * LUN devices in sysfs. */
	curlun = kzalloc(sizeof *curlun, GFP_KERNEL);
	if (unlikely(!curlun)) {
		rc = -ENOMEM;
		printk("[lismo_buf_cntrol]%s ERROR:curlun alloc fail\n",__func__);
		goto error_release;
	}

	/* use "usb_mass_storage" platform device as parent */
	curlun->dev.parent = &pdev->dev;
	dev_set_name(&curlun->dev,"lun0");
	curlun->dev.release = lun_release;

	rc = device_register(&curlun->dev);
	if (rc) {
		printk("[lismo_buf_cntrol]%s ERROR:failed to register LUN0\n",__func__);
		goto error_release;
	}

	rc = device_create_file(&curlun->dev, &dev_attr_export);
	if (rc){
		printk("[lismo_buf_cntrol]%s ERROR:failed to create export\n",__func__);
		goto error_release;
	}
	rc = device_create_file(&curlun->dev, &dev_attr_unexport);
	if (rc){
		printk("[lismo_buf_cntrol]%s ERROR:failed to create unexport\n",__func__);
		goto error_release;
	}

	/* alloc for common buffer */
	for (i=0; i < ALLOC_CMD_CNT; i++){
		curlun->reserve_buf[i] = kzalloc(ALLOC_INI_SIZE, GFP_KERNEL);
		pr_debug("%s alloc buf[%d]\n", __func__,i);
		if(!curlun->reserve_buf[i]){
			printk("%s Error : buffer malloc fail! cmd_idx=%d \n", __func__, i);
			rc = -ENOMEM;
			goto error_release;
		}
	}
	rc = vendor_cmd_export(&curlun->dev, 0xe4, 1);
	if (rc < 0){
		vendor_cmd_unexport(&curlun->dev, 0xe4);
		goto error_release;
	}

	pr_debug("[lismo_buf_cntrol]%s OK\n",__func__);
	return 0;

error_release:
	printk("[lismo_buf_cntrol]%s ERROR rc:%08x\n",__func__,rc);
	lun_release(&curlun->dev);
	return rc;
}

static struct platform_driver lun_platform_driver = {
	.driver = { .name = LUN_FUNCTION_NAME, },
//	.probe = lun_regist,
};

static int lun_device_init(void)
{
	int rc;

	/* allocate device structure */
	lun_device = platform_device_alloc( LUN_FUNCTION_NAME , -1 );
	if ( lun_device == NULL ) {
		printk("[lismo_buf_cntrol]%s device alloc fail\n",__func__);
		return -ENOMEM;
	}

	/* regist device */
	rc = platform_device_add( lun_device );
	if ( rc != 0 )
		printk("[lismo_buf_cntrol]%s device register failed rc:%08x\n", __func__, rc);

	return rc;
}

//static int __init lun_init(void)
static int lun_init(void)
{
	int rc;
	pr_debug("[lismo_buf_cntrol]%s \n",__func__);

	rc = lun_device_init();
	if ( rc == 0 ) {
		rc = platform_driver_probe(&lun_platform_driver, lun_regist);
		if (rc != 0)
			printk("[lismo_buf_cntrol]%s unable to register rc:%08x\n", __func__, rc);
	}

	pr_debug("[lismo_buf_cntrol]%s platform_driver_probe rc:%08x\n", __func__, rc);
	return rc;
}

//module_init(lun_init);
