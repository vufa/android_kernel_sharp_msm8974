/* drivers/sharp/shdiag/smd_read.c
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

#include <linux/module.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/poll.h>

#include <sharp/sh_smem.h>

#include <sharp/shdiag_smd.h>

#include <mach/gpio.h>
#include <mach/msm_iomap.h>
#include <linux/io.h>

#define SHDIAG_GPIO_CFG(n) (MSM_TLMM_BASE + 0x1000 + (0x10 * n))

/* Soft Update Flag */
#define D_SHSOFTUP_F_MASK	0x00000010		/* bit4 */
#define D_SHSOFTUP_F_SHIFT	4

static int smd_mode_open(struct inode *inode, struct file *filp)
{
/*	printk("%s\n", __func__);*/
	return 0;
}


static ssize_t smd_mode_read(struct file *filp, char __user *buf,size_t count, loff_t *ppos)
{
	sharp_smem_common_type *p_sh_smem_common_type = NULL;
	struct smem_comm_mode  smem_comm_data;
	unsigned long UpDateFlgStatus;

/*	printk("%s\n", __func__);*/
	if(count != sizeof(smem_comm_data)){
		return -EINVAL;
	}
	
	p_sh_smem_common_type = sh_smem_get_common_address();
	if( p_sh_smem_common_type != NULL){
		smem_comm_data.BootMode  = p_sh_smem_common_type->shdiag_BootMode;

		UpDateFlgStatus = p_sh_smem_common_type->shdiag_FlagData;
		smem_comm_data.UpDateFlg = ( UpDateFlgStatus & D_SHSOFTUP_F_MASK )>>D_SHSOFTUP_F_SHIFT;

		/* user aera */
		if( copy_to_user( buf, (void *)&smem_comm_data, sizeof(smem_comm_data) ) ){
			printk( "copy_to_user failed\n" );
			return -EFAULT;
		}
	} else {
		printk("[SH]smd_read_probe: smem_alloc FAILE\n");
	}
	return count;
}

static ssize_t smd_mode_write(struct file *filp, const char __user *buf,size_t count, loff_t *ppos)
{
/*	printk("%s\n", __func__);*/
	return count;
}

static int smd_mode_release(struct inode *inode, struct file *filp)
{
/*	printk("%s\n", __func__);*/
	return 0;
}

static int smd_ioctl_set_qxdmflg(unsigned long arg)
{
	int ret = 0;
	unsigned char qxdm_work;
	sharp_smem_common_type *sharp_smem;
	
	if(copy_from_user(&qxdm_work, (unsigned short __user *)arg, sizeof(unsigned char)) != 0)
	{
		printk("[SH]smd_ioctl_set_qxdmflg: copy_to_user FAILE\n");
		ret = -EFAULT;
	}
	else
	{
		sharp_smem = sh_smem_get_common_address();
		sharp_smem->shusb_qxdm_ena_flag = qxdm_work;
	}
	
	return ret;
}

static int smd_ioctl_set_proadj(unsigned long arg)
{
	int ret = 0;
	struct shdiag_procadj procadj_work;
	sharp_smem_common_type *sharp_smem;
	
	if(copy_from_user(&procadj_work, (unsigned short __user *)arg, sizeof(struct shdiag_procadj)) != 0)
	{
		printk("[SH]smd_ioctl_set_proadj: copy_to_user FAILE\n");
		ret = -EFAULT;
	}
	else
	{
		sharp_smem = sh_smem_get_common_address();
		sharp_smem->shdiag_proxadj[0] = (unsigned short)(procadj_work.proxcheckdata_min & 0x0000FFFF);
		sharp_smem->shdiag_proxadj[1] = (unsigned short)(procadj_work.proxcheckdata_max & 0x0000FFFF);
	}
	
	return ret;
}

static int smd_ioctl_get_hw_revision(unsigned long arg)
{
	int ret = 0;
	sharp_smem_common_type *sharp_smem;
	unsigned long hw_revision;
	
	sharp_smem = sh_smem_get_common_address();
	if( sharp_smem != 0 )
	{
		hw_revision = sharp_smem->sh_hw_revision;
		if(copy_to_user((unsigned long __user *)arg, &hw_revision, sizeof(unsigned long)) != 0)
		{
			printk("[SH]smd_ioctl_get_hw_revision: copy_to_user FAILE\n");
			ret = -EFAULT;
		}
	}else{
		printk("[SH]smd_ioctl_get_hw_revision: get smem_addr FAILE\n");
		return -EFAULT;
	}
	
	return ret;
}

static int smd_ioctl_set_hapticscal(unsigned long arg)
{
	int ret = 0;
	struct shdiag_hapticscal hapticscal_work;
	sharp_smem_common_type *sharp_smem;
	
	if(copy_from_user(&hapticscal_work, (unsigned short __user *)arg, sizeof(struct shdiag_hapticscal)) != 0)
	{
		printk("[SH]smd_ioctl_set_hapticscal: copy_to_user FAILE\n");
		ret = -EFAULT;
	}
	else
	{
		sharp_smem = sh_smem_get_common_address();
		memcpy( (void*)(sharp_smem->shdiag_tspdrv_acal_data), hapticscal_work.buf, SHDIAG_HAPTICSCAL_SIZE );
	}
	
	return ret;
}

static int smd_ioctl_set_gpio_pull(unsigned long arg)
{
	int ret = 0;
	int gpio_ret = 0;
	unsigned gpio_config = 0;
	struct shdiag_gpio gpio_work;
	int *cfg_reg;
	int cfg_pull = 0;
	int cfg_func = 0;
	int cfg_drvstr = 0;
	int cfg_dir = 0;
	
	if(copy_from_user(&gpio_work, (unsigned short __user *)arg, sizeof(struct shdiag_gpio)) != 0)
	{
		printk("[SH]smd_ioctl_set_gpio_pull: copy_to_user FAILE\n");
		ret = -EFAULT;
	}
	else
	{
		printk("[SH]smd_ioctl_set_gpio_pull: port = %d pull = %d\n", (int)gpio_work.port, (int)gpio_work.pull);
		
		if( gpio_work.pull > GPIO_CFG_PULL_UP )
		{
			printk("[SH]smd_ioctl_set_gpio_pull: invalid value pull setting\n");
			ret = -EFAULT;
		}
		else
		{
			/* get gpio config */
			cfg_reg = SHDIAG_GPIO_CFG(gpio_work.port);
			printk("[SH]smd_ioctl_set_gpio_pull: cfg_reg = %d *cfg_reg = %d\n", (int)cfg_reg, *cfg_reg);
			
			/* set pull */
			cfg_pull = gpio_work.pull;
			/* set func : 2-5bit */
			cfg_func = ( *cfg_reg >> 2 ) & 0x0000000F;
			/* set drvstr : 6-7bit */
			cfg_drvstr = ( *cfg_reg >> 6 ) & 0x00000007;
			/* set dir : 9bit */
			cfg_dir = ( *cfg_reg >> 9 ) & 0x00000001;
		
			printk("[SH]smd_ioctl_set_gpio_pull: cfg_pull = %d cfg_func = %d cfg_drvstr = %d cfg_dir = %d\n", cfg_pull, cfg_func, cfg_drvstr, cfg_dir);
			
			/* set gpio config */
			gpio_config = GPIO_CFG( gpio_work.port, cfg_func, cfg_dir, cfg_pull, cfg_drvstr );
			
			gpio_ret = gpio_tlmm_config( gpio_config, GPIO_CFG_ENABLE );
			if( gpio_ret < 0 )
			{
				printk("[SH]smd_ioctl_set_gpio_pull: gpio_tlmm_config FAILE\n");
				ret = -EFAULT;
			}
		}
	}
	
	return ret;
}

static long smd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	
	switch(cmd) {
	case SHDIAG_IOCTL_SET_QXDMFLG:
		ret = smd_ioctl_set_qxdmflg(arg);
		break;
	case SHDIAG_IOCTL_SET_PROADJ:
		ret = smd_ioctl_set_proadj(arg);
		break;
	case SHDIAG_IOCTL_GET_HW_REVISION:
		ret = smd_ioctl_get_hw_revision(arg);
		break;
	case SHDIAG_IOCTL_SET_HAPTICSCAL:
		ret = smd_ioctl_set_hapticscal(arg);
		break;
	case SHDIAG_IOCTL_SET_GPIO_PULL:
		ret = smd_ioctl_set_gpio_pull(arg);
		break;
	default:
		printk("[SH]smd_ioctl: cmd FAILE\n");
		ret = -EFAULT;
		break;
	}

	return ret;
}

static struct file_operations smd_mode_fops = {
	.owner		= THIS_MODULE,
	.read		= smd_mode_read,
	.write		= smd_mode_write,
	.open		= smd_mode_open,
	.release	= smd_mode_release,
	.unlocked_ioctl = smd_ioctl,
};

static struct miscdevice smd_mode_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "smd_read",
	.fops = &smd_mode_fops,
};

static int __init smd_mode_init( void )
{
	int ret;

	ret = misc_register(&smd_mode_dev);
	if (ret) {
		printk("fail to misc_register (smd_mode_dev)\n");
		return ret;
	}
	printk("smd_mode loaded.\n");
	return 0;
}

module_init(smd_mode_init);

MODULE_DESCRIPTION("smd_read");
MODULE_LICENSE("GPL v2");

