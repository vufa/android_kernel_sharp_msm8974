/* drivers/sharp/shvibrator/sh_vibrator.c
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

#include <linux/module.h>

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/hrtimer.h>
#include "../../staging/android/timed_output.h"
#include <linux/sched.h>

#include <linux/gpio.h>
#include <linux/qpnp/pin.h>
#include <linux/regulator/consumer.h> /* for regulator_xx() */
#include <linux/delay.h>
#include <linux/wakelock.h>

#include <mach/pmic.h>
#include <mach/msm_rpcrouter.h>

#include <linux/slab.h>
#include <linux/i2c.h>

#include <linux/fs.h>
#include <asm/uaccess.h> /* for struct file_operations */
#include <linux/cdev.h>

#include <sharp/sh_smem.h>

#ifdef CONFIG_SHTERM
#include <sharp/shterm_k.h>
#endif /* CONFIG_SHTERM */
#include <sharp/sh_boot_manager.h>

#define SHVIB_DRIVER_NAME	"shvibrator"

#define SH_VIB_EN		qpnp_pin_map("pm8941-gpio", 29)
#define SH_VIB_REGULATOR_NAME	"8941_l10"

#define SH_VIB_START_WAIT_TIME	(1000)
#define SH_VIB_STOP_WAIT_TIME	(30 * 1000)

#define SH_VIB_DEFAULT_TIMEOUT	15000

/* adb debug_log */
static int shvibrator_debug_log = 0;
static int shvibrator_error_log = 1;

#ifdef CONFIG_ANDROID_ENGINEERING
module_param(shvibrator_debug_log, int, 0600);
module_param(shvibrator_error_log, int, 0600);
#endif  /* CONFIG_ANDROID_ENGINEERING */

#define SHVIBRATOR_DBG_LOG(fmt, args...) \
	if( shvibrator_debug_log == 1 ){ \
		printk( KERN_DEBUG "[shvibrator][%d][%s] " fmt "\n", __LINE__, __func__, ## args ); \
	}
#define SHVIBRATOR_ERR_LOG(fmt, args...) \
	if( shvibrator_error_log == 1 ){ \
		printk( KERN_DEBUG "[shvibrator] " fmt "\n",## args ); \
	} \
	else if( shvibrator_error_log == 2 ){ \
		printk( KERN_DEBUG "[shvibrator][%d][%s] " fmt "\n", __LINE__, __func__, ## args ); \
	}

static struct work_struct work_vibrator_on;
static struct work_struct work_vibrator_off;
static struct work_struct work_hrtimer;
static struct hrtimer shvib_timer;

static struct regulator *shvib_vreg = NULL;

static int sh_vibrator_reqtime = 0;
static struct mutex sh_vibrator_pmic_mutex;
static struct wake_lock shvibrator_linear_wake_lock;

static int shvibrator_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid);
static int shvibrator_i2c_remove(struct i2c_client *client);
static int shvibrator_i2c_suspend(struct i2c_client *client, pm_message_t mesg);

int shvibrator_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size);
int shvibrator_i2c_write(struct i2c_client *client,
									int reg, u8 data);
static int vibrator_init = 0;
struct shvib_i2c_data {
	struct i2c_client *client_p;
};
struct shvib_i2c_data *vib_data = NULL;

static dev_t 		shvib_devid;
static struct class*	shvib_class;
static struct device*	shvib_device;
struct cdev 		shvib_cdev;


static void shvibrator_set(int on)
{
	mutex_lock(&sh_vibrator_pmic_mutex);

	if (on) {
		if (!regulator_is_enabled(shvib_vreg)) {
			usleep(SH_VIB_START_WAIT_TIME);
			SHVIBRATOR_DBG_LOG("usleep = %d", SH_VIB_START_WAIT_TIME);
			/* regulator ON ( & RESET release) */
			regulator_enable(shvib_vreg);
			SHVIBRATOR_DBG_LOG("regulator ON");
		}

		/* Wait */
		usleep(SH_VIB_START_WAIT_TIME);
		SHVIBRATOR_DBG_LOG("usleep = %d", SH_VIB_START_WAIT_TIME);

		if (vib_data) {
			/* I2C control */
			if (shvibrator_i2c_write(vib_data->client_p, 0x01, 0x0D) < 0) goto set_pmic_vibrator_i2c_err;
			if (shvibrator_i2c_write(vib_data->client_p, 0x02, 0x0B) < 0) goto set_pmic_vibrator_i2c_err;
			if (shvibrator_i2c_write(vib_data->client_p, 0x03, 0x05) < 0) goto set_pmic_vibrator_i2c_err;
			if (shvibrator_i2c_write(vib_data->client_p, 0x04, 0x6F) < 0) goto set_pmic_vibrator_i2c_err;

			SHVIBRATOR_DBG_LOG("i2c write OK");
		} else {
			SHVIBRATOR_ERR_LOG("i2c regist error");
			goto set_pmic_vibrator_i2c_err;
		}
		/* EN pin is High */
		gpio_set_value(SH_VIB_EN, 1);
		SHVIBRATOR_DBG_LOG("VIB IC EN: High");
	} else {
		/* EN pin is Low */
		gpio_set_value(SH_VIB_EN, 0);
		SHVIBRATOR_DBG_LOG("VIB IC EN: Low");

		if (regulator_is_enabled(shvib_vreg)) {
			/* Wait for braking */
			usleep(SH_VIB_STOP_WAIT_TIME);
			SHVIBRATOR_DBG_LOG("usleep = %d", SH_VIB_STOP_WAIT_TIME);
			/* regulator OFF ( & RESET ) */
			regulator_disable(shvib_vreg);
			SHVIBRATOR_DBG_LOG("regulator OFF");
		} else {
			SHVIBRATOR_DBG_LOG("OFF during OFF : do nothing");
		}

		wake_unlock(&shvibrator_linear_wake_lock);
		SHVIBRATOR_DBG_LOG("shvibrator_linear_wake_lock : unlock ");
	}
	mutex_unlock(&sh_vibrator_pmic_mutex);

	return;
	
set_pmic_vibrator_i2c_err:

	SHVIBRATOR_ERR_LOG("i2c write error");

	mutex_unlock(&sh_vibrator_pmic_mutex);

	return;
}

static void pmic_vibrator_on(struct work_struct *work)
{
/* [BatteryTemperatureLog] [start] */
#ifdef CONFIG_SHTERM
    pr_debug("%s() shterm_k_set_info( SHTERM_INFO_VIB, 1 ) \n", __func__);
    shterm_k_set_info( SHTERM_INFO_VIB, 1 );
#endif /* CONFIG_SHTERM */
/* [BatteryTemperatureLog] [end] */

	shvibrator_set(1);
	
	hrtimer_start(&shvib_timer,
		      ktime_set(sh_vibrator_reqtime / 1000, (sh_vibrator_reqtime % 1000) * 1000000),
		      HRTIMER_MODE_REL);
	pr_debug("[shvibrator] timer start. %d \n", sh_vibrator_reqtime);
}

static void pmic_vibrator_off(struct work_struct *work)
{
	shvibrator_set(0);

/* [BatteryTemperatureLog] [start] */
#ifdef CONFIG_SHTERM
    pr_debug("%s() shterm_k_set_info( SHTERM_INFO_VIB, 0 ) \n", __func__);
    shterm_k_set_info( SHTERM_INFO_VIB, 0 );
#endif /* CONFIG_SHTERM */
/* [BatteryTemperatureLog] [end] */
}

static void timed_vibrator_on(struct timed_output_dev *sdev)
{
	if(schedule_work(&work_vibrator_on) != 1){
		pr_debug("[shvibrator] update vibrator on workqueue\n");
		cancel_work_sync(&work_vibrator_on);
		schedule_work(&work_vibrator_on);
	}
}

static void timed_vibrator_off(struct timed_output_dev *sdev)
{
	if(schedule_work(&work_vibrator_off) != 1){
		pr_debug("[shvibrator] update vibrator off workqueue\n");
		cancel_work_sync(&work_vibrator_off);
		schedule_work(&work_vibrator_off);
	}
}

static void shvibrator_enable(struct timed_output_dev *dev, int value)
{
	pr_debug("[shvibrator] value=%d.\n", value);
	SHVIBRATOR_DBG_LOG("value = %d", value);
	
	hrtimer_cancel(&shvib_timer);

	cancel_work_sync(&work_hrtimer);
	cancel_work_sync(&work_vibrator_on);
	cancel_work_sync(&work_vibrator_off);

	if (value == 0)
		timed_vibrator_off(dev);
	else {
		wake_lock(&shvibrator_linear_wake_lock);
		SHVIBRATOR_DBG_LOG("shvibrator_linear_wake_lock : lock ");
		if (sh_boot_get_bootmode() == SH_BOOT_NORMAL) {
			sh_vibrator_reqtime = (value > SH_VIB_DEFAULT_TIMEOUT ? SH_VIB_DEFAULT_TIMEOUT : value);
			SHVIBRATOR_DBG_LOG("sh_vibrator_reqtime = %d", sh_vibrator_reqtime);
		} else {
			sh_vibrator_reqtime = value;
			SHVIBRATOR_DBG_LOG("reqtime is not changed");
		}
		timed_vibrator_on(dev);
	}
}

static int shvibrator_get_time(struct timed_output_dev *dev)
{
	if (hrtimer_active(&shvib_timer)) {
		ktime_t r = hrtimer_get_remaining(&shvib_timer);
		SHVIBRATOR_DBG_LOG("(int)ktime_to_ms(r) = %d", (int)ktime_to_ms(r));		/* linux/ktime.h */
		return (int)ktime_to_ms(r);
	} else {
		return 0;
	}
}

static enum hrtimer_restart shvibrator_timer_func(struct hrtimer *timer)
{
	SHVIBRATOR_DBG_LOG("timer stop.");
	schedule_work(&work_hrtimer);

	return HRTIMER_NORESTART;
}

static void hrtimer_work_func(struct work_struct *work)
{
	pmic_vibrator_off(NULL);
}

int shvibrator_i2c_read(struct i2c_client *client,
									int reg, u8 *data, int size)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags= 0,
			.len  = 1,
			.buf  = buf,
		},
		{
			.addr = client->addr,
			.flags= I2C_M_RD,
			.len  = size,
			.buf  = data,
		}
	};

	buf[0] = reg;
	rc = i2c_transfer(client->adapter, msg, 2);
	SHVIBRATOR_DBG_LOG("i2c read: reg = 0x%02X  data = 0x%02X",reg, *data);
	if(rc != 2){
		dev_err(&client->dev,
		       "shvibrator_i2c_read FAILED: read of register 0x%02X\n", reg);
		rc = -1;
		goto i2c_rd_exit;
	}

i2c_rd_exit:
	return rc;
}

int shvibrator_i2c_write(struct i2c_client *client,
									int reg, u8 data)
{
	int rc;
	u8 buf[2];
	struct i2c_msg msg = {
		.addr = client->addr,
		.flags= 0,
		.len  = 2,
		.buf  = buf,
	};

	SHVIBRATOR_DBG_LOG("i2c write: reg = 0x%02X  data = 0x%02X",reg, data);
	
	buf[0] = reg;
	buf[1] = data;
	rc = i2c_transfer(client->adapter, &msg, 1);
	if(rc != 1){
		dev_err(&client->dev,
		       "shvibrator_i2c_write FAILED: writing to reg 0x%02X\n", reg);
		rc = -1;
	}

	return rc;
}


/* ################################ for diag ################################ */
static int vib_connect_check(void)
{
	int rc = -1;
	int i2c_err = 0;

	SHVIBRATOR_DBG_LOG();

	if (!regulator_is_enabled(shvib_vreg)) {
		usleep(SH_VIB_START_WAIT_TIME);
		SHVIBRATOR_DBG_LOG("usleep = %d", SH_VIB_START_WAIT_TIME);
		/* regulator ON ( & RESET release) */
		regulator_enable(shvib_vreg);
		SHVIBRATOR_DBG_LOG("regulator ON");
	}

	usleep(SH_VIB_START_WAIT_TIME);
	SHVIBRATOR_DBG_LOG("usleep = %d", SH_VIB_START_WAIT_TIME);

	if (vib_data) {
		u8 getval;

		if( shvibrator_i2c_write(vib_data->client_p, 0x03, 0x05) < 0)
			i2c_err++;
		if( shvibrator_i2c_read(vib_data->client_p, 0x03, &getval ,1) < 0)
			i2c_err++;
		if ((getval == 0x05) && (i2c_err == 0))
			rc = 0;
	}

	if (regulator_is_enabled(shvib_vreg)) {
		/* regulator OFF ( & RESET ) */
		regulator_disable(shvib_vreg);
		SHVIBRATOR_DBG_LOG("regulator OFF");
	}

	return rc;
}

static int shvib_open(struct inode *inode, struct file *file)
{
	SHVIBRATOR_DBG_LOG();

	return 0;
}

static ssize_t shvib_write(struct file *filp, const char __user *ubuf, size_t count, loff_t *ppos)
{
	char cmd;

	if(get_user(cmd, ubuf)){
		SHVIBRATOR_ERR_LOG("get_user error");
		return -EFAULT;
	}

	SHVIBRATOR_DBG_LOG("cmd = %c", cmd);

	if (cmd == '2') {
		/* i2c connect check */
		if (vib_connect_check() < 0) {
			SHVIBRATOR_ERR_LOG("i2c connect check: error");
			return -1;
		} else {
			SHVIBRATOR_DBG_LOG("i2c connect check: OK");
		}
	} else {
		SHVIBRATOR_ERR_LOG("cmd error (cmd = %c)", cmd);
		return -1;
	}

	return count;
}

static int shvib_release(struct inode *inode, struct file *file)
{
	SHVIBRATOR_DBG_LOG();

	return 0;
}

static const struct file_operations shvib_fileops = {
	.owner   = THIS_MODULE,
	.open    = shvib_open,
	.write   = shvib_write,
	.release = shvib_release,
};

static int shvib_init(void)
{
	int rc = 0;

	rc = alloc_chrdev_region(&shvib_devid, 0, 1, SHVIB_DRIVER_NAME);
	if(rc < 0){
		SHVIBRATOR_ERR_LOG("alloc_chrdev_region error");
		return rc;
	}

	shvib_class = class_create(THIS_MODULE, SHVIB_DRIVER_NAME);
	if (IS_ERR(shvib_class)) {
		rc = PTR_ERR(shvib_class);
		SHVIBRATOR_ERR_LOG("class_create error");
		goto error_class_create;
	}

	shvib_device = device_create(shvib_class, NULL, shvib_devid, &shvib_cdev, SHVIB_DRIVER_NAME);
	if (IS_ERR(shvib_device)) {
		rc = PTR_ERR(shvib_device);
		SHVIBRATOR_ERR_LOG("device_create error");
		goto error_device_create;
	}

	cdev_init(&shvib_cdev, &shvib_fileops);
	shvib_cdev.owner = THIS_MODULE;
	rc = cdev_add(&shvib_cdev, shvib_devid, 1);
	if(rc < 0){
		SHVIBRATOR_ERR_LOG("cdev_add error");
		goto error_cdev_add;
	}

	return 0;

error_cdev_add:
	cdev_del(&shvib_cdev);
error_device_create:
	class_destroy(shvib_class);
error_class_create:
	unregister_chrdev_region(shvib_devid, 1);

	return rc;
}
/* ################################ for diag ################################ */


#ifdef CONFIG_OF
static struct of_device_id shvib_i2c_table[] = {
	{ .compatible = "sharp,shvib_lra" },
	{}
};
#else
#define shvib_i2c_table NULL
#endif /* CONFIG_OF */


static const struct i2c_device_id shvibrator_i2c_id[] = {
	{ "sh_vib_i2c", 0 },
	{ }
};

static struct i2c_driver shvibrator_i2c_driver = {
	.driver		= {
		.name = "sh_vib_i2c",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = shvib_i2c_table,
#endif /* CONFIG_OF */
	},
	.probe		= shvibrator_i2c_probe,
	.remove		= __devexit_p(shvibrator_i2c_remove),
	.suspend	= shvibrator_i2c_suspend,
	.id_table	= shvibrator_i2c_id
};

static int shvibrator_i2c_probe(struct i2c_client *client, const struct i2c_device_id *devid)
{
	int rc;
	struct shvib_i2c_data *sd;

	SHVIBRATOR_DBG_LOG();

	if(vib_data){
		SHVIBRATOR_DBG_LOG("vib_data fail");
		rc = -EPERM;
		goto probe_exit;
	}

	sd = (struct shvib_i2c_data*)kzalloc(sizeof *sd, GFP_KERNEL);
	if (!sd) {
		SHVIBRATOR_DBG_LOG("kzalloc fail");
		rc = -ENOMEM;
		goto probe_exit;
	}
	vib_data = sd;
	i2c_set_clientdata(client, sd);
	sd->client_p = client;

	return 0;

probe_exit:

	return rc;
}

static int shvibrator_i2c_remove(struct i2c_client *client)
{
	SHVIBRATOR_DBG_LOG();

	return 0;
}

static int shvibrator_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
	SHVIBRATOR_DBG_LOG();

	hrtimer_cancel(&shvib_timer);
	cancel_work_sync(&work_hrtimer);
	cancel_work_sync(&work_vibrator_on);
	cancel_work_sync(&work_vibrator_off);
	shvibrator_set(0);

	return 0;
}

static struct timed_output_dev shvib_driver = {
	.name = "vibrator",
	.get_time = shvibrator_get_time,
	.enable = shvibrator_enable,
};

static int __init shvibrator_init(void)
{
	int rc;

	if (vibrator_init) {
		SHVIBRATOR_ERR_LOG("already vibrator_init = 1");
		return 0;
	}
	rc = i2c_add_driver(&shvibrator_i2c_driver);	/* =i2c_register_driver (i2c.h/i2c-core.c) */
	if (rc) {
		pr_err("i2c_add_driver failed");
		return rc;
	}
	vibrator_init = 1;

	mutex_init(&sh_vibrator_pmic_mutex);

	INIT_WORK(&work_vibrator_on, pmic_vibrator_on);
	INIT_WORK(&work_vibrator_off, pmic_vibrator_off);
	INIT_WORK(&work_hrtimer, hrtimer_work_func);

	hrtimer_init(&shvib_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	shvib_timer.function = shvibrator_timer_func;

	rc = timed_output_dev_register(&shvib_driver);
	if (rc < 0)
		return rc;

	shvib_vreg = regulator_get(shvib_driver.dev, SH_VIB_REGULATOR_NAME);
	if (IS_ERR(shvib_vreg)) {
		SHVIBRATOR_ERR_LOG("regulator get error");
		return -1;
	}

	wake_lock_init(&shvibrator_linear_wake_lock, WAKE_LOCK_SUSPEND, "shvibrator_linear_wake_lock");

	/* for diag */
	rc = shvib_init();
	if (rc < 0)
		return rc;

	return rc;

}
module_init(shvibrator_init);

static void __exit shvibrator_exit(void)
{
	if (vibrator_init)
		i2c_del_driver(&shvibrator_i2c_driver);

	regulator_put(shvib_vreg);	/* release */

	timed_output_dev_unregister(&shvib_driver);
	wake_unlock(&shvibrator_linear_wake_lock);
	wake_lock_destroy(&shvibrator_linear_wake_lock);
}
module_exit(shvibrator_exit);

MODULE_AUTHOR("SHARP CORPORATION");
MODULE_DESCRIPTION("linear vibrator driver");
MODULE_LICENSE("GPL v2");
