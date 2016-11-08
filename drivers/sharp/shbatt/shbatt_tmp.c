/*
 * Copyright (C) 2011 Sharp Corporation
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

/*+-----------------------------------------------------------------------------+*/
/*| @ DEFINE COMPILE SWITCH :                                                   |*/
/*+-----------------------------------------------------------------------------+*/

	#define SHTMP_ENABLE_DEVELOP_BUILD
//	#define SHTMP_ENABLE_DEBUG

/*+-----------------------------------------------------------------------------+*/
/*| @ INCLUDE FILE :                                                            |*/
/*+-----------------------------------------------------------------------------+*/

#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/gpio.h>

#include "shbatt_tmp.h"

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL MACRO DECLARE :                                                     |*/
/*+-----------------------------------------------------------------------------+*/

#ifdef SHTMP_ENABLE_DEBUG

#define SHTMP_INFO(x...)	printk(KERN_INFO  "[SHTMP][INF] " x)
#define SHTMP_TRACE(x...)	printk(KERN_DEBUG "[SHTMP][TRC] " x)
#define SHTMP_ERROR(x...)	printk(KERN_ERR   "[SHTMP][ERR] " x)

#else  /* SHTMP_ENABLE_DEBUG */

#define SHTMP_INFO(x...)	do {} while(0)
#define SHTMP_TRACE(x...)	do {} while(0)
#define SHTMP_ERROR(x...)	do {} while(0)

#endif /* SHTMP_ENABLE_DEBUG */

#ifdef SHTMP_ENABLE_DEVELOP_BUILD
static int debug_tmp = 0;
module_param_named(debug_tmp, debug_tmp, int, S_IRUSR | S_IWUSR );
#endif /* SHTMP_ENABLE_DEVELOP_BUILD */

#define TMP103_TEMP_REG			0x00
#define TMP103_CONF_REG			0x01
#define TMP103_CONF_ID			BIT(7) /*0x80*/
#define TMP103_CONF_CR1			BIT(6) /*0x40*/
#define TMP103_CONF_CR0			BIT(5) /*0x20*/
#define TMP103_CONF_FH			BIT(4) /*0x10*/
#define TMP103_CONF_FL			BIT(3) /*0x08*/
#define TMP103_CONF_LC			BIT(2) /*0x04*/
#define TMP103_CONF_M1			BIT(1) /*0x02*/
#define TMP103_CONF_M0			BIT(0) /*0x01*/
#define TMP103_TLOW_REG			0x02
#define TMP103_THIGH_REG		0x03

#define TMP103_CONF_SD			(TMP103_CONF_M1 | TMP103_CONF_M0)
#define TMP103_CONF_DEFAULT		TMP103_CONF_M1
#define TMP103_CONF_INIT		(TMP103_CONF_CR0 | TMP103_CONF_M1)

#define SHTMP_NO_IC_TEMP		0
#define SHTMP_IC_DISABLE_TEMP	70
#define SHTMP_RESUME_THRESH		20

/*+-----------------------------------------------------------------------------+*/
/*| @ STRUCT & UNION DECLARE :                                                  |*/
/*+-----------------------------------------------------------------------------+*/

typedef struct shtmp_i2c_data_tag
{
	struct i2c_client* clt_p;

} shtmp_i2c_data_t;

/*+-----------------------------------------------------------------------------+*/
/*| @ STATIC VARIABLE :                                                         |*/
/*+-----------------------------------------------------------------------------+*/

static const u8 tmp103_reg[] =
{
	TMP103_TEMP_REG,
    TMP103_CONF_REG,
	TMP103_TLOW_REG,
	TMP103_THIGH_REG,
};

static shtmp_i2c_data_t* shtmp_i2c_p = NULL;

static int shtmp_ic_use_flg = 0;
static int shtmp_ic_disable_flg = 0;

static struct mutex shtmp_i2c_lock;

static int8_t shtmp_prev_temp = 25;
static int shtmp_resume_flg = 0;

/*+-----------------------------------------------------------------------------+*/
/*| @ EXTERN FUNCTION PROTO TYPE DECLARE :                                      |*/
/*+-----------------------------------------------------------------------------+*/

/* NONE.. */

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION PROTO TYPE DECLARE :                                       |*/
/*+-----------------------------------------------------------------------------+*/

/* Read Register */
static shtmp_result_t shtmp_i2c_read_reg( uint8_t *value, uint8_t len, uint8_t reg );

/* Write Register */
static shtmp_result_t shtmp_i2c_write_reg( uint8_t *value, uint8_t len, uint8_t reg );

/* Check HW Revision and HW Type*/
static shtmp_result_t shtmp_check_HW_type_revision(void);

#define DBGB_SW_BIT		31
#define GPIO_GET(gpio)	((__gpio_cansleep(gpio) == 0) ? gpio_get_value(gpio) : gpio_get_value_cansleep(gpio))

/*+-----------------------------------------------------------------------------+*/
/*| @ PUBLIC FUNCTION'S CODE AREA :                                             |*/
/*+-----------------------------------------------------------------------------+*/

shtmp_result_t shtmp_get_temperature( int* temp_p )
{
	shtmp_result_t result = SHTMP_RESULT_SUCCESS;
	int8_t value;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);

	if(shtmp_ic_use_flg == 1 && shtmp_ic_disable_flg == 0)
	{
		if(shtmp_i2c_read_reg((uint8_t*)(&value), 1, TMP103_TEMP_REG) != SHTMP_RESULT_SUCCESS)
		{
			value = SHTMP_IC_DISABLE_TEMP;
			shtmp_ic_disable_flg = 1;
			result = SHTMP_RESULT_FAIL;
			SHTMP_TRACE("[M] %s : Read Fail(%d)\n",__FUNCTION__,value);
		}

		if(shtmp_resume_flg)
		{
			if(abs(value - shtmp_prev_temp) > SHTMP_RESUME_THRESH)
			{
				value = shtmp_prev_temp;
			}
			shtmp_resume_flg = 0;
		}

		shtmp_prev_temp = value;
	}
	else if(shtmp_ic_use_flg == 0)
	{
		value = SHTMP_NO_IC_TEMP;
		SHTMP_TRACE("[M] %s : No IC\n",__FUNCTION__);
	}
	else if(shtmp_ic_disable_flg == 1)
	{
		value = SHTMP_IC_DISABLE_TEMP;
		result = SHTMP_RESULT_FAIL;
		SHTMP_TRACE("[M] %s : IC DISABLE\n",__FUNCTION__);
	}
	else
	{
		value = SHTMP_IC_DISABLE_TEMP;
		result = SHTMP_RESULT_FAIL;
		SHTMP_TRACE("[M] %s : Consider IC DISABLE\n",__FUNCTION__);
	}

#ifdef SHTMP_ENABLE_DEVELOP_BUILD
	if(debug_tmp != 0)
	{
		SHTMP_TRACE("[M] %s : debug, return dummy temprature\n",__FUNCTION__);
		result = SHTMP_RESULT_SUCCESS;
		value = (int8_t)debug_tmp;
	}
#endif /* SHTMP_ENABLE_DEVELOP_BUILD */

	*temp_p = value;
	SHTMP_INFO("%s : value=%d, result=%d\n", __FUNCTION__, (int)value, (int)result);
	SHTMP_TRACE("[E] %s \n",__FUNCTION__);

	return SHTMP_RESULT_SUCCESS;
};

shtmp_result_t shtmp_set_configure( uint8_t value )
{
	shtmp_result_t result = SHTMP_RESULT_SUCCESS;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_INFO("%s : value=%d \n", __FUNCTION__, (int)value);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);

	if(shtmp_ic_use_flg == 1 && shtmp_ic_disable_flg == 0)
	{
		if(shtmp_i2c_write_reg(&value, 1, TMP103_CONF_REG) != SHTMP_RESULT_SUCCESS)
		{
			shtmp_ic_disable_flg = 1;
			result = SHTMP_RESULT_FAIL;
			SHTMP_TRACE("[M] %s : Write Fail\n",__FUNCTION__);
		}
	}
	else if(shtmp_ic_disable_flg == 1)
	{
		result = SHTMP_RESULT_FAIL;
		SHTMP_TRACE("[M] %s : IC DISABLE\n",__FUNCTION__);
	}

	SHTMP_INFO("%s : result=%d\n", __FUNCTION__, (int)result);
	SHTMP_TRACE("[E] %s \n",__FUNCTION__);

	return result;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ LOCAL FUNCTION'S CODE AREA :                                              |*/
/*+-----------------------------------------------------------------------------+*/

/* check HW revision and HW type */
static shtmp_result_t shtmp_check_HW_type_revision(void)
{
	int is_db;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);

	is_db = GPIO_GET(DBGB_SW_BIT);
	SHTMP_TRACE("[P] %s is_db=%d \n",__FUNCTION__, is_db);
	if(is_db == 0)
	{
		/* Debug Board */
		shtmp_ic_use_flg = 0;
		shtmp_ic_disable_flg = 0;
	}
	else
	{
		/* Product */
		shtmp_ic_use_flg = 1;
		shtmp_ic_disable_flg = 0;
	}

	SHTMP_TRACE("[E] %s \n",__FUNCTION__);

	return SHTMP_RESULT_SUCCESS;
}

static shtmp_result_t shtmp_i2c_read_reg( uint8_t *value, uint8_t len, uint8_t reg )
{
	struct i2c_msg msg[2];

	int retry;
	uint8_t data = reg;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);
	SHTMP_INFO("%s : value=%p, len=%d, reg=%d\n",__FUNCTION__, value, (int)len, (int)reg);

	mutex_lock(&shtmp_i2c_lock);

	msg[0].addr  = shtmp_i2c_p->clt_p->addr;
	msg[0].flags = 0;		/* I2C_M_WR */
	msg[0].len   = 1;
	msg[0].buf   = &data;

	msg[1].addr  = shtmp_i2c_p->clt_p->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len   = len;
	msg[1].buf   = value;

	for(retry = 0; retry <= 10; retry++)
	{
		if(i2c_transfer(shtmp_i2c_p->clt_p->adapter, msg, 2) > 0)
		{
			SHTMP_TRACE("[E] %s : Success(%d)\n",__FUNCTION__, *value);
			mutex_unlock(&shtmp_i2c_lock);
			return SHTMP_RESULT_SUCCESS;
		}
		udelay(1000);
	}

	mutex_unlock(&shtmp_i2c_lock);

	printk(KERN_DEBUG "[SHTMP] i2c read error");
	SHTMP_TRACE("[E] %s : Fail\n",__FUNCTION__);

	return SHTMP_RESULT_FAIL;
}

static shtmp_result_t shtmp_i2c_write_reg( uint8_t *value, uint8_t len, uint8_t reg)
{
	struct i2c_msg msg;
	uint8_t tdata[2];
	uint8_t retry = 0;

	SHTMP_TRACE("[S] %s : %d\n",__FUNCTION__, *value);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);
	SHTMP_INFO("%s : value=%p, len=%d, reg=%d\n",__FUNCTION__, value, (int)len, (int)reg);

	mutex_lock(&shtmp_i2c_lock);

	tdata[0] = reg;
	tdata[1] = (*value);

	msg.addr  = shtmp_i2c_p->clt_p->addr;
	msg.flags = 0;			/* I2C_M_WR */
	msg.len   = len;
	msg.buf   = tdata;

	for(retry = 0; retry < 10; retry++)
	{
		if(i2c_transfer(shtmp_i2c_p->clt_p->adapter, &msg, 1) > 0)
		{
			SHTMP_TRACE("[E] %s : Success\n",__FUNCTION__);
			mutex_unlock(&shtmp_i2c_lock);
			return SHTMP_RESULT_SUCCESS;
		}
		udelay(1000);
	}

	mutex_unlock(&shtmp_i2c_lock);

	printk(KERN_DEBUG "[SHTMP] i2c write error");
	SHTMP_TRACE("[E] %s : Fail\n",__FUNCTION__);

	return SHTMP_RESULT_FAIL;
}

/*+-----------------------------------------------------------------------------+*/
/*| @ PLATFORM DRIVER MODULE CODE AREA :                                        |*/
/*+-----------------------------------------------------------------------------+*/

/* device attribute READ */
static ssize_t show_device_tmp(struct device* dev_p, struct device_attribute* attr_p, char* buf_p)
{
	ssize_t size = -1;
	int8_t value;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);

	if(shtmp_ic_use_flg == 1 && shtmp_ic_disable_flg == 0)
	{
		if(shtmp_i2c_read_reg((uint8_t*)(&value), 1, TMP103_TEMP_REG) != SHTMP_RESULT_SUCCESS){
			value = SHTMP_IC_DISABLE_TEMP;
			shtmp_ic_disable_flg = 1;
			SHTMP_TRACE("[M] %s : Read Fail(%d)\n",__FUNCTION__,value);
		}

		if(shtmp_resume_flg)
		{
			if(abs(value - shtmp_prev_temp) > SHTMP_RESUME_THRESH)
			{
				value = shtmp_prev_temp;
			}
			shtmp_resume_flg = 0;
		}

		shtmp_prev_temp = value;
	}
	else if(shtmp_ic_use_flg == 0)
	{
		value = SHTMP_NO_IC_TEMP;
		SHTMP_TRACE("[M] %s : No IC\n",__FUNCTION__);
	}
	else if(shtmp_ic_disable_flg == 1)
	{
		value = SHTMP_IC_DISABLE_TEMP;
		SHTMP_TRACE("[M] %s : IC DISABLE\n",__FUNCTION__);
	}
	else
	{
		value = SHTMP_IC_DISABLE_TEMP;
		SHTMP_TRACE("[M] %s : Consider IC DISABLE\n",__FUNCTION__);
	}
#ifdef SHTMP_ENABLE_DEVELOP_BUILD
	if(debug_tmp != 0)
	{
		value = (int8_t)debug_tmp;
	}
#endif /* SHTMP_ENABLE_DEVELOP_BUILD */

	size = sprintf(buf_p, "%d\n", value);

	SHTMP_INFO("%s : value=%d, size=%d\n",__FUNCTION__, value, size);
	SHTMP_TRACE("[E] %s \n",__FUNCTION__);

	return size;
}

/* device attribute WRITE */
static ssize_t store_device_tmp(struct device* dev_p, struct device_attribute* attr_p, const char* buf_p, size_t size)
{
	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_TRACE("[E] %s \n",__FUNCTION__);
	return size;
}

/* define device attribute */
static DEVICE_ATTR(temperature, S_IRUGO | S_IWUSR, show_device_tmp, store_device_tmp);

static struct attribute *shtmp_attrs[] =
{
	&dev_attr_temperature.attr,
	NULL,
};

static struct attribute_group shtmp_attr_group =
{
	.attrs = shtmp_attrs,
};

/* driver probe event */
static int shtmp_probe(struct platform_device* pdev)
{
	int ret;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);

	ret = sysfs_create_group(&pdev->dev.kobj, &shtmp_attr_group);
	if(ret)
	{
		SHTMP_ERROR(" %s : sysfs_create_group() fail, %d.\n",__FUNCTION__, ret);
	}

	SHTMP_TRACE("[E] %s \n",__FUNCTION__);
	return ret;
}

/* driver remove event */
static int shtmp_remove(struct platform_device* pdev)
{
	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_TRACE("[E] %s \n",__FUNCTION__);
	return 0;
}

/* i2c probe event */
static int shtmp_i2c_probe(struct i2c_client* clt_p, const struct i2c_device_id* id_p)
{
	int ret = 0;

	shtmp_i2c_data_t* i2c_p;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);

	// log info shtmp_i2c
	if(clt_p->name)
	{
		SHTMP_INFO("%s : name=\"%s\"\n",__FUNCTION__, clt_p->name);
	}
	SHTMP_INFO("%s : flags=0x%04x, addr=0x%04x\n",__FUNCTION__, (int)clt_p->flags, (int)clt_p->addr);
	SHTMP_INFO("%s : adapter=%p, driver=%p, irq=0x%08x\n",__FUNCTION__, (void*)clt_p->adapter, (void*) clt_p->driver,clt_p->irq);

	mutex_init(&shtmp_i2c_lock);

	if(shtmp_i2c_p != NULL)
	{
		ret = -EPERM;
		SHTMP_ERROR(" %s : has already been initialized.\n",__FUNCTION__);
		goto i2c_probe_exit;
	}

	i2c_p = (shtmp_i2c_data_t*)kzalloc(sizeof(shtmp_i2c_data_t),GFP_KERNEL);
	if(i2c_p == NULL)
	{
		ret = -ENOMEM;
		SHTMP_ERROR(" %s : memory allocation failed.\n",__FUNCTION__);
		goto i2c_probe_exit;
	}

	shtmp_i2c_p = i2c_p;

	i2c_set_clientdata(clt_p,i2c_p);

	i2c_p->clt_p = clt_p;

	SHTMP_TRACE("[M] %s : init finished\n",__FUNCTION__);

	// check HW Revision
	shtmp_check_HW_type_revision();

	// init configuration register
	if(shtmp_ic_use_flg == 1 && shtmp_ic_disable_flg == 0)
	{
		uint8_t config = TMP103_CONF_INIT;
		if(shtmp_i2c_write_reg(&config, 1, TMP103_CONF_REG) != SHTMP_RESULT_SUCCESS)
		{
			shtmp_ic_disable_flg = 1;
		}
	}

	SHTMP_TRACE("[E] %s : ic_use_flg(%d)\n",__FUNCTION__, shtmp_ic_use_flg);
	SHTMP_TRACE("[E] %s : ic_disable_flg(%d)\n",__FUNCTION__, shtmp_ic_disable_flg);

	return ret;

i2c_probe_exit:

	return ret;
}

/* i2c remove event */
static int __devexit shtmp_i2c_remove(struct i2c_client* clt_p)
{
	shtmp_i2c_data_t* i2c_p;

	SHTMP_TRACE("[S] %s \n",__FUNCTION__);

	i2c_p = i2c_get_clientdata(clt_p);

	SHTMP_TRACE("[E] %s \n",__FUNCTION__);

	kfree(i2c_p);

	return 0;
}

/* i2c suspend event */
static int shtmp_i2c_suspend(struct platform_device * dev_p, pm_message_t state)
{
	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);

	if(shtmp_ic_use_flg == 1)
	{
		if(shtmp_ic_disable_flg == 0)
		{
			uint8_t config;

			config = TMP103_CONF_INIT;
			config &= ~TMP103_CONF_SD;

			if(shtmp_i2c_write_reg(&config, 1, TMP103_CONF_REG) != SHTMP_RESULT_SUCCESS)
			{
				SHTMP_TRACE("[E] %s : i2c_write_reg fail\n",__FUNCTION__);
				shtmp_ic_disable_flg = 1;
			}

			SHTMP_TRACE("[E] %s \n",__FUNCTION__);
			return 0;
		}
		else
		{
			SHTMP_TRACE("[E] %s : IC DISABLE\n",__FUNCTION__);
			return 0;
		}
	}
	else
	{
		SHTMP_TRACE("[E] %s : NO IC\n",__FUNCTION__);
		return 0;
	}
}

/* i2c resume event */
static int shtmp_i2c_resume(struct platform_device * dev_p)
{
	SHTMP_TRACE("[S] %s \n",__FUNCTION__);
	SHTMP_INFO("%s : use=%d, disable=%d\n",__FUNCTION__, shtmp_ic_use_flg, shtmp_ic_disable_flg);

	if(shtmp_ic_use_flg == 1)
	{
		if(shtmp_ic_disable_flg == 0)
		{
			uint8_t config;

			config = TMP103_CONF_INIT;
			config |= (TMP103_CONF_M1 | TMP103_CONF_CR0);
			config &= ~TMP103_CONF_M0;

			if(shtmp_i2c_write_reg(&config, 1, TMP103_CONF_REG) != SHTMP_RESULT_SUCCESS)
			{
				SHTMP_TRACE("[E] %s : i2c_write_reg fail\n",__FUNCTION__);
				shtmp_ic_disable_flg = 1;
			}

			shtmp_resume_flg = 1;

			SHTMP_TRACE("[E] %s \n",__FUNCTION__);
			return 0;
		}
		else
		{
			SHTMP_TRACE("[E] %s : IC DISABLE\n",__FUNCTION__);
			return 0;
		}
	}
	else
	{
		SHTMP_TRACE("[E] %s : NO IC\n",__FUNCTION__);
		return 0;
	}
}

static const struct i2c_device_id shtmp_i2c_id[] =
{
	{ "shtmp_i2c", 0 },
	{ }
};

/* define i2c driver */
static struct i2c_driver shtmp_i2c_driver =
{
	.probe		= shtmp_i2c_probe,
	.remove		= __devexit_p(shtmp_i2c_remove),
	.driver		= {
		.name = "shtmp_i2c",
	},
	.id_table	= shtmp_i2c_id,
};

/* define device */
static struct platform_device shtmp_device =
{
	.name = "shtmp",
	.id   = -1,
};

/* define driver */
static struct platform_driver shtmp_driver =
{
	.probe	      = shtmp_probe,
	.remove	      = shtmp_remove,
	.driver	      = {
		.name = "shtmp",
	},
	.resume       = shtmp_i2c_resume,
	.suspend      = shtmp_i2c_suspend,
};

static int __init shtmp_init(void)
{
	SHTMP_TRACE("[S] %s \n",__FUNCTION__);

	i2c_add_driver(&shtmp_i2c_driver);

	platform_device_register(&shtmp_device);

	platform_driver_register(&shtmp_driver);

	SHTMP_TRACE("[E] %s \n",__FUNCTION__);
	return 0;
}

static void __exit shtmp_exit(void)
{
	SHTMP_TRACE("[S] %s \n",__FUNCTION__);

	platform_driver_unregister(&shtmp_driver);

	platform_device_unregister(&shtmp_device);

	i2c_del_driver(&shtmp_i2c_driver);

	SHTMP_TRACE("[E] %s \n",__FUNCTION__);
}

module_init(shtmp_init);
module_exit(shtmp_exit);

MODULE_DESCRIPTION("SH Battery Driver Tmp Sensor");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/*+-----------------------------------------------------------------------------+*/
/*| @ THIS FILE END :                                                           |*/
/*+-----------------------------------------------------------------------------+*/
