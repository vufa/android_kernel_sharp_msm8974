/* Copyright (c) 2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "msm_sensor.h"
#include "msm_camera_io_util.h"
#include <linux/io.h>
#include <mach/msm_iomap.h>
#include <mach/perflock.h>

#define IMX132_SENSOR_NAME "imx132"
DEFINE_MSM_MUTEX(imx132_mut);

/* #define CONFIG_MSMB_CAMERA_DEBUG */
#undef CDBG
#ifdef CONFIG_MSMB_CAMERA_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) do { } while (0)
#endif

static struct msm_sensor_ctrl_t imx132_s_ctrl;

static struct perf_lock imx132_perf_lock;

static struct msm_sensor_power_setting imx132_power_setting[] = {
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VDIG,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VANA,
		.config_val = 0,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_VREG,
		.seq_val = CAM_VIO,
		.config_val = 0,
		.delay = 5,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_LOW,
		.delay = 0,
	},
	{
		.seq_type = SENSOR_GPIO,
		.seq_val = SENSOR_GPIO_RESET,
		.config_val = GPIO_OUT_HIGH,
		.delay = 0,
	},
#if defined(CONFIG_MACH_LYNX_DL40)
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 6590000,
		.delay = 5,
	},
#else
	{
		.seq_type = SENSOR_CLK,
		.seq_val = SENSOR_CAM_MCLK,
		.config_val = 6310000,
		.delay = 5,
	},
#endif
	{
		.seq_type = SENSOR_I2C_MUX,
		.seq_val = 0,
		.config_val = 0,
		.delay = 0,
	},
};

static struct msm_cam_clk_info imx132_clk_info[] = {
	[SENSOR_CAM_MCLK] = {"cam_gp1_src_clk", 6590000},
	[SENSOR_CAM_CLK] = {"cam_gp1_clk", 0},
};

static struct v4l2_subdev_info imx132_subdev_info[] = {
	{
		.code = V4L2_MBUS_FMT_SBGGR10_1X10,
		.colorspace = V4L2_COLORSPACE_JPEG,
		.fmt = 1,
		.order = 0,
	},
};

int32_t imx132_sensor_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_sensor_ctrl_t *s_ctrl;
	CDBG("%s", __func__);
	
	rc = msm_sensor_i2c_probe(client, id, &imx132_s_ctrl);
	if (rc < 0) {
		CDBG("%s msm_sensor_i2c_probe failed\n", __func__);
		return rc;
	}
	
	s_ctrl = (struct msm_sensor_ctrl_t *)(id->driver_data);
	if (!s_ctrl) {
		pr_err("%s:%d sensor ctrl structure NULL\n", __func__,
			__LINE__);
		return -EINVAL;
	}
	
	s_ctrl->clk_info = imx132_clk_info;
	s_ctrl->clk_info_size = ARRAY_SIZE(imx132_clk_info);
	
	return rc;
}

static const struct i2c_device_id imx132_i2c_id[] = {
	{IMX132_SENSOR_NAME, (kernel_ulong_t)&imx132_s_ctrl},
	{ }
};

static struct i2c_driver imx132_i2c_driver = {
	.id_table = imx132_i2c_id,
	.probe  = imx132_sensor_i2c_probe,
	.driver = {
		.name = IMX132_SENSOR_NAME,
	},
};

static struct msm_camera_i2c_client imx132_sensor_i2c_client = {
	.addr_type = MSM_CAMERA_I2C_WORD_ADDR,
};

static const struct of_device_id imx132_dt_match[] = {
	{.compatible = "qcom,imx132", .data = &imx132_s_ctrl},
	{}
};

MODULE_DEVICE_TABLE(of, imx132_dt_match);

static struct platform_driver imx132_platform_driver = {
	.driver = {
		.name = "qcom,imx132",
		.owner = THIS_MODULE,
		.of_match_table = imx132_dt_match,
	},
};

static int32_t imx132_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	const struct of_device_id *match;
	match = of_match_device(imx132_dt_match, &pdev->dev);
	rc = msm_sensor_platform_probe(pdev, match->data);
	return rc;
}

static int __init imx132_init_module(void)
{
	int32_t rc = 0;
	pr_info("%s:%d\n", __func__, __LINE__);
	perf_lock_init(&imx132_perf_lock, PERF_LOCK_729600KHz, "camera_imx132");
	rc = platform_driver_probe(&imx132_platform_driver,
		imx132_platform_probe);
	if (!rc)
		return rc;
	pr_err("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&imx132_i2c_driver);
}

static void __exit imx132_exit_module(void)
{
	pr_info("%s:%d\n", __func__, __LINE__);
	if (imx132_s_ctrl.pdev) {
		msm_sensor_free_sensor_data(&imx132_s_ctrl);
		platform_driver_unregister(&imx132_platform_driver);
	} else
		i2c_del_driver(&imx132_i2c_driver);
	return;
}

int32_t imx132_i2c_read(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	void *data;
	
	CDBG("%s start\n", __func__);
	
	data = kmalloc(cdata->cfg.i2c_info.length, GFP_KERNEL);
	if(data == NULL){
		pr_err("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}
	
	CDBG("%s i2c_info.addr = 0x%0x\n", __func__, cdata->cfg.i2c_info.addr);
	CDBG("%s i2c_info.length = 0x%0x\n", __func__, cdata->cfg.i2c_info.length);
	rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read_seq(s_ctrl->sensor_i2c_client, cdata->cfg.i2c_info.addr, data, cdata->cfg.i2c_info.length);
	if(rc < 0){
		pr_err("%s i2c_read_seq failed\n", __func__);
		kfree(data);
		return -EFAULT;
	}
	
	if (copy_to_user((void *)cdata->cfg.i2c_info.data,
		data,
		cdata->cfg.i2c_info.length)){
		kfree(data);
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	kfree(data);
	
	return rc;
}

int32_t imx132_i2c_write(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	struct msm_camera_i2c_reg_setting conf_array;
	struct msm_camera_i2c_reg_array *reg_setting = NULL;
	struct msm_camera_i2c_reg_array *pos_reg_setting = NULL;
	struct msm_camera_i2c_seq_reg_setting conf_sec_array;
	struct msm_camera_i2c_seq_reg_array *reg_sec_setting = NULL;
	struct msm_camera_i2c_seq_reg_array *pos_reg_sec_setting = NULL;
	int i;
	int cur_size, array_size;

	if (copy_from_user(&conf_array,
		(void *)cdata->cfg.setting,
		sizeof(struct msm_camera_i2c_reg_setting))) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}

	reg_setting = kzalloc(conf_array.size *
		(sizeof(struct msm_camera_i2c_reg_array)), GFP_KERNEL);
	if (!reg_setting) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}
	reg_sec_setting = kzalloc(conf_array.size *
		(sizeof(struct msm_camera_i2c_seq_reg_array)),
		GFP_KERNEL);
	if (!reg_sec_setting) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		return -EFAULT;
	}

	if (copy_from_user(reg_setting, (void *)conf_array.reg_setting,
		conf_array.size *
		sizeof(struct msm_camera_i2c_reg_array))) {
		pr_err("%s:%d failed\n", __func__, __LINE__);
		kfree(reg_setting);
		kfree(reg_sec_setting);
		return -EFAULT;
	}
	
	if(conf_array.data_type == MSM_CAMERA_I2C_BYTE_DATA){
	
		pos_reg_setting = reg_setting;
		pos_reg_sec_setting = reg_sec_setting;
		
		array_size = 1;
		conf_sec_array.addr_type = conf_array.addr_type;
		conf_sec_array.delay = conf_array.delay;
		conf_sec_array.reg_setting = reg_sec_setting;
		
		pos_reg_sec_setting->reg_addr = pos_reg_setting->reg_addr;
		pos_reg_sec_setting->reg_data[0] = pos_reg_setting->reg_data;
		cur_size = 1;
		pos_reg_setting++;
		
		for (i = 1; i < conf_array.size; i++) {
			if(pos_reg_setting->reg_addr == (pos_reg_sec_setting->reg_addr + cur_size)){
				if(cur_size >= 7){
					pos_reg_sec_setting->reg_data_size = cur_size;
					
					pos_reg_sec_setting++;
					array_size++;
					
					cur_size = 0;
					pos_reg_sec_setting->reg_addr = pos_reg_setting->reg_addr;
				}
				pos_reg_sec_setting->reg_data[cur_size] = pos_reg_setting->reg_data;
				cur_size++;
			} else {
				pos_reg_sec_setting->reg_data_size = cur_size;
				
				pos_reg_sec_setting++;
				array_size++;
				
				pos_reg_sec_setting->reg_addr = pos_reg_setting->reg_addr;
				pos_reg_sec_setting->reg_data[0] = pos_reg_setting->reg_data;
				cur_size = 1;
			}
			pos_reg_setting++;
		}

		CDBG("%s array_size = %d", __func__, array_size);
		pos_reg_sec_setting->reg_data_size = cur_size;

		conf_sec_array.size = array_size;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->
			i2c_write_seq_table(s_ctrl->sensor_i2c_client,
			&conf_sec_array);
		
		CDBG("%s rc = %d", __func__, (int)rc);

	} else {

		CDBG("%s conf_array.data_type = %d", __func__, conf_array.data_type);
		
		conf_array.reg_setting = reg_setting;
		rc = s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write_table(
			s_ctrl->sensor_i2c_client, &conf_array);
			
		CDBG("%s rc = %d", __func__, (int)rc);
	}
	
	kfree(reg_setting);
	kfree(reg_sec_setting);

	CDBG("%s end", __func__);
	
	return rc;
}

int32_t imx132_sensor_config(struct msm_sensor_ctrl_t *s_ctrl,
	void __user *argp)
{
	struct sensorb_cfg_data *cdata = (struct sensorb_cfg_data *)argp;
	long rc = 0;
	mutex_lock(s_ctrl->msm_sensor_mutex);
	CDBG("%s:%d %s cfgtype = %d\n", __func__, __LINE__,
		s_ctrl->sensordata->sensor_name, cdata->cfgtype);
	switch (cdata->cfgtype) {
		case SHCFG_GET_I2C_DATA:
			rc = imx132_i2c_read(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s imx132_i2c_read failed", __func__);
			}
			break;
		case CFG_WRITE_I2C_ARRAY:
			rc = imx132_i2c_write(s_ctrl, argp);
			if(rc < 0){
				pr_err("%s imx081_i2c_read failed", __func__);
			}
			break;
		default:
			mutex_unlock(s_ctrl->msm_sensor_mutex);
			rc = msm_sensor_config(s_ctrl, argp);
			return rc;
			break;
	}

	mutex_unlock(s_ctrl->msm_sensor_mutex);

	return rc;
}

int32_t imx132_sensor_power_up(struct msm_sensor_ctrl_t *s_ctrl)
{
	int32_t rc = 0;

	CDBG("%s\n", __func__);
	
	CDBG("%s MSM_TLMM_BASE + 0x2024 = 0x%0x\n", __func__, (unsigned int)MSM_TLMM_BASE + 0x2024);
	CDBG("%s __raw_readl = 0x%0x\n", __func__, __raw_readl((MSM_TLMM_BASE + 0x2024)));
	
	__raw_writel(__raw_readl((MSM_TLMM_BASE + 0x2024)) | 0x02, (MSM_TLMM_BASE + 0x2024));
	
	s_ctrl->clk_info = imx132_clk_info;
	s_ctrl->clk_info_size = ARRAY_SIZE(imx132_clk_info);
	
	rc = msm_sensor_power_up(s_ctrl);
	if (rc < 0) {
		pr_err("%s: msm_sensor_power_up failed\n",
			__func__);
		return 0;
	}

	perf_lock(&imx132_perf_lock);
	return rc;
}

int32_t imx132_sensor_power_down(struct msm_sensor_ctrl_t *s_ctrl)
{
	long fps = 0;
	uint16_t ll_pclk;
	uint16_t fl_lines;
	uint32_t delay = 0;
	
	CDBG("%s\n", __func__);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_write(
			s_ctrl->sensor_i2c_client,
			0x0100,
			0x00, MSM_CAMERA_I2C_BYTE_DATA);
	
	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x342,
			&ll_pclk, MSM_CAMERA_I2C_WORD_DATA);

	s_ctrl->sensor_i2c_client->i2c_func_tbl->i2c_read(
			s_ctrl->sensor_i2c_client,
			0x340,
			&fl_lines, MSM_CAMERA_I2C_WORD_DATA);
	
	CDBG("%s ll_pclk = %d, frame fl_lines = %d\n", __func__, ll_pclk, fl_lines);

	if((ll_pclk != 0) && (fl_lines != 0)){
#if defined(CONFIG_MACH_LYNX_DL40)
		fps = 151648352 /
			fl_lines / ll_pclk;
#else
		fps = 142736842 /
			fl_lines / ll_pclk;
#endif
		if(fps != 0)
			delay = 1000 / fps;
	}
	CDBG("%s fps = %ld, frame time = %d\n", __func__, fps, delay);
	msleep(delay);
	
	msm_sensor_power_down(s_ctrl);
	
	CDBG("%s MSM_TLMM_BASE + 0x2024 = 0x%0x\n", __func__, (unsigned int)MSM_TLMM_BASE + 0x2024);
	CDBG("%s __raw_readl = 0x%0x\n", __func__, __raw_readl(MSM_TLMM_BASE + 0x2024));
	
	__raw_writel(__raw_readl((MSM_TLMM_BASE + 0x2024)) & ~(0x02), (MSM_TLMM_BASE + 0x2024));

	perf_unlock(&imx132_perf_lock);

	return 0;
}

static struct msm_sensor_fn_t imx132_sensor_func_tbl = {
	.sensor_config = imx132_sensor_config,
	.sensor_power_up = imx132_sensor_power_up,
	.sensor_power_down = imx132_sensor_power_down,
	.sensor_match_id = msm_sensor_match_id,
};

static struct msm_sensor_ctrl_t imx132_s_ctrl = {
	.sensor_i2c_client = &imx132_sensor_i2c_client,
	.power_setting_array.power_setting = imx132_power_setting,
	.power_setting_array.size = ARRAY_SIZE(imx132_power_setting),
	.msm_sensor_mutex = &imx132_mut,
	.sensor_v4l2_subdev_info = imx132_subdev_info,
	.sensor_v4l2_subdev_info_size = ARRAY_SIZE(imx132_subdev_info),
	.func_tbl = &imx132_sensor_func_tbl,
};

module_init(imx132_init_module);
module_exit(imx132_exit_module);
MODULE_DESCRIPTION("imx132");
MODULE_LICENSE("GPL v2");
