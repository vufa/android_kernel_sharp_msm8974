/* Copyright (c) 2011-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/module.h>
#include "msm_sd.h"
#include "msm_actuator.h"
#include "msm_cci.h"

DEFINE_MSM_MUTEX(ak7345_act_mutex);

//#define MSM_ACUTUATOR_DEBUG
#undef CDBG
#ifdef MSM_ACUTUATOR_DEBUG
#define CDBG(fmt, args...) pr_err(fmt, ##args)
#else
#define CDBG(fmt, args...) pr_debug(fmt, ##args)
#endif

#undef CDBG_DEB
//#define CDBG_DEB(fmt, args...) pr_err(fmt, ##args)
//#define CDBG_DEB(fmt, args...) pr_debug(fmt, ##args)
#define CDBG_DEB(fmt, args...)

//#define ACTUATOR_DEBUG

static struct msm_actuator ak7345_act_table;

static struct i2c_driver ak7345_act_i2c_driver;
static struct msm_actuator *actuators[] = {
	&ak7345_act_table,
};

static void ak7345_act_parse_i2c_params(struct msm_actuator_ctrl_t *a_ctrl,
	int16_t next_lens_position, uint32_t hw_params, uint16_t delay)
{
	struct msm_actuator_reg_params_t *write_arr = a_ctrl->reg_tbl;
	uint32_t hw_dword = hw_params;
	uint16_t i2c_byte1 = 0, i2c_byte2 = 0;
	uint16_t value = 0;
	uint32_t size = a_ctrl->reg_tbl_size, i = 0;
	struct msm_camera_i2c_reg_array *i2c_tbl = a_ctrl->i2c_reg_tbl;
	CDBG("Enter\n");

	if( a_ctrl->i2c_reg_tbl == NULL )
		return;
	
	for (i = 0; i < size; i++) {
		CDBG("next_lens_position =%d\n", next_lens_position );
		CDBG("data_shift =%d\n",write_arr[i].data_shift );
		CDBG("hw_mask    =%d\n",write_arr[i].hw_mask );
		CDBG("hw_shift   =%d\n",write_arr[i].hw_shift);
		CDBG("reg_addr   =%x\n",write_arr[i].reg_addr );
		CDBG("hw_params  =%x\n",hw_params );
		
		if (write_arr[i].reg_write_type == MSM_ACTUATOR_WRITE_DAC) {
			value = (next_lens_position <<write_arr[i].data_shift) |((hw_dword & write_arr[i].hw_mask) >>write_arr[i].hw_shift);
			if (write_arr[i].reg_addr != 0xFFFF) {
				i2c_byte1 = write_arr[i].reg_addr;
				i2c_byte2 = value;
#if 0 /* send word */
				if (size != (i+1)) {
					i2c_byte2 = value & 0xFF;
					CDBG("byte1:0x%x, byte2:0x%x\n",i2c_byte1, i2c_byte2);
					i2c_tbl[a_ctrl->i2c_tbl_index].reg_addr = i2c_byte1;
					i2c_tbl[a_ctrl->i2c_tbl_index].reg_data = i2c_byte2;
					i2c_tbl[a_ctrl->i2c_tbl_index].delay    = 0;
					a_ctrl->i2c_tbl_index++;
					i++;
					i2c_byte1 = write_arr[i].reg_addr;
					i2c_byte2 = (value & 0xFF00) >> 8;
				}
#endif
			} else {
				i2c_byte1 = (value & 0xFF00) >> 8;
				i2c_byte2 = value & 0xFF;
			}
		} else {
			i2c_byte1 = write_arr[i].reg_addr;
			i2c_byte2 = (hw_dword & write_arr[i].hw_mask) >>write_arr[i].hw_shift;
		}
		CDBG_DEB("i2c_byte1:0x%x, i2c_byte2:0x%x\n", i2c_byte1, i2c_byte2);
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_addr = i2c_byte1;
		i2c_tbl[a_ctrl->i2c_tbl_index].reg_data = i2c_byte2;
		i2c_tbl[a_ctrl->i2c_tbl_index].delay    = delay;
		a_ctrl->i2c_tbl_index++;
	}
	CDBG("Exit\n");
}

static int32_t ak7345_act_init_focus(struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t size, enum msm_actuator_data_type type,
	struct reg_settings_t *settings)
{
	int32_t rc = -EFAULT;
	int32_t i = 0;
	CDBG("Enter\n");
	
	/* 10msec delay */
	usleep_range(10000, 11000);
	
	for (i = 0; i < size; i++) {
		switch (type) {
		case MSM_ACTUATOR_BYTE_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, MSM_CAMERA_I2C_BYTE_DATA);
			break;
		case MSM_ACTUATOR_WORD_DATA:
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
				&a_ctrl->i2c_client,
				settings[i].reg_addr,
				settings[i].reg_data, MSM_CAMERA_I2C_WORD_DATA);
			break;
		default:
			pr_err("Unsupport data type: %d\n", type);
			break;
		}
		if (rc < 0)
			break;
	}

	a_ctrl->curr_step_pos = 0;
	CDBG("Exit\n");
	return rc;
}

static void ak7345_act_write_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	uint16_t curr_lens_pos,
	struct damping_params_t *damping_params,
	int8_t sign_direction,
	int16_t next_lens_pos)
{
	CDBG("Enter\n");
	
	if (curr_lens_pos != next_lens_pos) {
		CDBG("ak7345_act_write_focus2 %d %d %d \n", curr_lens_pos, next_lens_pos, damping_params->hw_params );
		
		a_ctrl->func_tbl->actuator_parse_i2c_params(a_ctrl,
			next_lens_pos, damping_params->hw_params, 0);
	}
	CDBG("Exit\n");
}

static int32_t ak7345_act_move_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	int8_t sign_dir = move_params->sign_dir;
	uint16_t step_boundary = 0;
	uint16_t target_step_pos = 0;
	uint16_t target_lens_pos = 0;
	int16_t dest_step_pos = move_params->dest_step_pos;
	uint16_t curr_lens_pos = 0;
	int dir = move_params->dir;
	int32_t num_steps = move_params->num_steps;
	struct msm_camera_i2c_reg_setting reg_setting;

	CDBG("called, dir %d, num_steps %d\n", dir, num_steps);

	if (dest_step_pos == a_ctrl->curr_step_pos)
		return rc;

	if( a_ctrl->step_position_table == NULL )
		return -ENOMEM;
	
	curr_lens_pos = a_ctrl->step_position_table[a_ctrl->curr_step_pos];
	a_ctrl->i2c_tbl_index = 0;
	
	CDBG_DEB("curr_step_pos =%d dest_step_pos =%d curr_lens_pos=%d\n", a_ctrl->curr_step_pos, dest_step_pos, curr_lens_pos);

	while (a_ctrl->curr_step_pos != dest_step_pos) {
		step_boundary = a_ctrl->region_params[a_ctrl->curr_region_index].step_bound[dir];
		
		if ((dest_step_pos * sign_dir) <= (step_boundary * sign_dir)) {

			target_step_pos = dest_step_pos;
			target_lens_pos =
				a_ctrl->step_position_table[target_step_pos];
			
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&(move_params->
						ringing_params[a_ctrl->
						curr_region_index]),
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

		} else {
			target_step_pos = step_boundary;
			target_lens_pos = a_ctrl->step_position_table[target_step_pos];
			
			a_ctrl->func_tbl->actuator_write_focus(a_ctrl,
					curr_lens_pos,
					&(move_params->ringing_params[a_ctrl->
						curr_region_index]),
					sign_dir,
					target_lens_pos);
			curr_lens_pos = target_lens_pos;

			a_ctrl->curr_region_index += sign_dir;
		}
		a_ctrl->curr_step_pos = target_step_pos;
	}

	reg_setting.reg_setting = a_ctrl->i2c_reg_tbl;
//	reg_setting.data_type = a_ctrl->i2c_data_type;
	reg_setting.data_type = MSM_CAMERA_I2C_WORD_DATA;
	reg_setting.size = a_ctrl->i2c_tbl_index;
	rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write_table_w_microdelay(
		&a_ctrl->i2c_client, &reg_setting);
	if (rc < 0) {
		pr_err("i2c write error:%d\n", rc);
		return rc;
	}
	a_ctrl->i2c_tbl_index = 0;
	CDBG("Exit\n");

	return rc;
}

static int32_t ak7345_act_init_step_table(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info)
{
	int16_t code_per_step = 0;
	int16_t cur_code = 0;
	int16_t step_index = 0, region_index = 0;
	uint16_t step_boundary = 0;
	uint32_t max_code_size = 1;
	uint16_t data_size = set_info->actuator_params.data_size;
	CDBG("Enter\n");

	for (; data_size > 0; data_size--)
		max_code_size *= 2;

	if( a_ctrl->step_position_table ){
		kfree(a_ctrl->step_position_table);
		a_ctrl->step_position_table = NULL;
	}
	/* Fill step position table */
	a_ctrl->step_position_table =
		kmalloc(sizeof(uint16_t) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);

	if (a_ctrl->step_position_table == NULL)
		return -ENOMEM;

	cur_code = set_info->af_tuning_params.initial_code;
	a_ctrl->step_position_table[step_index++] = cur_code;
	
	CDBG_DEB("step table[%d] = %d\n", (step_index-1), a_ctrl->step_position_table[(step_index-1)]);
	CDBG("region_size =%d\n", a_ctrl->region_size );
	
	for (region_index = 0;
		region_index < a_ctrl->region_size;
		region_index++) {
		
		code_per_step =
			a_ctrl->region_params[region_index].code_per_step;
		step_boundary =
			a_ctrl->region_params[region_index].
			step_bound[MOVE_NEAR];
		
		CDBG("region_index =%d code_per_step =%d boudary=%d max_code_size=%d \n", region_index, code_per_step, step_boundary, max_code_size );
		
		for (; step_index <= step_boundary;
			step_index++) {
			cur_code += code_per_step;
			
			if (cur_code < max_code_size){
				CDBG_DEB("step table[%d] = %d\n", step_index, cur_code );
				a_ctrl->step_position_table[step_index] =
					cur_code;
				
			}else {
				for (; step_index <
					set_info->af_tuning_params.total_steps;
					step_index++){
						CDBG_DEB("step table[%d] = %d\n", step_index, max_code_size );
					a_ctrl->
						step_position_table[
						step_index] =
						max_code_size;
				}
			}
		}
	}
	CDBG("Exit\n");
	return 0;
}

static int32_t ak7345_act_set_default_focus(
	struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_move_params_t *move_params)
{
	int32_t rc = 0;
	CDBG("Enter\n");

	if (a_ctrl->curr_step_pos != 0)
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl, move_params);
	CDBG("Exit\n");
	return rc;
}

static int32_t ak7345_act_power_down(struct msm_actuator_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
#if 0
	if (a_ctrl->vcm_enable) {
		rc = gpio_direction_output(a_ctrl->vcm_pwd, 0);
		if (!rc)
			gpio_free(a_ctrl->vcm_pwd);
	}
#endif
	
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		if( a_ctrl->cci_inited ){
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
				&a_ctrl->i2c_client, MSM_CCI_RELEASE);
			if (rc < 0)
				pr_err("cci_release failed\n");
		}
	}
	if( a_ctrl->step_position_table ){
		kfree(a_ctrl->step_position_table);
	}
	if( a_ctrl->i2c_reg_tbl ){
		kfree(a_ctrl->i2c_reg_tbl);
	}
	a_ctrl->step_position_table = NULL;
	a_ctrl->i2c_reg_tbl = NULL;
	a_ctrl->i2c_tbl_index = 0;
	
	a_ctrl->num_instance = 0;
	a_ctrl->cci_inited   = 0;
	
	CDBG("Exit\n");
	return rc;
}

static int32_t ak7345_act_init(struct msm_actuator_ctrl_t *a_ctrl,
	struct msm_actuator_set_info_t *set_info) {
	struct reg_settings_t *init_settings = NULL;
	int32_t rc = -EFAULT;
	uint16_t i = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	CDBG("Enter\n");
	
	for (i = 0; i < ARRAY_SIZE(actuators); i++) {
		if (set_info->actuator_params.act_type ==
			actuators[i]->act_type) {
			a_ctrl->func_tbl = &actuators[i]->func_tbl;
			rc = 0;
		}
	}
	if (rc < 0) {
		pr_err("Actuator function table not found\n");
		return rc;
	}

	a_ctrl->region_size = set_info->af_tuning_params.region_size;
	if (a_ctrl->region_size > MAX_ACTUATOR_REGION) {
		pr_err("MAX_ACTUATOR_REGION is exceeded.\n");
		return -EFAULT;
	}
	a_ctrl->pwd_step = set_info->af_tuning_params.pwd_step;
	a_ctrl->total_steps = set_info->af_tuning_params.total_steps;

	if (copy_from_user(&a_ctrl->region_params,
		(void *)set_info->af_tuning_params.region_params,
		a_ctrl->region_size * sizeof(struct region_params_t)))
		return -EFAULT;

	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		
#if 1
		if( !a_ctrl->cci_inited ){
			rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
				&a_ctrl->i2c_client, MSM_CCI_INIT);
			if (rc < 0)
				pr_err("cci_init failed\n");
			
		a_ctrl->cci_inited = 1;
		}
#endif
		cci_client = a_ctrl->i2c_client.cci_client;
		cci_client->sid =
			set_info->actuator_params.i2c_addr >> 1;
		cci_client->retries = 3;
		cci_client->id_map = 0;
		cci_client->cci_i2c_master = a_ctrl->cci_master;
	} else {
		a_ctrl->i2c_client.client->addr =
			set_info->actuator_params.i2c_addr;
	}

	a_ctrl->i2c_data_type = set_info->actuator_params.i2c_data_type;
	a_ctrl->i2c_client.addr_type = set_info->actuator_params.i2c_addr_type;
	a_ctrl->reg_tbl_size = set_info->actuator_params.reg_tbl_size;
	if (a_ctrl->reg_tbl_size > MAX_ACTUATOR_REG_TBL_SIZE) {
		pr_err("MAX_ACTUATOR_REG_TBL_SIZE is exceeded.\n");
		return -EFAULT;
	}
	
	if( a_ctrl->i2c_reg_tbl != NULL ){
		kfree(a_ctrl->i2c_reg_tbl);
	}
	a_ctrl->i2c_reg_tbl =
		kmalloc(sizeof(struct msm_camera_i2c_reg_array) *
		(set_info->af_tuning_params.total_steps + 1), GFP_KERNEL);
	if (!a_ctrl->i2c_reg_tbl) {
		pr_err("kmalloc fail\n");
		return -ENOMEM;
	}

	if (copy_from_user(&a_ctrl->reg_tbl,
		(void *)set_info->actuator_params.reg_tbl_params,
		a_ctrl->reg_tbl_size *
		sizeof(struct msm_actuator_reg_params_t))) {
		kfree(a_ctrl->i2c_reg_tbl);
		a_ctrl->i2c_reg_tbl = NULL;
		return -EFAULT;
	}

	if (set_info->actuator_params.init_setting_size) {
		if (a_ctrl->func_tbl->actuator_init_focus) {
			init_settings = kmalloc(sizeof(struct reg_settings_t) *
				(set_info->actuator_params.init_setting_size),
				GFP_KERNEL);
			if (init_settings == NULL) {
				kfree(a_ctrl->i2c_reg_tbl);
				a_ctrl->i2c_reg_tbl = NULL;
				pr_err("Error allocating memory for init_settings\n");
				return -EFAULT;
			}
			if (copy_from_user(init_settings,
				(void *)set_info->actuator_params.init_settings,
				set_info->actuator_params.init_setting_size *
				sizeof(struct reg_settings_t))) {
				kfree(init_settings);
				kfree(a_ctrl->i2c_reg_tbl);
				a_ctrl->i2c_reg_tbl = NULL;
				pr_err("Error copying init_settings\n");
				return -EFAULT;
			}
			rc = a_ctrl->func_tbl->actuator_init_focus(a_ctrl,
				set_info->actuator_params.init_setting_size,
				a_ctrl->i2c_data_type,
				init_settings);
			kfree(init_settings);
			if (rc < 0) {
				kfree(a_ctrl->i2c_reg_tbl);
				a_ctrl->i2c_reg_tbl = NULL;
				pr_err("Error actuator_init_focus\n");
				return -EFAULT;
			}
		}
	}

	a_ctrl->initial_code = set_info->af_tuning_params.initial_code;
	if (a_ctrl->func_tbl->actuator_init_step_table)
		rc = a_ctrl->func_tbl->
			actuator_init_step_table(a_ctrl, set_info);

	a_ctrl->curr_step_pos = 0;
	a_ctrl->curr_region_index = 0;
	CDBG("Exit\n");

	return rc;
}


static int32_t ak7345_act_i2c_write(struct msm_actuator_ctrl_t *a_ctrl,
	void __user *argp)
{
	struct msm_actuator_cfg_data *cdata = (struct msm_actuator_cfg_data *)argp;
	int32_t rc = -EFAULT;
	int32_t i = 0;
	uint16_t writedata = 0;
	uint8_t* data = NULL;
	CDBG("Enter\n");
	
	data = (uint8_t *)kmalloc(cdata->cfg.i2c_info.length, GFP_KERNEL);
	if(data == NULL){
		pr_err("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}
	
	if(copy_from_user(
			data, 
			(uint8_t*)cdata->cfg.i2c_info.data,
			cdata->cfg.i2c_info.length)){
		kfree(data);
		pr_err("%s copy failed\n", __func__);
		return -EFAULT;
	}
	
	CDBG_DEB("%s i2c_info.addr    = 0x%0x\n", __func__, cdata->cfg.i2c_info.addr);
	CDBG_DEB("%s i2c_info.data[0] = 0x%0x\n", __func__, data[0]);
	CDBG_DEB("%s i2c_info.length  = 0x%0x\n", __func__,   cdata->cfg.i2c_info.length);
	
	for (i = 0; i < cdata->cfg.i2c_info.length; i++) {
		writedata = data[i];
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_write(
			&a_ctrl->i2c_client,
			cdata->cfg.i2c_info.addr+i,
			writedata,
			MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0)
			break;
	}
	
	kfree(data);
	CDBG("Exit\n");
	return rc;
}

static int32_t ak7345_act_i2c_read(struct msm_actuator_ctrl_t *a_ctrl,
	void __user *argp)
{
	struct msm_actuator_cfg_data *cdata = (struct msm_actuator_cfg_data *)argp;
	int32_t rc = -EFAULT;
	int32_t i = 0;
	uint8_t* data = NULL;
	uint16_t readdata = 0;
	CDBG("Enter\n");
	
	data = kzalloc(cdata->cfg.i2c_info.length, GFP_KERNEL);
	if(data == NULL){
		pr_err("%s kmalloc failed\n", __func__);
		return -EFAULT;
	}
	
	CDBG_DEB("%s i2c_info.addr = 0x%0x\n",   __func__, cdata->cfg.i2c_info.addr);
	CDBG_DEB("%s i2c_info.length = 0x%0x\n", __func__, cdata->cfg.i2c_info.length);
	
	for (i = 0; i < cdata->cfg.i2c_info.length; i++) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_read(
			&a_ctrl->i2c_client,
			cdata->cfg.i2c_info.addr+i,
			&readdata, MSM_CAMERA_I2C_BYTE_DATA);
		if (rc < 0){
			kfree(data);
			return -EFAULT;
		}
		data[i] = (uint8_t)readdata;
		
		CDBG_DEB("%s read data[%d] = 0x%0x\n",   __func__, i, data[i]);
	}
	
	if (copy_to_user((uint8_t *)cdata->cfg.i2c_info.data,
		data,
		cdata->cfg.i2c_info.length)){
		kfree(data);
		pr_err("%s copy_to_user error\n",__func__);
		return -EFAULT;
	}
	kfree(data);
	
	CDBG("Exit\n");
	return rc;
}


static int32_t ak7345_act_config(struct msm_actuator_ctrl_t *a_ctrl,
	void __user *argp)
{
	struct msm_actuator_cfg_data *cdata =
		(struct msm_actuator_cfg_data *)argp;
	int32_t rc = 0;
	mutex_lock(a_ctrl->actuator_mutex);
	CDBG("Enter\n");
	CDBG("%s type %d\n", __func__, cdata->cfgtype);
	switch (cdata->cfgtype) {
	case CFG_GET_ACTUATOR_INFO:
		cdata->is_af_supported = 1;
		cdata->cfg.cam_name = a_ctrl->cam_name;
		break;

	case CFG_SET_ACTUATOR_INFO:
		/* no reload */
		if( a_ctrl->step_position_table == NULL){
			rc = ak7345_act_init(a_ctrl, &cdata->cfg.set_info);
		if (rc < 0)
			pr_err("init table failed %d\n", rc);
		}
		break;

	case CFG_SET_DEFAULT_FOCUS:
		rc = a_ctrl->func_tbl->actuator_set_default_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;

	case CFG_MOVE_FOCUS:
		rc = a_ctrl->func_tbl->actuator_move_focus(a_ctrl,
			&cdata->cfg.move);
		if (rc < 0)
			pr_err("move focus failed %d\n", rc);
		break;
	
	case SHCFG_GET_I2C_DATA:
		rc = ak7345_act_i2c_read(a_ctrl, argp);
		if (rc < 0)
			pr_err("get i2c failed %d\n", rc);
		break;
		
	case SHCFG_SET_I2C_DATA:
		rc = ak7345_act_i2c_write(a_ctrl, argp);
		if (rc < 0)
			pr_err("set i2c failed %d\n", rc);
		break;
	
	default:
		break;
	}
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static int32_t ak7345_act_get_subdev_id(struct msm_actuator_ctrl_t *a_ctrl,
	void *arg)
{
	uint32_t *subdev_id = (uint32_t *)arg;
	CDBG("Enter\n");
	if (!subdev_id) {
		pr_err("failed\n");
		return -EINVAL;
	}
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE)
		*subdev_id = a_ctrl->pdev->id;
	else
		*subdev_id = a_ctrl->subdev_id;

	CDBG("subdev_id %d\n", *subdev_id);
	CDBG("Exit\n");
	return 0;
}

static struct msm_camera_i2c_fn_t msm_sensor_cci_func_tbl = {
	.i2c_read = msm_camera_cci_i2c_read,
	.i2c_read_seq = msm_camera_cci_i2c_read_seq,
	.i2c_write = msm_camera_cci_i2c_write,
	.i2c_write_table = msm_camera_cci_i2c_write_table,
	.i2c_write_seq_table = msm_camera_cci_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_cci_i2c_write_table_w_microdelay,
	.i2c_util = msm_sensor_cci_i2c_util,
};

static struct msm_camera_i2c_fn_t msm_sensor_qup_func_tbl = {
	.i2c_read = msm_camera_qup_i2c_read,
	.i2c_read_seq = msm_camera_qup_i2c_read_seq,
	.i2c_write = msm_camera_qup_i2c_write,
	.i2c_write_table = msm_camera_qup_i2c_write_table,
	.i2c_write_seq_table = msm_camera_qup_i2c_write_seq_table,
	.i2c_write_table_w_microdelay =
		msm_camera_qup_i2c_write_table_w_microdelay,
};

static int ak7345_act_open(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	mutex_lock(a_ctrl->actuator_mutex);
	a_ctrl->cci_inited = 0;
	a_ctrl->num_instance++;
	mutex_unlock(a_ctrl->actuator_mutex);
	
#if 0 /* change the timing of init  */
	if (a_ctrl->act_device_type == MSM_CAMERA_PLATFORM_DEVICE) {
		rc = a_ctrl->i2c_client.i2c_func_tbl->i2c_util(
			&a_ctrl->i2c_client, MSM_CCI_INIT);
		if (rc < 0)
			pr_err("cci_init failed\n");
	}
#endif
	CDBG("Exit\n");
	
	return rc;
}

static int ak7345_act_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh) {
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl =  v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	if (!a_ctrl) {
		pr_err("failed\n");
		return -EINVAL;
	}
	mutex_lock(a_ctrl->actuator_mutex);
	a_ctrl->num_instance--;
	if( a_ctrl->num_instance < 0 )
		a_ctrl->num_instance = 0;
	
#ifdef ACTUATOR_DEBUG
	if( a_ctrl->num_instance == 0 )
#endif
	{
		ak7345_act_power_down(a_ctrl);
	}
	
	mutex_unlock(a_ctrl->actuator_mutex);
	
	CDBG("Exit\n");
	return rc;
}

static const struct v4l2_subdev_internal_ops ak7345_act_internal_ops = {
	.open = ak7345_act_open,
	.close = ak7345_act_close,
};

static long ak7345_act_subdev_ioctl(struct v4l2_subdev *sd,
			unsigned int cmd, void *arg)
{
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	void __user *argp = (void __user *)arg;
	CDBG("Enter\n");
	CDBG("%s:%d a_ctrl %p argp %p cmd %x\n", __func__, __LINE__, a_ctrl, argp, cmd);
	switch (cmd) {
	case VIDIOC_MSM_SENSOR_GET_SUBDEV_ID:
		return ak7345_act_get_subdev_id(a_ctrl, argp);
	case VIDIOC_MSM_ACTUATOR_CFG:
		return ak7345_act_config(a_ctrl, argp);
	case MSM_SD_SHUTDOWN:
		ak7345_act_close(sd, NULL);
		return 0;
	default:
		return -ENOIOCTLCMD;
	}
}

static int32_t ak7345_act_power_up(struct msm_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

#if 0
	CDBG("vcm info: %x %x\n", a_ctrl->vcm_pwd,
		a_ctrl->vcm_enable);
	if (a_ctrl->vcm_enable) {
		rc = gpio_request(a_ctrl->vcm_pwd, "msm_actuator");
		if (!rc) {
			CDBG("Enable VCM PWD\n");
			gpio_direction_output(a_ctrl->vcm_pwd, 1);
		}
	}
#endif
	CDBG("Exit\n");
	return rc;
}

static int32_t ak7345_act_power(struct v4l2_subdev *sd, int on)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *a_ctrl = v4l2_get_subdevdata(sd);
	CDBG("Enter\n");
	mutex_lock(a_ctrl->actuator_mutex);
	if (on)
		rc = ak7345_act_power_up(a_ctrl);
	else
		rc = ak7345_act_power_down(a_ctrl);
	mutex_unlock(a_ctrl->actuator_mutex);
	CDBG("Exit\n");
	return rc;
}

static struct v4l2_subdev_core_ops ak7345_act_subdev_core_ops = {
	.ioctl = ak7345_act_subdev_ioctl,
	.s_power = ak7345_act_power,  /* not called in normal power on/off sequence */
};

static struct v4l2_subdev_ops ak7345_act_subdev_ops = {
	.core = &ak7345_act_subdev_core_ops,
};

static const struct i2c_device_id ak7345_act_i2c_id[] = {
	{"ak7345,actuator", (kernel_ulong_t)NULL},
	{ }
};

static int32_t ak7345_act_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;
	struct msm_actuator_ctrl_t *act_ctrl_t = NULL;
	CDBG("Enter\n");

	if (client == NULL) {
		pr_err("ak7345_act_i2c_probe: client is null\n");
		rc = -EINVAL;
		goto probe_failure;
	}

	act_ctrl_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!act_ctrl_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("i2c_check_functionality failed\n");
		goto probe_failure;
	}

	CDBG("client = %x\n", (unsigned int) client);

	rc = of_property_read_u32(client->dev.of_node, "cell-index",
		&act_ctrl_t->subdev_id);
	CDBG("cell-index %d, rc %d\n", act_ctrl_t->subdev_id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	act_ctrl_t->i2c_driver = &ak7345_act_i2c_driver;
	act_ctrl_t->i2c_client.client = client;
	act_ctrl_t->curr_step_pos = 0,
	act_ctrl_t->curr_region_index = 0,
	/* Set device type as I2C */
	act_ctrl_t->act_device_type = MSM_CAMERA_I2C_DEVICE;
	act_ctrl_t->i2c_client.i2c_func_tbl = &msm_sensor_qup_func_tbl;
	act_ctrl_t->act_v4l2_subdev_ops = &ak7345_act_subdev_ops;
	act_ctrl_t->actuator_mutex = &ak7345_act_mutex;

	act_ctrl_t->cam_name = act_ctrl_t->subdev_id;
	CDBG("act_ctrl_t->cam_name: %d", act_ctrl_t->cam_name);
	/* Assign name for sub device */
	snprintf(act_ctrl_t->msm_sd.sd.name, sizeof(act_ctrl_t->msm_sd.sd.name),
		"%s", act_ctrl_t->i2c_driver->driver.name);

	/* Initialize sub device */
	v4l2_i2c_subdev_init(&act_ctrl_t->msm_sd.sd,
		act_ctrl_t->i2c_client.client,
		act_ctrl_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&act_ctrl_t->msm_sd.sd, act_ctrl_t);
	act_ctrl_t->msm_sd.sd.internal_ops = &ak7345_act_internal_ops;
	act_ctrl_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	media_entity_init(&act_ctrl_t->msm_sd.sd.entity, 0, NULL, 0);
	act_ctrl_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	act_ctrl_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	act_ctrl_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&act_ctrl_t->msm_sd);
#if 1
	act_ctrl_t->num_instance = 0;
	act_ctrl_t->cci_inited   = 0;
#endif

	pr_info("ak7345_act_i2c_probe: succeeded\n");
	CDBG("Exit\n");

probe_failure:
	return rc;
}

static int32_t ak7345_act_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	struct msm_camera_cci_client *cci_client = NULL;
	struct msm_actuator_ctrl_t *msm_actuator_t = NULL;
	CDBG("Enter\n");

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}

	msm_actuator_t = kzalloc(sizeof(struct msm_actuator_ctrl_t),
		GFP_KERNEL);
	if (!msm_actuator_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	rc = of_property_read_u32((&pdev->dev)->of_node, "cell-index",
		&pdev->id);
	CDBG("cell-index %d, rc %d\n", pdev->id, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	rc = of_property_read_u32((&pdev->dev)->of_node, "qcom,cci-master",
		&msm_actuator_t->cci_master);
	CDBG("qcom,cci-master %d, rc %d\n", msm_actuator_t->cci_master, rc);
	if (rc < 0) {
		pr_err("failed rc %d\n", rc);
		return rc;
	}

	msm_actuator_t->act_v4l2_subdev_ops = &ak7345_act_subdev_ops;
	msm_actuator_t->actuator_mutex = &ak7345_act_mutex;
	msm_actuator_t->cam_name = pdev->id;

	/* Set platform device handle */
	msm_actuator_t->pdev = pdev;
	/* Set device type as platform device */
	msm_actuator_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	msm_actuator_t->i2c_client.i2c_func_tbl = &msm_sensor_cci_func_tbl;
	msm_actuator_t->i2c_client.cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!msm_actuator_t->i2c_client.cci_client) {
		pr_err("failed no memory\n");
		return -ENOMEM;
	}

	cci_client = msm_actuator_t->i2c_client.cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	v4l2_subdev_init(&msm_actuator_t->msm_sd.sd,
		msm_actuator_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&msm_actuator_t->msm_sd.sd, msm_actuator_t);
	msm_actuator_t->msm_sd.sd.internal_ops = &ak7345_act_internal_ops;
	msm_actuator_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(msm_actuator_t->msm_sd.sd.name,
		ARRAY_SIZE(msm_actuator_t->msm_sd.sd.name), "ak7345_act");
	media_entity_init(&msm_actuator_t->msm_sd.sd.entity, 0, NULL, 0);
	msm_actuator_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	msm_actuator_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_ACTUATOR;
	msm_actuator_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&msm_actuator_t->msm_sd);
#if 1
	msm_actuator_t->num_instance = 0;
	msm_actuator_t->cci_inited   = 0;
#endif
	
	
	CDBG("Exit\n");
	return rc;
}

static const struct of_device_id ak7345_act_i2c_dt_match[] = {
	{.compatible = "ak7345,actuator"},
	{}
};

MODULE_DEVICE_TABLE(of, ak7345_act_i2c_dt_match);

static struct i2c_driver ak7345_act_i2c_driver = {
	.id_table = ak7345_act_i2c_id,
	.probe  = ak7345_act_i2c_probe,
	.remove = __exit_p(ak7345_act_i2c_remove),
	.driver = {
		.name = "ak7345,actuator",
		.owner = THIS_MODULE,
		.of_match_table = ak7345_act_i2c_dt_match,
	},
};

static const struct of_device_id ak7345_act_dt_match[] = {
	{.compatible = "ak7345,actuator", .data = NULL},
	{}
};

MODULE_DEVICE_TABLE(of, ak7345_act_dt_match);

static struct platform_driver ak7345_act_platform_driver = {
	.driver = {
		.name = "ak7345,actuator",
		.owner = THIS_MODULE,
		.of_match_table = ak7345_act_dt_match,
	},
};

static int __init ak7345_act_init_module(void)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	rc = platform_driver_probe(&ak7345_act_platform_driver,
		ak7345_act_platform_probe);
	if (!rc)
		return rc;
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);
	return i2c_add_driver(&ak7345_act_i2c_driver);
}

static struct msm_actuator ak7345_act_table = {
	.act_type = ACTUATOR_VCM,
	.func_tbl = {
		.actuator_init_step_table = ak7345_act_init_step_table,
		.actuator_move_focus = ak7345_act_move_focus,
		.actuator_write_focus = ak7345_act_write_focus,
		.actuator_set_default_focus = ak7345_act_set_default_focus,
		.actuator_init_focus = ak7345_act_init_focus,
		.actuator_parse_i2c_params = ak7345_act_parse_i2c_params,
	},
};

module_init(ak7345_act_init_module);
MODULE_DESCRIPTION("AK7345 ACTUATOR");
MODULE_LICENSE("GPL v2");
