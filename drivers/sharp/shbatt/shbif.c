#if defined(CONFIG_BIF) && defined(CONFIG_PM_SUPPORT_BIF)

#include "shbif.h"
#include "shbatt_type.h"
#include <linux/bif/consumer.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/err.h>
#include <linux/mutex.h>

//#define DUMP_DATA
//#define SHBIF_ENABLE_TRACE
//#define SHBIF_ENABLE_INFO



#ifdef SHBIF_ENABLE_TRACE
#define SHBIF_TRACE(x...)	printk(KERN_DEBUG"[SHBATT] "x)
#else
#define SHBIF_TRACE(x...)
#endif

#if defined(SHBIF_ENABLE_INFO) || defined(DUMP_DATA)
#define SHBATT_INFO(x...)	printk(KERN_INFO "[SHBATT] " x)
#else
#define SHBATT_INFO(x...)
#endif

#define SHBIF_ERROR(x...)	printk(KERN_ERR "[SHBATT] " x)


struct bif_ctrl* bif_ctl = NULL;
struct bif_slave* bif_sl = NULL;

static const uint16_t infineon_ag = 0x011A;
static const uint16_t origa2 = 0x0619;

typedef struct
{
	int transaction;
	uint8_t send_data;
	int *response;
}shbif_receive_raw_word_t;

typedef struct
{
	int transaction;
	uint8_t data;
}shbif_send_raw_word_t;

typedef struct
{
	int transaction;
	uint8_t data;
	int *query_response;
}shbif_raw_transaction_query_t;

typedef struct 
{
	uint16_t address;
	uint8_t* data;
	int size;
}shbif_write_register_space_t;

typedef shbif_write_register_space_t shbif_read_register_space_t;

#define SHBIF_IOCTL_RECEIVE_RAW_WORD					_IOR( SHBIF_IOCTL_MAGIC,   1, shbif_receive_raw_word_t)
#define SHBIF_IOCTL_SEND_RAW_WORD						_IOW( SHBIF_IOCTL_MAGIC,   2, shbif_send_raw_word_t)
#define SHBIF_IOCTL_RAW_TRANSACTION_QUERY				_IOR( SHBIF_IOCTL_MAGIC,   3, shbif_raw_transaction_query_t)
#define SHBIF_IOCTL_WRITE_REGISTER_SPACE				_IOW( SHBIF_IOCTL_MAGIC,   4, shbif_write_register_space_t)
#define SHBIF_IOCTL_READ_REGISTER_SPACE					_IOR( SHBIF_IOCTL_MAGIC,   5, shbif_read_register_space_t)
#define SHBIF_IOCTL_GET_UNIQUE_ID						_IOR( SHBIF_IOCTL_MAGIC,   6, uint8_t*)
#define SHBIF_IOCTL_INITIALIZE							_IO( SHBIF_IOCTL_MAGIC,   7)
#define SHBIF_IOCTL_BUS_POWER_DOWN						_IO( SHBIF_IOCTL_MAGIC,   8)

inline bool shbif_is_device_found(void)
{
	return bif_ctl != NULL && bif_sl != NULL;
}
static DEFINE_MUTEX(bif_access_mutex);

static int shbif_receive_raw_word(unsigned long arg)
{
	int result;
	int response;
	shbif_receive_raw_word_t data;
	
	SHBIF_TRACE("[S]%s\n",__FUNCTION__);
	
	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("[E]%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}
	
	result = copy_from_user(&data, (void*)arg, sizeof(data));
	if(result == 0)
	{
		result = bif_ctrl_raw_transaction_read(bif_ctl, data.transaction, data.send_data, &response);
		if(result == 0)
		{
			result = copy_to_user(data.response, &response, sizeof(response));
		}
	}
#ifdef DUMP_DATA
	SHBIF_INFO("[D]%s,transaction=%02x,send_data=%02x,response=%d\n", __FUNCTION__, data.transaction, data.send_data, response);
#endif
	SHBIF_TRACE("[E]%s result=%d,transaction=%d data=%02x\n",__FUNCTION__, result,  data.transaction, data.send_data);
	return result;
}
static int shbif_send_raw_word(unsigned long arg)
{
	int result;
	shbif_send_raw_word_t data;

	SHBIF_TRACE("[S]%s\n",__FUNCTION__);

	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("[E]%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}

	result = copy_from_user(&data, (void*)arg, sizeof(data));
	if(result == 0)
	{
		result = bif_ctrl_raw_transaction(bif_ctl, data.transaction, data.data);
	}
#ifdef DUMP_DATA
	SHBIF_INFO("[D]%s,transaction=%02x,send_data=%02x\n", __FUNCTION__, data.transaction, data.data);
#endif
	SHBIF_TRACE("[E]%s result=%d,transaction=%d data=%02x\n",__FUNCTION__, result, data.transaction, data.data);
	return result;
}
static int shbif_raw_transaction_query(unsigned long arg)
{
	int result;
	bool response = false;
	shbif_raw_transaction_query_t data;

	SHBIF_TRACE("[S]%s\n",__FUNCTION__);

	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}

	result = copy_from_user(&data, (void*)arg, sizeof(data));
	if(result == 0)
	{
		result = bif_ctrl_raw_transaction_query(bif_ctl, data.transaction, data.data, &response);
		if(result == 0)
		{
			int res = response ? 1:0;
			result = copy_to_user(data.query_response, &res, sizeof(res));
		}
	}
#ifdef DUMP_DATA	
	SHBIF_INFO("[D]%s,transaction=%02x,send_data=%02x,response=%d\n", __FUNCTION__, data.transaction, data.data, response);
#endif
	SHBIF_TRACE("[E]%s result=%d,response=%d transaction=%d data=%02x\n",__FUNCTION__, result, response, data.transaction, data.data);
	return result;
}
static int shbif_write_register_space(unsigned long arg)
{
	int result;
	shbif_write_register_space_t data;

	SHBIF_TRACE("[S]%s\n",__FUNCTION__);

	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}

	result = copy_from_user(&data, (void*)arg, sizeof(data));
	if(result == 0)
	{
		uint8_t *write_data = (uint8_t*)kzalloc(data.size, GFP_KERNEL);
		if(write_data != NULL)
		{
			result = copy_from_user(write_data, data.data, data.size);
			if(result == 0)
			{
				result = bif_slave_write(bif_sl, data.address, write_data, data.size);
#ifdef DUMP_DATA
				{
					char msg[512];
					int current_pos = 0;
					int i;
					for(i = 0; i < data.size; i++)
					{
						current_pos += sprintf(&msg[current_pos], "%02x ", write_data[i]);
						if(i != 0 && i % 16 == 0)
						{
							current_pos += sprintf(&msg[current_pos], "\n");
						}
					}
					
					
					SHBIF_INFO("[D]%s address=%04x size=%02x\n",__FUNCTION__, data.address, data.size);
					SHBIF_INFO("%s\n", msg);
				}
#endif
			}
			kfree(write_data);
		}
		else
		{
			result = -ENOMEM;
		}
	}
	
	SHBIF_TRACE("[E]%s result=%d address=%04x size=%02x\n",__FUNCTION__, result, data.address, data.size);
	return result;
}
static int shbif_read_register_space(unsigned long arg)
{
	int result;
	shbif_read_register_space_t data;

	SHBIF_TRACE("[S]%s\n",__FUNCTION__);

	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}


	result = copy_from_user(&data, (void*)arg, sizeof(data));
	if(result == 0)
	{
		uint8_t *read_data = (uint8_t*)kzalloc(data.size, GFP_KERNEL);
		if(read_data != NULL)
		{
			result = bif_slave_read(bif_sl, data.address, read_data, data.size);
			if(result == 0)
			{
				result = copy_to_user(data.data, read_data, data.size);
#ifdef DUMP_DATA
				{
					char msg[512];
					int current_pos = 0;
					int i;
					for(i = 0; i < data.size; i++)
					{
						current_pos += sprintf(&msg[current_pos], "%02x ", read_data[i]);
						if(i != 0 && i % 16 == 0)
						{
							current_pos += sprintf(&msg[current_pos], "\n");
						}
					}
					
					
					SHBIF_INFO("[D]%s address=%04x size=%02x\n",__FUNCTION__, data.address, data.size);
					SHBIF_INFO("%s\n", msg);
				}
#endif
			}
			kfree(read_data);
		}
		else
		{
			result = -ENOMEM;
		}
	}
	SHBIF_TRACE("[E]%s result=%d address=%04x size=%02x\n",__FUNCTION__, result, data.address, data.size);
	return result;
}

int shbif_initialize(void)
{
	struct bif_match_criteria criteria;
	int control_count;
	int err;
	int rid;
	
	SHBIF_TRACE("[S]%s\n",__FUNCTION__);
	memset(&criteria, 0, sizeof(criteria));
	
	criteria.match_mask = BIF_MATCH_MANUFACTURER_ID | BIF_MATCH_PRODUCT_ID;
	criteria.manufacturer_id = infineon_ag;
	criteria.product_id = origa2;
	
	control_count = bif_ctrl_count();
	if(control_count <= 0)
	{
		SHBIF_ERROR("%s control count <= 0,NODEV", __FUNCTION__);
		err =  -ENODEV;
		goto error;
	}
	
	bif_ctl = bif_ctrl_get_by_id(0);
	if(IS_ERR(bif_ctl))
	{
		err = PTR_ERR(bif_ctl);
		SHBIF_ERROR("bif_ctrl_get_by_id, %s,err=%d", __FUNCTION__, err);
		goto error;
	}
	
	rid = bif_ctrl_measure_rid(bif_ctl);
	if(rid > BIF_BATT_RID_SMART_MAX || BIF_BATT_RID_SMART_MIN > rid)
	{
		SHBIF_ERROR("not smart, %s,rid=%d", __FUNCTION__, rid);
		err = -ENODEV;
		goto put_control;
	}
	
	bif_sl = bif_slave_match_get(bif_ctl, 0, &criteria);
	if(IS_ERR(bif_sl))
	{
		err = PTR_ERR(bif_sl);
		SHBIF_ERROR("bif_slave_match_get, %s,err=%d", __FUNCTION__, err);
		goto put_control;
	}
	
	SHBIF_TRACE("[E]%s\n",__FUNCTION__);
	return 0;

put_control:
	bif_ctrl_put(bif_ctl);
error:
	bif_ctl = NULL;
	bif_sl = NULL;
	SHBIF_ERROR("[E]%s err=%d\n",__FUNCTION__, err);
	return err;
}

static int shbif_get_unique_id(unsigned long arg)
{
	int result;
	uint8_t unique_id[BIF_UNIQUE_ID_BYTE_LENGTH];
	SHBIF_TRACE("[S]%s\n",__FUNCTION__);
	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("[E]%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}
	
	result = bif_slave_get_unique_id(bif_sl,unique_id);
	if(result == 0)
	{
		result = copy_to_user((void*)arg, unique_id, sizeof(unique_id));
	}
	SHBIF_TRACE("[E]%s\n",__FUNCTION__);
	return result;
}

bool shbif_cmd_is_bif_command(unsigned int command)
{
	switch(command)
	{
		case SHBIF_IOCTL_RECEIVE_RAW_WORD:
		case SHBIF_IOCTL_SEND_RAW_WORD:
		case SHBIF_IOCTL_RAW_TRANSACTION_QUERY:
		case SHBIF_IOCTL_WRITE_REGISTER_SPACE:
		case SHBIF_IOCTL_READ_REGISTER_SPACE:
		case SHBIF_IOCTL_GET_UNIQUE_ID:
		case SHBIF_IOCTL_INITIALIZE:
		case SHBIF_IOCTL_BUS_POWER_DOWN:
			return true;
		default:
			return false;
	}
}

static int shbif_bus_power_down(void)
{
	int result;
	SHBIF_TRACE("[S]%s\n",__FUNCTION__);

	if(!shbif_is_device_found())
	{
		SHBIF_ERROR("[E]%s ENODEV\n",__FUNCTION__);
		return -ENODEV;
	}
	
	result = bif_ctrl_set_bus_state(bif_ctl, BIF_BUS_STATE_POWER_DOWN);

	if(result != 0)
	{
		SHBIF_ERROR("%s bif_ctrl_set_bus_state failed.result=%d", __FUNCTION__, result);
	}

	SHBIF_TRACE("[E]%s\n",__FUNCTION__);
	return result;
}

int shbif_handle_bif_access_command( struct file* fi_p, unsigned int cmd, unsigned long arg )
{
	int result = -EINVAL;
	mutex_lock(&bif_access_mutex);
	SHBIF_TRACE("[S]%s\n",__FUNCTION__);
	
	switch(cmd)
	{
		case SHBIF_IOCTL_RECEIVE_RAW_WORD:
			result = shbif_receive_raw_word(arg);
		break;
		
		case SHBIF_IOCTL_SEND_RAW_WORD:
			result = shbif_send_raw_word(arg);
		break;
		
		case SHBIF_IOCTL_RAW_TRANSACTION_QUERY:
			result = shbif_raw_transaction_query(arg);
		break;
		
		case SHBIF_IOCTL_WRITE_REGISTER_SPACE:
			result = shbif_write_register_space(arg);
		break;
		
		case SHBIF_IOCTL_READ_REGISTER_SPACE:
			result = shbif_read_register_space(arg);
		break;

		case SHBIF_IOCTL_GET_UNIQUE_ID:
			result = shbif_get_unique_id(arg);
		break;
		
		case SHBIF_IOCTL_INITIALIZE:
			result = shbif_initialize();
		break;
		
		case SHBIF_IOCTL_BUS_POWER_DOWN:
			result = shbif_bus_power_down();
		break;
		
		default:
			/*nothing todo*/
		break;
	}
	SHBIF_TRACE("[E]%s result=%d\n",__FUNCTION__, result);
	mutex_unlock(&bif_access_mutex);
	return result;
}

#endif
