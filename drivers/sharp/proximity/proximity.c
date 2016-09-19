/* driver/sharp/proximity/proximity.c  (proximitySensor Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION All rights reserved.
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

/* ------------------------------------------------------------------------- */
/* SHARP PROXIMITY SENSOR DRIVER FOR KERNEL MODE                             */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* INCLUDE FILES                                                             */
/* ------------------------------------------------------------------------- */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <mach/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/device.h>
#include <sharp/proximity.h>
#include <linux/hrtimer.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/regulator/consumer.h>
#include <linux/wakelock.h>
#include <mach/vreg.h>
#include <sharp/sh_smem.h>

#include<sharp/shdisp_kerl.h>
#include <linux/module.h>

#include <linux/proc_fs.h>

/* ------------------------------------------------------------------------- */
/* MACRAOS                                                                   */
/* ------------------------------------------------------------------------- */

#define PROX_USE_SMEM

#define HW_ES0      0x00
#define HW_ES05     0x01
#define HW_ES1      0x02
#define HW_PP1      0x03
#define HW_PP2      0x04
#define HW_PM       0x07

#define MODEL_TYPE_A1       0x01
#define MODEL_TYPE_S1       0x02
#define MODEL_TYPE_D1       0x03
#define MODEL_TYPE_D2       0x04
#define MODEL_TYPE_D3       0x05
#define MODEL_TYPE_D4       0x06

/* adb debug_log */
static int    proximity_dbg_func_log = 0;      /* log : Init = OFF */
static int    proximity_dbg_func_fin_log = 0;  /* log : Init = OFF */
static int    proximity_dbg_enable_log = 0;    /* log : Init = OFF */
static int    proximity_dbg_sensor_log = 0;    /* log : Init = OFF */
static int    proximity_dbg_error_log = 1;     /* log : Init = ON  */

#if defined (CONFIG_ANDROID_ENGINEERING)
    module_param(proximity_dbg_func_log, int, 0600);
    module_param(proximity_dbg_func_fin_log, int, 0600);
    module_param(proximity_dbg_enable_log, int, 0600);
    module_param(proximity_dbg_sensor_log, int, 0600);
    module_param(proximity_dbg_error_log, int, 0600);
#endif  /* CONFIG_ANDROID_ENGINEERING */

#define FUNC_LOG() \
    if(proximity_dbg_func_log == 1){ \
       printk(KERN_DEBUG "[PROXIMITY] %s is called\n", __func__); \
    }

#define FUNC_FIN_LOG() \
    if(proximity_dbg_func_fin_log == 1){ \
       printk(KERN_DEBUG "[PROXIMITY] %s is finished\n", __func__); \
    }

#define DEBUG_LOG(format, ...) \
    if(proximity_dbg_enable_log == 1){ \
       printk(KERN_DEBUG "[PROXIMITY][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }

#define DEBUG_SENSOR_LOG(format, ...) \
    if(proximity_dbg_sensor_log == 1){ \
       printk(KERN_DEBUG "[PROXIMITY][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }

#define DEBUG_ERROR_LOG(format, ...) \
    if(proximity_dbg_error_log == 1){ \
       printk(KERN_DEBUG "[PROXIMITY][%s] " format "\n", __func__, ## __VA_ARGS__); \
    }

/* ------------------------------------------------------------------------- */
/* PROTOTYPES                                                                */
/* ------------------------------------------------------------------------- */

static uint16_t sh_get_hw_revision(void);
static void prox_subscribe( void );
static void prox_unsubscribe( void );
//static int PROX_I2cWrite(unsigned char RegAdr, unsigned char wData);
static int PROX_I2cRead(unsigned char RegAdr,unsigned char *rData);
static int IOECS_Enable(void);
static int IOECS_Disable(void);
static int IOECS_SetCycle(short cycle_data);
static int IOECS_GetVO_DATA(uint8_t *cycle_data);
static int IOECS_GetD2_DATA(unsigned short *psresult);
static int IOECS_LT_Thresshold_Write(unsigned short lt_threshold);
static int IOECS_HT_Thresshold_Write(unsigned short ht_threshold);
static int PROX_open(struct inode *inode, struct file *filp);
static int PROX_release(struct inode *inode, struct file *filp);
static int PROX_read(struct file *filp, char __user *buf, size_t count, loff_t *ppos);
static long PROX_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);
static void PROX_Irq_workfunc( void );
//static int PROX_ConfigGPIO(void);
static int PROX_Initialize(void);
static int PROX_Remove(void);
static int PROX_Probe(void);
#if defined (CONFIG_ANDROID_ENGINEERING)
static int proximity_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);
#endif /* CONFIG_ANDROID_ENGINEERING */
static int __init PROX_Init(void);
static void __exit PROX_Exit(void);

/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */

typedef struct drv_data_tag     drv_data;
typedef struct work_struct      WorkStruct;
typedef struct input_dev        InputDev;
typedef struct device           Device;

static int Threshold_Low;
static int Threshold_High;

struct drv_data_tag
{
    int         irq_gpio;
    InputDev    *input_dev;
    WorkStruct  IrqWork;
    int         irq;
};

static drv_data     *poProximityRec;
static atomic_t     open_flag = ATOMIC_INIT(0);
static atomic_t     sensor_data = ATOMIC_INIT(7);    /* Init = Far */
static atomic_t     enable_mode = ATOMIC_INIT(0);    /* 0=Disable,1=Enable */


/* adb debug */
static int          sensor_data_tmp = 7;    /* for debug : Init = Far */
static int          prox_dbg_mode   = 0;    /* 0=Disable, 1=Enable */

module_param(sensor_data_tmp, int, 0600);
module_param(prox_dbg_mode, int, 0600);

static struct wake_lock prox_timeout_wake_lock;
static struct wake_lock prox_wake_lock;

static struct file_operations PROX_ctl_fops = {
    .owner          = THIS_MODULE,
    .open           = PROX_open,
    .release        = PROX_release,
    .unlocked_ioctl = PROX_ioctl,
    .read           = PROX_read,
};

static struct miscdevice PROX_device = {
    .minor   = MISC_DYNAMIC_MINOR,
    .name    = "proximity_dev",
    .fops    = &PROX_ctl_fops,
};

/* ------------------------------------------------------------------------- */
/* FUNCTIONS                                                                 */
/* ------------------------------------------------------------------------- */

/* ------------------------------------------------------------------------- */
/* sh_get_hw_revision                                                        */
/* ------------------------------------------------------------------------- */

static uint16_t sh_get_hw_revision(void)
{
    sharp_smem_common_type *p_sharp_smem_common_type;

    FUNC_LOG();

    p_sharp_smem_common_type = sh_smem_get_common_address();

    if( p_sharp_smem_common_type != 0 ) {
        return p_sharp_smem_common_type->sh_hw_revision;
    }
    else {
        return 0xFF;
    }

    FUNC_FIN_LOG();
}


/* ------------------------------------------------------------------------- */
/* prox_subscribe                                                            */
/* ------------------------------------------------------------------------- */

static void prox_subscribe( void )
{
    int nResult;
    struct shdisp_subscribe prox_subscribe;

    FUNC_LOG();

    prox_subscribe.irq_type = SHDISP_IRQ_TYPE_PS;
    prox_subscribe.callback = PROX_Irq_workfunc;

    nResult = shdisp_api_event_subscribe(&prox_subscribe);
    
    if(nResult != SHDISP_RESULT_SUCCESS) {
        DEBUG_ERROR_LOG("Proximity_subscribe Error");
    }

    FUNC_FIN_LOG();
}


/* ------------------------------------------------------------------------- */
/* prox_unsubscribe                                                          */
/* ------------------------------------------------------------------------- */

static void prox_unsubscribe( void )
{
    int nResult;

    FUNC_LOG();

    nResult = shdisp_api_event_unsubscribe(SHDISP_IRQ_TYPE_PS);

    if(nResult != SHDISP_RESULT_SUCCESS) {
        DEBUG_ERROR_LOG("Proximity_unsubscribe Error");
    }

    FUNC_FIN_LOG();
}


/* ------------------------------------------------------------------------- */
/* PROX_I2cWrite                                                             */
/* ------------------------------------------------------------------------- */

#if 0
static int PROX_I2cWrite(unsigned char RegAdr, unsigned char wData)
{
    int nResult = 0;
    struct shdisp_bdic_i2c_msg i2c_msg;
    unsigned char i2c_wbuf[2];

	FUNC_LOG();

    i2c_wbuf[0]  = RegAdr;
    i2c_wbuf[1]  = wData;
    i2c_msg.addr = 0x39;
    i2c_msg.mode = SHDISP_BDIC_I2C_M_W;
    i2c_msg.wlen = 2;
    i2c_msg.rlen = 0;
    i2c_msg.wbuf = &i2c_wbuf[0];
    i2c_msg.rbuf = NULL;

    nResult = shdisp_api_write_bdic_i2c(&i2c_msg);
    if(nResult != SHDISP_RESULT_SUCCESS) {
        DEBUG_ERROR_LOG("I2C_WriteError id = %d",nResult);
    }

    DEBUG_SENSOR_LOG("I2cWrite(reg:%02X,Data:%02X)", i2c_wbuf[0], i2c_wbuf[1]);

    FUNC_FIN_LOG();
    return nResult;
}
#endif


/* ------------------------------------------------------------------------- */
/* PROX_I2cRead                                                              */
/* ------------------------------------------------------------------------- */

static int PROX_I2cRead(unsigned char RegAdr,unsigned char *rData)
{
    int nResult = 0;
    int nRetry = 0;
    struct shdisp_bdic_i2c_msg i2c_msg;
    unsigned char i2c_wbuf[1];
    unsigned char i2c_rbuf[1];

#ifdef PROXIMITY_DBG_SENSOR
    FUNC_LOG();
#endif /* PROXIMITY_DBG_SENSOR */

    if (rData == NULL) {
        DEBUG_ERROR_LOG("[PROX_I2cRead NULL_Error : rData]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    i2c_wbuf[0]  = RegAdr;
    i2c_rbuf[0]  = 0x00;
    i2c_msg.addr = 0x39;
    i2c_msg.mode = SHDISP_BDIC_I2C_M_R;
    i2c_msg.wlen = 1;
    i2c_msg.rlen = 1;
    i2c_msg.wbuf = &i2c_wbuf[0];
    i2c_msg.rbuf = &i2c_rbuf[0];

    for(nRetry=0;nRetry<5;nRetry++) {
        nResult = shdisp_api_read_bdic_i2c(&i2c_msg);
        if(nResult == SHDISP_RESULT_SUCCESS) {
            /*DEBUG_SENSOR_LOG("I2cRead : nResult = %d", nResult);*/
            break;
        }
        DEBUG_SENSOR_LOG("I2cReadError retry : nResult = %d", nResult);
        if (nRetry < 4) msleep(20);
    }

    if(nResult != SHDISP_RESULT_SUCCESS) {
        DEBUG_ERROR_LOG("I2C_ReadError id = %d  retry = %d", nResult, nRetry);
        return nResult;
    }
    /* DEBUG_SENSOR_LOG("I2cRead(reg:%02X,Data:%02X)", i2c_wbuf[0], i2c_rbuf[0]);*/
    DEBUG_SENSOR_LOG("Addr: %02X  Data:%02X)", i2c_wbuf[0], i2c_rbuf[0]);

    *rData = i2c_rbuf[0];

#ifdef PROXIMITY_DBG_SENSOR
    FUNC_FIN_LOG();
#endif /* PROXIMITY_DBG_SENSOR */

    return nResult;
}


/* ------------------------------------------------------------------------- */
/* IOECS_Enable                                                              */
/* ------------------------------------------------------------------------- */

static int IOECS_Enable(void)
{
    int nResult = SH_PROXIMITY_RESULT_FAILURE;
    uint16_t rev;

    struct shdisp_prox_params prox_params;
    unsigned char rData = 0x00;

#if CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D4

    unsigned short proxadj[2];
    sharp_smem_common_type *p_sh_smem_common_type = NULL;

    FUNC_LOG();

    if(atomic_read(&enable_mode) == 0) {
        rev = sh_get_hw_revision();
        rev = rev & 0x07;
        DEBUG_LOG("hw_revision = %02X",rev);

        prox_subscribe();

        if(rev != HW_ES1) {
            memset((void*)proxadj, 0x00, sizeof(proxadj));
            p_sh_smem_common_type = sh_smem_get_common_address();
            memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
            prox_params.threshold_low  = proxadj[0];
            prox_params.threshold_high = proxadj[1];
            DEBUG_LOG("[%s][smem] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
        }
        else {
            prox_params.threshold_low  = Threshold_Low;
            prox_params.threshold_high = Threshold_High;
            DEBUG_LOG("[%s][local] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
        }

        nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);

        if(nResult != SHDISP_RESULT_SUCCESS) {
            DEBUG_ERROR_LOG("I2C PowerON Error");
            nResult = SH_PROXIMITY_RESULT_FAILURE;
            return nResult;
        }

        atomic_set(&enable_mode, 1);
    }

#else

    unsigned short proxadj[2];
    sharp_smem_common_type *p_sh_smem_common_type = NULL;

    FUNC_LOG();

    if(atomic_read(&enable_mode) == 0) {
        rev = sh_get_hw_revision();
        rev = rev & 0x07;
        DEBUG_LOG("hw_revision = %02X",rev);

        prox_subscribe();

        memset((void*)proxadj, 0x00, sizeof(proxadj));
        p_sh_smem_common_type = sh_smem_get_common_address();

#ifdef PROX_USE_SMEM
          if (p_sh_smem_common_type != NULL) {
            memcpy(proxadj, p_sh_smem_common_type->shdiag_proxadj, sizeof(proxadj));
            prox_params.threshold_low = proxadj[0];
            prox_params.threshold_high = proxadj[1];
            DEBUG_LOG("[%s][smem] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
        }
        else {
            prox_params.threshold_low = Threshold_Low;
            prox_params.threshold_high = Threshold_High;
            DEBUG_LOG("[%s][local] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
        }
#else   /* PROX_USE_SMEM */
        prox_params.threshold_low = Threshold_Low;
        prox_params.threshold_high = Threshold_High;
        DEBUG_LOG("[%s][local] LT:0x%04x, HT:0x%04x.\n", __func__, prox_params.threshold_low, prox_params.threshold_high);
#endif  /* PROX_USE_SMEM */

        nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_ON, &prox_params);

        if(nResult != SHDISP_RESULT_SUCCESS) {
            DEBUG_ERROR_LOG("I2C PowerON Error");
            nResult = SH_PROXIMITY_RESULT_FAILURE;
            return nResult;
        }

        atomic_set(&enable_mode, 1);
    }

#endif

    nResult = PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);
    if(nResult != 0){
        DEBUG_ERROR_LOG("I2CRead_Error");
    }
    rData = (rData & 0x08);

    if(0x08 == rData){
        atomic_set(&sensor_data, 0);
    }else {
        atomic_set(&sensor_data, 7);
    }

        input_abs_set_val(poProximityRec->input_dev, ABS_DISTANCE, -1);
        input_report_abs(poProximityRec->input_dev, ABS_DISTANCE, atomic_read(&sensor_data));
        input_sync(poProximityRec->input_dev);

    FUNC_FIN_LOG();
    return nResult;
}


/* ------------------------------------------------------------------------- */
/* IOECS_Disable                                                             */
/* ------------------------------------------------------------------------- */

static int IOECS_Disable(void)
{
    int nResult = SH_PROXIMITY_RESULT_FAILURE;
    uint16_t rev;

    FUNC_LOG();

    if(atomic_read(&enable_mode) == 1) {
        rev = sh_get_hw_revision();
        rev = rev & 0x07;

        prox_unsubscribe();

        nResult = shdisp_api_prox_sensor_pow_ctl(SHDISP_PROX_SENSOR_POWER_OFF, NULL);
        
        if(nResult != SHDISP_RESULT_SUCCESS) {
            DEBUG_ERROR_LOG("I2C PowerOFF Error");
            nResult = SH_PROXIMITY_RESULT_FAILURE;
            return nResult;
        }
        atomic_set(&enable_mode, 0);
    }
    FUNC_FIN_LOG();
    return nResult;
}


/* ------------------------------------------------------------------------- */
/* IOECS_SetCycle                                                            */
/* ------------------------------------------------------------------------- */

static int IOECS_SetCycle(short cycle_data)
{
    FUNC_LOG();

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* IOECS_GetVO_DATA                                                          */
/* ------------------------------------------------------------------------- */

static int IOECS_GetVO_DATA(uint8_t *cycle_data)
{
    FUNC_LOG();

    if (cycle_data == NULL) {
        DEBUG_ERROR_LOG("[IOECS_GetVO_DATA NULL_Error : cycle_data]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* IOECS_GetD2_DATA                                                          */
/* ------------------------------------------------------------------------- */

static int IOECS_GetD2_DATA(unsigned short *psresult)
{
    int nResult = 0;
    unsigned char lData  = 0x00;
    unsigned short LData = 0x00;
    unsigned char mData  = 0x00;
    unsigned short MData = 0x00;

    FUNC_LOG();

    if (psresult == NULL) {
        DEBUG_ERROR_LOG("[IOECS_GetD2_DATA NULL_Error : psresult]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    if(atomic_read(&enable_mode) == 1) {

        nResult = PROX_I2cRead(GP2AP030_REG_D2_LSB,&lData);

        if(nResult != 0) {
            DEBUG_ERROR_LOG("I2CRead_Error");
        }
        
        LData = ( lData & 0x00FF );
        nResult = PROX_I2cRead(GP2AP030_REG_D2_MSB,&mData);

        if(nResult != 0) {
            DEBUG_ERROR_LOG("I2CRead_Error");
        }
        
        MData = ( mData & 0x00FF );
        *psresult = (( MData<<8 ) | LData );
        DEBUG_SENSOR_LOG("psresult = 0x%04x",*psresult);

    }

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* IOECS_LT_Thresshold_Write                                                 */
/* ------------------------------------------------------------------------- */

static int IOECS_LT_Thresshold_Write(unsigned short lt_threshold)
{
    FUNC_LOG();

    Threshold_Low = lt_threshold;

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* IOECS_HT_Thresshold_Write                                                 */
/* ------------------------------------------------------------------------- */

static int IOECS_HT_Thresshold_Write(unsigned short ht_threshold)
{
    FUNC_LOG();
    
    Threshold_High = ht_threshold;

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* PROX_open                                                                 */
/* ------------------------------------------------------------------------- */

static int PROX_open(struct inode *inode, struct file *filp)
{
    int nResult = SH_PROXIMITY_RESULT_FAILURE;

    FUNC_LOG();
    
    if (inode == NULL) {
        DEBUG_ERROR_LOG("[PROX_open NULL_Error : inode]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    if (filp == NULL) {
        DEBUG_ERROR_LOG("[PROX_open NULL_Error : filp]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
        /* Init = Far */
        atomic_set(&sensor_data, 7);
        nResult = 0;
    }else{
        DEBUG_ERROR_LOG("[PROX_open flag_Error : open_flag ]");
    }

    FUNC_FIN_LOG();
    return nResult;
}


/* ------------------------------------------------------------------------- */
/* PROX_release                                                              */
/* ------------------------------------------------------------------------- */

static int PROX_release(struct inode *inode, struct file *filp)
{
    FUNC_LOG();

    if (inode == NULL) {
        DEBUG_ERROR_LOG("[PROX_release NULL_Error : inode]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    if (filp == NULL) {
        DEBUG_ERROR_LOG("[PROX_release NULL_Error : filp]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    IOECS_Disable();

    atomic_set(&open_flag, 0);

    wake_unlock(&prox_timeout_wake_lock);

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* PROX_read                                                                 */
/* ------------------------------------------------------------------------- */

static int PROX_read(struct file *filp, char __user *buf,
            size_t count, loff_t *ppos)

{
    char tmp;

#ifdef PROXIMITY_DBG_SENSOR
    FUNC_LOG();
#endif /* PROXIMITY_DBG_SENSOR */

    if (filp == NULL) {
        DEBUG_ERROR_LOG("[PROX_read NULL_Error : filp]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    if (buf == NULL) {
        DEBUG_ERROR_LOG("[PROX_read NULL_Error : buf]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }
    
    if (ppos == NULL) {
        DEBUG_ERROR_LOG("[PROX_read NULL_Error : ppos]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    if (prox_dbg_mode == 0) {
	    tmp = (char)atomic_read(&sensor_data);
    }
    else {
        tmp = (char)sensor_data_tmp;
    }

    DEBUG_SENSOR_LOG("PROX_read_SENSOR_DATA = %x",tmp);

    if (copy_to_user(buf, &tmp, sizeof(tmp))) {
        return -EFAULT;
    }

#ifdef PROXIMITY_DBG_SENSOR
    FUNC_FIN_LOG();
#endif /* PROXIMITY_DBG_SENSOR */

    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* PROX_ioctl                                                                */
/* ------------------------------------------------------------------------- */

static long PROX_ioctl(struct file *filp,
                         unsigned int cmd, unsigned long arg)
{
    void __user *argp = (void __user *)arg;
    uint8_t bData;
    short cycle;
    unsigned short lt_threshold;
    unsigned short ht_threshold;
    unsigned short psresult;

    FUNC_LOG();

    if (filp == NULL) {
        DEBUG_ERROR_LOG("[PROX_ioctl NULL_Error : filp]");
        return SH_PROXIMITY_RESULT_FAILURE;
    }

    switch (cmd) {
    case ECS_IOCTL_SET_CYCLE:
        DEBUG_LOG("ECS_IOCTL_SET_CYCLE");
        if (copy_from_user(&cycle, argp, sizeof(cycle))) {
            DEBUG_ERROR_LOG("ECS_IOCTL_SET_CYCLE ERR");
            return -EFAULT;
        }
        break;
    case ECS_IOCTL_LT_THRESHOLD_WRITE:
        DEBUG_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE");
        if (copy_from_user(&lt_threshold, argp, sizeof(lt_threshold))) {
            DEBUG_ERROR_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE ERR");
            return -EFAULT;
        }
        break;
    case ECS_IOCTL_HT_THRESHOLD_WRITE:
        DEBUG_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE");
        if (copy_from_user(&ht_threshold, argp, sizeof(ht_threshold))) {
            DEBUG_ERROR_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE ERR");
            return -EFAULT;
        }
        break;
    case ECS_IOCTL_GET_D2_DATA:
        DEBUG_LOG("ECS_IOCTL_GET_D2_DATA");
        if (copy_from_user(&psresult, argp, sizeof(psresult))) {
            DEBUG_ERROR_LOG("ECS_IOCTL_GET_D2_DATA ERR");
            return -EFAULT;
        }
        break;

    default:
        break;
    }

    switch (cmd) {
    case ECS_IOCTL_ENABLE:
        DEBUG_LOG("ECS_IOCTL_ENABLE");
        if(IOECS_Enable() < 0) {
           DEBUG_ERROR_LOG("ECS_IOCTL_ENABLE ERR");
            return -EIO;
        }
        break;
    case ECS_IOCTL_DISABLE:
        DEBUG_LOG("ECS_IOCTL_DISABLE");
        if(IOECS_Disable() < 0) {
            DEBUG_ERROR_LOG("ECS_IOCTL_DISABLE ERR");
            return -EIO;
        }
        break;
    case ECS_IOCTL_SET_CYCLE:
        DEBUG_LOG("ECS_IOCTL_SET_CYCLE");
        if(IOECS_SetCycle(cycle) < 0) {
           DEBUG_ERROR_LOG("ECS_IOCTL_SET_CYCLE ERR");
            return -EIO;
        }
        break;
    case ECS_IOCTL_GET_VO_DATA:
        DEBUG_LOG("ECS_IOCTL_GET_VO_DATA");
        if(IOECS_GetVO_DATA(&bData) < 0) {
           DEBUG_ERROR_LOG("ECS_IOCTL_GET_VO_DATA ERR");
            return -EIO;
        }
        break;
    case ECS_IOCTL_GET_D2_DATA:
        DEBUG_LOG("ECS_IOCTL_GET_D2_DATA");
        if(IOECS_GetD2_DATA(&psresult) < 0) {
           DEBUG_ERROR_LOG("ECS_IOCTL_GET_D2_DATA ERR");
            return -EIO;
        }
        break;
    case ECS_IOCTL_LT_THRESHOLD_WRITE:
        DEBUG_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE");
        if(IOECS_LT_Thresshold_Write(lt_threshold) < 0) {
           DEBUG_ERROR_LOG("ECS_IOCTL_LT_THRESHOLD_WRITE ERR");
            return -EIO;
        }
        break;
    case ECS_IOCTL_HT_THRESHOLD_WRITE:
        DEBUG_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE");
        if(IOECS_HT_Thresshold_Write(ht_threshold) < 0) {
           DEBUG_ERROR_LOG("ECS_IOCTL_HT_THRESHOLD_WRITE ERR");
            return -EIO;
        }
        break;

    default:
        break;
    }

    switch (cmd) {
    case ECS_IOCTL_GET_VO_DATA:
        DEBUG_LOG("ECS_IOCTL_GET_VO_DATA");
        if (copy_to_user(argp, &bData, sizeof(bData))) {
            DEBUG_ERROR_LOG("ECS_IOCTL_GET_VO_DATA ERR");
            return -EFAULT;
        }
        break;
    case ECS_IOCTL_GET_D2_DATA:
        DEBUG_LOG("ECS_IOCTL_GET_D2_DATA");
        if (copy_to_user(argp, &psresult, sizeof(psresult))) {
            DEBUG_ERROR_LOG("ECS_IOCTL_GET_D2_DATA ERR");
            return -EFAULT;
        }
        break;
    default:
        break;
    }

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* PROX_Irq_workfunc                                                         */
/* ------------------------------------------------------------------------- */

static void PROX_Irq_workfunc( void )
{
    int nResult = 0;
    unsigned char rData = 0x00;

    wake_lock(&prox_wake_lock);

#ifdef PROXIMITY_DBG_SENSOR
    FUNC_LOG();
#endif /* PROXIMITY_DBG_SENSOR */

    nResult = PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);
    
    if(nResult != 0) {
        DEBUG_ERROR_LOG("I2CRead_Error");
    }
    
    rData = (rData & 0x08);

    if(0x08 == rData) {
        atomic_set(&sensor_data, 0);
    }
    else {
        atomic_set(&sensor_data, 7);
    }

    DEBUG_SENSOR_LOG("PROX_Irq_workfunc_SENSOR_DATA = %x\n",atomic_read(&sensor_data));

    if(atomic_read(&sensor_data) == 0x07) {
        wake_lock_timeout(&prox_timeout_wake_lock, 1 * HZ);
    }

    input_report_abs(poProximityRec->input_dev, ABS_DISTANCE, atomic_read(&sensor_data));
    input_sync(poProximityRec->input_dev);

#ifdef PROXIMITY_DBG_SENSOR

    PROX_I2cRead(GP2AP030_REG_D2_LSB,&rData);
    PROX_I2cRead(GP2AP030_REG_D2_MSB,&rData);
    PROX_I2cRead(GP2AP030_REG_LT_LSB,&rData);
    PROX_I2cRead(GP2AP030_REG_LT_MSB,&rData);
    PROX_I2cRead(GP2AP030_REG_HT_LSB,&rData);
    PROX_I2cRead(GP2AP030_REG_HT_MSB,&rData);

#endif /* PROXIMITY_DBG_SENSOR */

#ifdef PROXIMITY_DBG_SENSOR
    FUNC_FIN_LOG();
#endif /* PROXIMITY_DBG_SENSOR */

    wake_unlock(&prox_wake_lock);
}


/* ------------------------------------------------------------------------- */
/* PROX_ConfigGPIO                                                           */
/* ------------------------------------------------------------------------- */

#if 0
static int PROX_ConfigGPIO(void)
{

    FUNC_LOG();

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}
#endif


/* ------------------------------------------------------------------------- */
/* PROX_Initialize                                                           */
/* ------------------------------------------------------------------------- */

static int PROX_Initialize(void)
{
    uint16_t rev;

    FUNC_LOG();

    rev = sh_get_hw_revision();
    rev = rev & 0x07;

#if CONFIG_PROXIMITY_MODEL_TYPE == MODEL_TYPE_D4
    switch(rev){
    case HW_ES0:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_ES05:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_ES1:
        Threshold_Low  = 0x0539;
        Threshold_High = 0x05CF;
        break;
    case HW_PP1:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_PP2:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_PM:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    default:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    }
#else
    switch(rev){
    case HW_ES0:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break; 
    case HW_ES05:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_ES1:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_PP1:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_PP2:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    case HW_PM:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    default:
        Threshold_Low  = 0x012c;
        Threshold_High = 0x0190;
        break;
    }
#endif

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* PROX_Remove                                                               */
/* ------------------------------------------------------------------------- */

static int PROX_Remove(void)
{
    FUNC_LOG();

    input_free_device(poProximityRec->input_dev);
    kfree(poProximityRec);

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;
}


/* ------------------------------------------------------------------------- */
/* PROX_Probe                                                                */
/* ------------------------------------------------------------------------- */

static int PROX_Probe(void)
{
    int nResult = 0;

    FUNC_LOG();

    /* Allocate memory for driver data */
    poProximityRec = kzalloc(sizeof(drv_data), GFP_KERNEL);

    if (!poProximityRec) {
        DEBUG_ERROR_LOG("memory allocation failed.");
        nResult = -ENOMEM;
        goto memory_error;
    }

    nResult = PROX_Initialize();

    if (nResult < 0) {
        DEBUG_ERROR_LOG("initialize failed.");
        goto initialize_error;
    }

    nResult = misc_register(&PROX_device);

    if (nResult) {
        DEBUG_ERROR_LOG("misc_register failed.");
        goto misc_register_error;
    }

    /* Declare input device */
    poProximityRec->input_dev = input_allocate_device();

    if (!poProximityRec->input_dev) {
        nResult = -ENOMEM;
        DEBUG_ERROR_LOG("Failed to allocate input device.");
        goto input_dev_error;
    }

    /* Setup input device */
    set_bit(EV_ABS, poProximityRec->input_dev->evbit);

    /* proximity value near=7, far=0 */
    input_set_abs_params(poProximityRec->input_dev, ABS_DISTANCE, 0, 7, 0, 0);

    /* Set name */
    poProximityRec->input_dev->name = "proximity";

    /* Register */
    nResult = input_register_device(poProximityRec->input_dev);

    if (nResult) {
        DEBUG_ERROR_LOG("Unable to register input device.");
        goto input_register_error;
    }

    FUNC_FIN_LOG();
    return SH_PROXIMITY_RESULT_SUCCESS;

input_register_error:
    input_free_device(poProximityRec->input_dev);
input_dev_error:
    misc_deregister(&PROX_device);
misc_register_error:
initialize_error:
    kfree(poProximityRec);
memory_error:
    return nResult;
}


#if defined (CONFIG_ANDROID_ENGINEERING)
static int proximity_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
#define LEN_ID    (2)
#define LEN_PARAM (0)
#define PARAM_MAX (0)

   unsigned long len = count;
   int proximity_id;
   char buf[LEN_ID + 1];
   char kbuf[LEN_ID + PARAM_MAX * LEN_PARAM];
   unsigned int threshold = 0x00;
   unsigned char rData = 0x00;

    len--;
    /* Check length */
    if (len < LEN_ID){
        return count;
    }
    if (len > (LEN_ID + PARAM_MAX * LEN_PARAM)){
        len = LEN_ID + PARAM_MAX * LEN_PARAM;
    }

    if (copy_from_user(kbuf, buffer, len)){
        return -EFAULT;
    }

    /* Get FunctionID */
    memcpy(buf, kbuf, LEN_ID);
    buf[LEN_ID] = '\0';
    proximity_id = simple_strtol(buf, NULL, 10);

    printk("[PROXIMITY] proximity_proc_write ID = %02X\n", proximity_id);

    switch (proximity_id) {
        /* Show all register values */
        case 0:
            PROX_I2cRead(GP2AP030_REG_COMMAND1, &rData);  /* 0x00 */
            PROX_I2cRead(GP2AP030_REG_COMMAND2, &rData);  /* 0x01 */
            PROX_I2cRead(GP2AP030_REG_COMMAND3, &rData);  /* 0x02 */
            PROX_I2cRead(GP2AP030_REG_COMMAND4, &rData);  /* 0x03 */
            PROX_I2cRead(GP2AP030_REG_LT_LSB, &rData);    /* 0x08 */
            PROX_I2cRead(GP2AP030_REG_LT_MSB, &rData);    /* 0x09 */
            PROX_I2cRead(GP2AP030_REG_HT_LSB, &rData);    /* 0x0A */
            PROX_I2cRead(GP2AP030_REG_HT_MSB, &rData);    /* 0x0B */
            PROX_I2cRead(GP2AP030_REG_D2_LSB, &rData);    /* 0x10 */
            PROX_I2cRead(GP2AP030_REG_D2_MSB, &rData);    /* 0x11 */
            break;
        /* Show threadshold */
        case 1:
            if (PROX_I2cRead(GP2AP030_REG_LT_MSB, &rData) == 0) {
                threshold = (unsigned int)rData;
                threshold <<= 8;
                if (PROX_I2cRead(GP2AP030_REG_LT_LSB, &rData) == 0) {
                    threshold += (unsigned int)rData;
                    printk("[PROXIMITY] Threshold low = %04X\n", threshold);
                }
            }
            if (PROX_I2cRead(GP2AP030_REG_HT_MSB, &rData) == 0) {
                threshold = (unsigned int)rData;
                threshold <<= 8;
                if (PROX_I2cRead(GP2AP030_REG_HT_LSB, &rData) == 0) {
                    threshold += (unsigned int)rData;
                    printk("[PROXIMITY] Threshold high = %04X\n", threshold);
                }
            }
            break;
        default:
            break;
    }

    return count;
}
#endif /* CONFIG_ANDROID_ENGINEERING */


/* ------------------------------------------------------------------------- */
/* PROX_Init                                                                 */
/* ------------------------------------------------------------------------- */

static int __init PROX_Init(void)
{
#if defined (CONFIG_ANDROID_ENGINEERING)
    struct proc_dir_entry *entry;
#endif /* CONFIG_ANDROID_ENGINEERING */

    FUNC_LOG();

#if defined (CONFIG_ANDROID_ENGINEERING)
    entry = create_proc_entry("driver/PROXIMITY", 0666, NULL);

    if (entry == NULL) {
        goto proc_fs_error;
    }

    entry->write_proc = proximity_proc_write;

proc_fs_error:
#endif /* CONFIG_ANDROID_ENGINEERING */

    wake_lock_init(&prox_timeout_wake_lock, WAKE_LOCK_SUSPEND, "prox_timeout_wake_lock");
    wake_lock_init(&prox_wake_lock, WAKE_LOCK_SUSPEND, "proximity_wake_lock");

    FUNC_FIN_LOG();
    return PROX_Probe();
}


/* ------------------------------------------------------------------------- */
/* PROX_Exit                                                                 */
/* ------------------------------------------------------------------------- */

static void __exit PROX_Exit(void)
{
    FUNC_LOG();

    wake_unlock(&prox_wake_lock);
    wake_lock_destroy(&prox_wake_lock);
    wake_unlock(&prox_timeout_wake_lock);
    wake_lock_destroy(&prox_timeout_wake_lock);

    FUNC_FIN_LOG();
    PROX_Remove();
}


module_init(PROX_Init);
module_exit(PROX_Exit);

MODULE_DESCRIPTION("proximity sensor driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.0");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
