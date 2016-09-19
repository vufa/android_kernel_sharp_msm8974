/* drivers/sharp/shdisp/shdisp_clmr_usr.c  (Display Driver)
 *
 * Copyright (C) 2012 SHARP CORPORATION
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

/*---------------------------------------------------------------------------*/
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/* INCLUDE FILES                                                             */
/*---------------------------------------------------------------------------*/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <sharp/shdisp_kerl.h>

#include "shdisp_system.h"
#include "shdisp_clmr.h"
#include "shdisp_panel.h"
#include "shdisp_dbg.h"
#include "shdisp_pm.h"
/*---------------------------------------------------------------------------*/
/* MACROS                                                                    */
/*---------------------------------------------------------------------------*/
#define SHDISP_CLMR_DEVCODE_VALUE               0x00004002

/*---------------------------------------------------------------------------*/
/* DEBUG MACROS                                                              */
/*---------------------------------------------------------------------------*/


/* ------------------------------------------------------------------------- */
/* VARIABLES                                                                 */
/* ------------------------------------------------------------------------- */
static struct shdisp_clmr_ewb  clmr_ewb;


/*---------------------------------------------------------------------------*/
/* PROTOTYPES                                                                */
/*---------------------------------------------------------------------------*/
extern int shdisp_check_clmr_exist(void);
extern int shdisp_check_initialized(void);

/*---------------------------------------------------------------------------*/
/* FUNCTIONS                                                                 */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_write_reg                                              */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_write_reg(void __user *argp)
{
    int ret;
    int size = sizeof(char) * 4;
    struct shdisp_diag_lcdc_reg data;
    union regDat_t {
        unsigned long lDat;
        unsigned char cDat[size];
    } sRegDat;

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if( shdisp_SYS_FWCMD_istimeoutexception() ){
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }

    ret = copy_from_user(&data, argp, sizeof(struct shdisp_diag_lcdc_reg));
    if(0 == ret) {
        sRegDat.lDat = htonl(data.value);
        ret = shdisp_SYS_clmr_sio_transfer(data.reg,
                                &sRegDat.cDat[0], size, NULL, 0);
        if (ret != SHDISP_RESULT_SUCCESS) {
            pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
            ret = -1;
        }
    }
    else {
        pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_read_reg                                               */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_read_reg(void __user *argp)
{
    int ret;
    int size = sizeof(char) * 4;
    struct shdisp_diag_lcdc_reg data;
    union regDat_t {
        unsigned long lDat;
        unsigned char cDat[size];
    } sRegDat;

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if( shdisp_SYS_FWCMD_istimeoutexception() ){
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }

    ret = copy_from_user(&data, argp, sizeof(struct shdisp_diag_lcdc_reg));
    if(0 == ret) {
        ret = shdisp_SYS_clmr_sio_transfer(data.reg,
                                NULL, 0, &sRegDat.cDat[0], size);
        if (ret != SHDISP_RESULT_SUCCESS) {
            pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
            ret = -1;
        }
        else {
            data.value = ntohl(sRegDat.lDat);

            ret = copy_to_user(argp, &data, sizeof(struct shdisp_diag_lcdc_reg));
            if(0 != ret) {
                pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
                ret = -1;
            }
        }
    }
    else {
        pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_devchk                                                 */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_devchk(void)
{
    int ret = 0;
    int size = sizeof(char) * 4;
    unsigned long regVal;
    union data_t {
        unsigned long lDat;
        unsigned char cDat[size];
    } sData;

    ret = shdisp_SYS_clmr_sio_transfer(SHDISP_CLMR_REG_DEVCODE,
                            NULL, 0, &sData.cDat[0], size);
    if (ret != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }
    else {
        regVal = ntohl(sData.lDat);
        if(SHDISP_CLMR_DEVCODE_VALUE != regVal) {
            pr_err("%s():[%d] error. devcode = 0x%04x\n",
                            __func__, __LINE__, (int)regVal);
            ret = -1;
        }
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_i2c_write                                              */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_i2c_write(void __user *argp)
{
    int ret;
    struct shdisp_diag_lcdc_i2c lcdc_i2c;

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if( shdisp_SYS_FWCMD_istimeoutexception() ){
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }

    ret = copy_from_user(&lcdc_i2c, argp, sizeof(struct shdisp_diag_lcdc_i2c));

    if (lcdc_i2c.size > 256) {
        lcdc_i2c.size = 256;
    }
    
    if (ret == 0) {
        shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
        shdisp_FWCMD_buf_add_multi(SHDISP_CLMR_FWCMD_I2C_MULTI_WRITE, (lcdc_i2c.slave_addr << 1), 0x00, lcdc_i2c.size, lcdc_i2c.buf);
        shdisp_FWCMD_buf_finish();
        ret = shdisp_FWCMD_doKick(1, 0, NULL);
        if (ret != SHDISP_RESULT_SUCCESS) {
            pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
            ret = -1;
        }
    }
    else {
        pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_i2c_read                                               */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_i2c_read(void __user *argp)
{
    int ret;
    int i = 0;
    struct shdisp_diag_lcdc_i2c lcdc_i2c;
    char ary[2] = {0};
    unsigned char rtnbuf[4] = {0};

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if( shdisp_SYS_FWCMD_istimeoutexception() ){
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }

    ret = copy_from_user(&lcdc_i2c, argp, sizeof(struct shdisp_diag_lcdc_i2c));

    if (lcdc_i2c.size > 256) {
        lcdc_i2c.size = 256;
    }

    if (ret == 0) {

        ary[0] = lcdc_i2c.slave_addr << 1;

        for (i = 0; i < lcdc_i2c.size; i++) {
            shdisp_FWCMD_buf_init(SHDISP_CLMR_FWCMD_APINO_PROXSENSOR);
            shdisp_FWCMD_buf_add(SHDISP_CLMR_FWCMD_I2C_1BYTE_READ, 2, ary);
            shdisp_FWCMD_buf_finish();
            ret = shdisp_FWCMD_doKick(1, 4, rtnbuf);
            if (ret != SHDISP_RESULT_SUCCESS) {
                pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
                break;
            }
            lcdc_i2c.buf[i] = rtnbuf[0];
            SHDISP_DEBUG("[SHDISP]%s addr=0x%02x, buf=0x%02x\n", __func__, ary[1], lcdc_i2c.buf[i]);
            ary[1]++;
        }

        if (ret == SHDISP_RESULT_SUCCESS) {
            ret = copy_to_user(argp, &lcdc_i2c, sizeof(struct shdisp_diag_lcdc_i2c));
            if (ret != 0) {
                pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
                ret = -1;
            }
        }
    }
    else {
        pr_err("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_ewb_tbl                                            */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_ewb_tbl(void __user *argp)
{
    int ret;
    struct shdisp_diag_ewb_tbl diag_ewb_tbl;
    struct shdisp_clmr_ewb_accu *ewb_accu;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_SYS_FWCMD_istimeoutexception()) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    ret = copy_from_user(&diag_ewb_tbl, argp, sizeof(struct shdisp_diag_ewb_tbl));
    if(0 == ret) {
        memcpy(clmr_ewb.valR, diag_ewb_tbl.valR, SHDISP_LCDC_EWB_TBL_SIZE);
        memcpy(clmr_ewb.valG, diag_ewb_tbl.valG, SHDISP_LCDC_EWB_TBL_SIZE);
        memcpy(clmr_ewb.valB, diag_ewb_tbl.valB, SHDISP_LCDC_EWB_TBL_SIZE);
        ewb_accu = shdisp_clmr_api_get_ewb_accu(SHDISP_CLMR_EWB_LUT_NO_0);
        shdisp_clmr_api_convert_ewb_param(&clmr_ewb, ewb_accu);
        ret = shdisp_clmr_api_set_ewb_tbl(ewb_accu, SHDISP_CLMR_EWB_LUT_NO_0);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_ewb                                                */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_ewb(void __user *argp)
{
    int ret;
    struct shdisp_diag_set_ewb ewb;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_SYS_FWCMD_istimeoutexception()) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    ret = copy_from_user(&ewb, argp, sizeof(struct shdisp_diag_set_ewb));
    if(0 == ret) {
        ret = shdisp_clmr_api_diag_set_ewb(&ewb);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_read_ewb                                               */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_read_ewb(void __user *argp)
{
    int ret;
    struct shdisp_diag_read_ewb rewb;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_SYS_FWCMD_istimeoutexception()) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    ret = copy_from_user(&rewb, argp, sizeof(struct shdisp_diag_read_ewb));
    if(0 == ret) {
        ret = shdisp_clmr_api_diag_read_ewb(&rewb);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
        else {
            ret = copy_to_user(argp,
                              &rewb,
                              sizeof(struct shdisp_diag_read_ewb));
            if(0 != ret) {
                SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
                ret = -1;
            }
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_ewb_tbl                                            */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_ewb_tbl2(void __user *argp)
{
    int ret;
    struct shdisp_diag_ewb_tbl diag_ewb_tbl;
    struct shdisp_clmr_ewb_accu *ewb_accu;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_SYS_FWCMD_istimeoutexception()) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    ret = copy_from_user(&diag_ewb_tbl, argp, sizeof(struct shdisp_diag_ewb_tbl));
    if(0 == ret) {
        memcpy(clmr_ewb.valR, diag_ewb_tbl.valR, SHDISP_LCDC_EWB_TBL_SIZE);
        memcpy(clmr_ewb.valG, diag_ewb_tbl.valG, SHDISP_LCDC_EWB_TBL_SIZE);
        memcpy(clmr_ewb.valB, diag_ewb_tbl.valB, SHDISP_LCDC_EWB_TBL_SIZE);
        ewb_accu = shdisp_clmr_api_get_ewb_accu(SHDISP_CLMR_EWB_LUT_NO_1);
        if(ewb_accu == NULL) {
            SHDISP_ERR("[%d] error. ewb_accu == NULL", __LINE__);
            return -1;
        }
        shdisp_clmr_api_convert_ewb_param(&clmr_ewb, ewb_accu);
        ret = shdisp_clmr_api_set_ewb_tbl(ewb_accu, SHDISP_CLMR_EWB_LUT_NO_1);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_pic_adj_param                                      */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_pic_adj_param(void __user *argp)
{
    int ret;
    struct shdisp_main_pic_adj pic_adj;

    ret = copy_from_user(&pic_adj, argp, sizeof(struct shdisp_main_pic_adj));
    if(0 == ret) {
        ret = shdisp_clmr_api_set_pic_adj_param(&pic_adj);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_trv_param                                          */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_trv_param(void __user *argp)
{
    int ret;
    struct shdisp_trv_param trv_param;
    struct shdisp_trv_data_header trv_data_header;
    struct shdisp_clmr_trv_info *trv_info;
    unsigned int align_size = 0;

    ret = copy_from_user(&trv_param, argp, sizeof(struct shdisp_trv_param));
    if(0 == ret) {
        if ((trv_param.request == SHDISP_LCDC_TRV_REQ_SET_PARAM)
         && (trv_param.data != NULL)) {
            ret = copy_from_user(&trv_data_header,
                                 trv_param.data,
                                 sizeof(struct shdisp_trv_data_header));
            if (ret == 0) {
                SHDISP_DEBUG("data_size = %d\n", trv_data_header.data_size);
                if (trv_data_header.data_size != 0) {
                    if (trv_data_header.data_size > SHDISP_TRV_DATA_MAX) {
                        SHDISP_ERR("TRV DATA MAX SIZE OVER");
                        return -1;
                    }
                    trv_info = shdisp_clmr_api_get_trv_info();
                    ret = copy_from_user(trv_info->data,
                                         trv_param.data + sizeof(struct shdisp_trv_data_header),
                                         trv_data_header.data_size);
                    if (ret == 0) {
                        align_size = ((trv_data_header.data_size | 0x0000000F) & ~0x0000000F);
                        trv_info->data_size = align_size;
                        trv_info->hw        = trv_data_header.hw;
                        trv_info->y_size    = trv_data_header.y_size;
                    }
                }
            }
        }
        ret = shdisp_clmr_api_set_trv_param(&trv_param);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_dbc_param                                          */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_dbc_param(void __user *argp)
{
    int ret;
    struct shdisp_main_dbc dbc;

    ret = copy_from_user(&dbc, argp, sizeof(struct shdisp_main_dbc));
    if(0 == ret) {
        ret = shdisp_clmr_api_set_dbc_param(&dbc);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_ae_param                                           */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_ae_param(void __user *argp)
{
    int ret;
    struct shdisp_main_ae ae;

    ret = copy_from_user(&ae, argp, sizeof(struct shdisp_main_ae));
    if(0 == ret) {
        ret = shdisp_clmr_api_set_ae_param(&ae);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}

/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_pic_adj_ap_type                                    */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_pic_adj_ap_type(void __user *argp)
{
    int ret;
    unsigned short ap_type;

    ret = copy_from_user(&ap_type, argp, sizeof(unsigned short));
    if(0 == ret) {
        ret = shdisp_clmr_api_set_pic_adj_ap_type(ap_type);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_drive_freq                                            */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_drive_freq(void __user *argp)
{
    int ret;
    int type;

    ret = copy_from_user(&type, argp, sizeof(int));
    if(0 == ret) {
        if (type == SHDISP_MAIN_DISP_DRIVE_FREQ_DEFAULT) {
            type = SHDISP_MAIN_DISP_DRIVE_FREQ_TYPE_A;
        }
        ret = shdisp_panel_API_set_drive_freq(type);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_set_flicker_trv                                        */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_set_flicker_trv(void __user *argp)
{
    int ret;
    struct shdisp_flicker_trv flicker_trv;

    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    if (shdisp_SYS_FWCMD_istimeoutexception()) {
        SHDISP_ERR("%s():[%d] error. \n", __func__, __LINE__);
        return -1;
    }

    ret = copy_from_user(&flicker_trv, argp, sizeof(struct shdisp_flicker_trv));
    if(0 == ret) {
        ret = shdisp_clmr_api_set_flicker_trv(&flicker_trv);
        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("[%d] error. ret = %d\n", __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR(":[%d] error. ret = %d\n", __LINE__, ret);
        ret = -1;
    }

    return ret;
}



/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_fw_cmd_write                                              */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_fw_cmd_write(void __user *argp)
{
    int ret;
    struct shdisp_diag_fw_cmd fw_cmd;

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if( shdisp_SYS_FWCMD_istimeoutexception() ){
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }

    ret = copy_from_user(&fw_cmd, argp, sizeof(struct shdisp_diag_fw_cmd));

    SHDISP_DEBUG("[SHDISP]%s cmd=0x%02x, write_count=0x%02x\n", __func__,fw_cmd.cmd, fw_cmd.write_count);
    SHDISP_DEBUG("[SHDISP] fw_cmd.write_val[0-3]=%02x%02x%02x%02x\n", fw_cmd.write_val[0], fw_cmd.write_val[1], fw_cmd.write_val[2], fw_cmd.write_val[3]);

    if (fw_cmd.write_count > 512) {
        fw_cmd.write_count = 512;
    }
    if (fw_cmd.read_count > 16) {
        fw_cmd.read_count = 16;
    }

    if (ret == 0) {
        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add(fw_cmd.cmd, fw_cmd.write_count, fw_cmd.write_val);
        shdisp_FWCMD_buf_finish();
        ret = shdisp_FWCMD_doKick(1, 0, NULL);

        if (ret != SHDISP_RESULT_SUCCESS) {
            SHDISP_ERR("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }

    return ret;
}


/*---------------------------------------------------------------------------*/
/*  shdisp_ioctl_lcdc_fw_cmd_read                                            */
/*---------------------------------------------------------------------------*/
int shdisp_ioctl_lcdc_fw_cmd_read(void __user *argp)
{
    int ret;
    struct shdisp_diag_fw_cmd fw_cmd;

    if (shdisp_check_clmr_exist() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_check_initialized() != SHDISP_RESULT_SUCCESS) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if (shdisp_pm_is_clmr_on() != SHDISP_DEV_STATE_ON) {
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }
    if( shdisp_SYS_FWCMD_istimeoutexception() ){
        pr_err("%s():[%d] error. \n", __func__, __LINE__);
        return SHDISP_RESULT_FAILURE;
    }

    ret = copy_from_user(&fw_cmd, argp, sizeof(struct shdisp_diag_fw_cmd));

    SHDISP_DEBUG("[SHDISP]%s cmd=0x%02x, write_count=0x%02x\n", __func__,fw_cmd.cmd, fw_cmd.write_count);
    SHDISP_DEBUG("[SHDISP] fw_cmd.write_val[0-3]=%02x%02x%02x%02x\n", fw_cmd.write_val[0], fw_cmd.write_val[1], fw_cmd.write_val[2], fw_cmd.write_val[3]);

    if (fw_cmd.write_count > 512) {
        fw_cmd.write_count = 512;
    }
    if (fw_cmd.read_count > 16) {
        fw_cmd.read_count = 16;
    }

    if (ret == 0) {
        switch(fw_cmd.cmd) {
        case SHDISP_CLMR_FWCMD_I2C_1BYTE_READ:
            fw_cmd.read_count = 1;
            break;
        case SHDISP_CLMR_FWCMD_LIGHTCTL_READ:
            fw_cmd.read_count = 2;
            break;
        case SHDISP_CLMR_FWCMD_LIGHTCTL_LUXPARAM_READ:
            if(fw_cmd.write_val[0] == 0x00){
                fw_cmd.read_count = 2;
            }
            else if (fw_cmd.write_val[0] == 0x01){
                fw_cmd.read_count = 2;
            }
            else if (fw_cmd.write_val[0] == 0x02){
                fw_cmd.read_count = 2;
            }
            else if (fw_cmd.write_val[0] == 0x03){
                fw_cmd.read_count = 2;
            }
            else if (fw_cmd.write_val[0] == 0x04){
                fw_cmd.read_count = 6;
            }
            else if (fw_cmd.write_val[0] == 0x05){
                fw_cmd.read_count = 4;
            }
            else if (fw_cmd.write_val[0] == 0x06){
                fw_cmd.read_count = 3;
            }
            else {
                fw_cmd.read_count = 8;
            }
            break;
        case SHDISP_CLMR_FWCMD_ALS_1BYTE_READ:
            fw_cmd.read_count = 1;
            break;
        case SHDISP_CLMR_FWCMD_MIPI_TEST_RESULT_READ:
            fw_cmd.read_count = 16;
            break;
        default:
            fw_cmd.read_count = 8;
            break;
        }

        shdisp_FWCMD_buf_init(0);
        shdisp_FWCMD_buf_add(fw_cmd.cmd, fw_cmd.write_count, fw_cmd.write_val);
        shdisp_FWCMD_buf_finish();
        ret = shdisp_FWCMD_doKick(1, fw_cmd.read_count, fw_cmd.read_val);

        if (ret == SHDISP_RESULT_SUCCESS) {
            ret = copy_to_user(argp, &fw_cmd, sizeof(struct shdisp_diag_fw_cmd));
            if (ret != 0) {
                SHDISP_ERR("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
                ret = -1;
            }
        }
        else {
            SHDISP_ERR("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
            ret = -1;
        }
    }
    else {
        SHDISP_ERR("%s():[%d] error. ret = %d\n", __func__, __LINE__, ret);
        ret = -1;
    }

    SHDISP_DEBUG("[SHDISP]%s cmd=0x%02x, read_count=0x%02x\n", __func__,fw_cmd.cmd, fw_cmd.read_count);
    SHDISP_DEBUG("[SHDISP] fw_cmd.read_val[0-3]=%02x%02x%02x%02x\n", fw_cmd.read_val[0], fw_cmd.read_val[1], fw_cmd.read_val[2], fw_cmd.read_val[3]);
    SHDISP_DEBUG("[SHDISP] fw_cmd.read_val[4-7]=%02x%02x%02x%02x\n", fw_cmd.read_val[4], fw_cmd.read_val[5], fw_cmd.read_val[6], fw_cmd.read_val[7]);

    return ret;
}



MODULE_DESCRIPTION("SHARP DISPLAY DRIVER MODULE");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_VERSION("1.00");

/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
