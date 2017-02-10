/* include/sharp/shdisp_boot_context.h  (Display Driver)
 *
 * Copyright (C) 2011-2012 SHARP CORPORATION
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

struct shdisp_boot_context {
    int driver_is_initialized;
    unsigned short hw_handset;
    unsigned short hw_revision;
    unsigned char device_code;
    int handset_color;
    int upper_unit_is_connected;
    int main_disp_status;
    struct shdisp_main_bkl_ctl main_bkl;
    struct shdisp_tri_led tri_led;
    unsigned short alpha;
    unsigned short alpha_low;
    unsigned short alpha_nvram;
    struct shdisp_photo_sensor_adj photo_sensor_adj;
    struct shdisp_lcddr_phy_gamma_reg lcddr_phy_gamma;
    struct shdisp_ledc_status ledc_status;
    struct shdisp_clmr_status clmr_status;
    struct shdisp_bdic_status bdic_status;
    struct shdisp_clmr_ewb clmr_ewb[2];
    unsigned short slave_alpha;
    unsigned short slave_alpha_low;
    unsigned char pll_on_ctl_count;
    char err_on[SHDISP_NOOS_RESET_NUM];
    struct shdisp_dbg_error_code err_code[SHDISP_NOOS_RESET_NUM];
#if 0
    unsigned char ram_dump_avail;
    unsigned char edram_dump_buf[SHDISP_CALI_EDRAM_DUMP_SIZE];
    unsigned char sram_dump_buf[SHDISP_CALI_SRAM_DUMP_SIZE];
#endif
};
