/* drivers/sharp/shdisp/shdisp_bdic.h  (Display Driver)
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

/* ------------------------------------------------------------------------- */
/* SHARP DISPLAY DRIVER FOR KERNEL MODE                                      */
/* ------------------------------------------------------------------------- */

#ifndef SHDISP_BDIC_H
#define SHDISP_BDIC_H

#if defined(CONFIG_SHDISP_PANEL_GEMINI)
  #include    "shdisp_lm3533.h"
#else   /* CONFIG_SHDISP_PANEL_GEMINI */
  #include    "shdisp_bl69y6.h"
#endif  /* CONFIG_SHDISP_PANEL_GEMINI */

#ifdef CONFIG_SHLCDC_LED_BD2802GU
  #include    "shdisp_bd2802gu.h"
#endif  /* CONFIG_SHLCDC_LED_BD2802GU */

#endif /* SHDISP_BDIC_H */
/* ------------------------------------------------------------------------- */
/* END OF FILE                                                               */
/* ------------------------------------------------------------------------- */
