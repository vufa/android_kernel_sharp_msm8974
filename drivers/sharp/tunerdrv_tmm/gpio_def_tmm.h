/**************************************************************************************************/
/** 
	@file		gpio_def.h
	@brief		GPIO Definition Header
*/
/**************************************************************************************************/

#ifndef GPIO_DEF
	#define GPIO_DEF

/* LINUX/android/kernel/arch/arm/mach-msm/board-msm8960.h */
#include <mach/gpio.h>
#define PM8841_GPIO_BASE		1000
#define PM8841_GPIO_PM_TO_SYS(pm_gpio)  (pm_gpio + PM8841_GPIO_BASE)

#include "gpio_def_ext_tmm.h"

typedef struct GPIO_DEF {
	unsigned int id;		/* GPIO Number (ID) */
	unsigned int no;		/* GPIO PORT Number */
	int direction;			/* I/O Direction */
	int out_val;			/* Initialized Value */
	int init_done;			/* GPIO Initialized ? 1:Complete (Don't Care) */
	char *label;			/* labels may be useful for diagnostics */
} stGPIO_DEF;

#define DirctionIn (0)
#define DirctionOut (1)

#define GPIO_DTVEN_PORTNO			(92)
#define GPIO_DTVRST_PORTNO			(13)
/* #define GPIO_DTVLNAEN_PORTNO		(xx) */
/* #define GPIO_DTVANTSW_PORTNO		(PM8841_GPIO_PM_TO_SYS(15)) */
#define GPIO_DTVMANTSL_PORTNO		(PM8841_GPIO_PM_TO_SYS(1))
#define GPIO_DTVUANTSL_PORTNO		(PM8841_GPIO_PM_TO_SYS(2))
/* #define GPIO_DTVCANTSL_PORTNO	(PM8841_GPIO_PM_TO_SYS(28)) */

//#define CLOCK_CONTROL_PORTNO		(37)

#endif	//GPIO_DEF
