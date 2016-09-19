#ifndef SHBIF_H__
#define SHBIF_H__
#include <linux/err.h>
//#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
struct file;

int shbif_handle_bif_access_command( struct file* fi_p, unsigned int cmd, unsigned long arg );
bool shbif_cmd_is_bif_command(unsigned int cmd);
int shbif_initialize(void);

#endif /*SHBIF_H__*/
