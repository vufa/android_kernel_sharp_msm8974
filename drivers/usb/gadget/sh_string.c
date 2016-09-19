/* drivers/usb/gadget/sh_string.c
 *
 * sh_string.c - Gadget Driver for Android USB string descriptor
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

#define USB_STRING_DESC_LEN		32
static char manufacturer_string[USB_STRING_DESC_LEN];
static char product_string[USB_STRING_DESC_LEN];
static char serial_string[USB_STRING_DESC_LEN];

#define USB_I_INT_STRING_DESC_LEN	64

#ifdef CONFIG_USB_ANDROID_SH_SERIALS
static char obex_iInterface[USB_I_INT_STRING_DESC_LEN];
static char mdlm_iInterface[USB_I_INT_STRING_DESC_LEN];
static char acm_iInterface[USB_I_INT_STRING_DESC_LEN];
#endif /* CONFIG_USB_ANDROID_SH_SERIALS */

#ifdef CONFIG_USB_ANDROID_SH_UMS
static char msc_iInterface[USB_I_INT_STRING_DESC_LEN];
#endif /* CONFIG_USB_ANDROID_SH_UMS */

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
static char cd_iInterface[USB_I_INT_STRING_DESC_LEN];
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */

#define DESCRIPTOR_STRING_ATTR(field, buffer, len)			\
static ssize_t								\
field ## _show(struct device *dev, struct device_attribute *attr,	\
		char *buf)						\
{									\
	return snprintf(buf, PAGE_SIZE, "%s\n", buffer);		\
}									\
static ssize_t								\
field ## _store(struct device *dev, struct device_attribute *attr,	\
		const char *buf, size_t size)				\
{									\
	char *line_feed;						\
	if ((size == 0) || (buf == NULL)) return -EINVAL;		\
	if ((size == 1) && (buf[0] == 0x0a)) return -EINVAL;		\
	if (size > len) {						\
		pr_err("%s:over input length\n", __func__);		\
		return -EINVAL;						\
	}								\
	strncpy(buffer, buf, len);					\
	buffer[len-1] = 0x00;						\
	line_feed = strstr(buffer, "\n");				\
	if(line_feed) *line_feed = 0x00;				\
	return size;							\
}									\
static DEVICE_ATTR(field, S_IRUGO | S_IWUSR, field ## _show, field ## _store);

DESCRIPTOR_STRING_ATTR(iManufacturer, manufacturer_string, USB_STRING_DESC_LEN)
DESCRIPTOR_STRING_ATTR(iProduct, product_string, USB_STRING_DESC_LEN)
DESCRIPTOR_STRING_ATTR(iSerial, serial_string, USB_STRING_DESC_LEN)

#ifdef CONFIG_USB_ANDROID_SH_SERIALS
DESCRIPTOR_STRING_ATTR(obex_iInterface, obex_iInterface, USB_I_INT_STRING_DESC_LEN)
DESCRIPTOR_STRING_ATTR(mdlm_iInterface, mdlm_iInterface, USB_I_INT_STRING_DESC_LEN)
DESCRIPTOR_STRING_ATTR(acm_iInterface, acm_iInterface, USB_I_INT_STRING_DESC_LEN)
#endif /* CONFIG_USB_ANDROID_SH_SERIALS */

#ifdef CONFIG_USB_ANDROID_SH_UMS
DESCRIPTOR_STRING_ATTR(msc_iInterface, msc_iInterface, USB_I_INT_STRING_DESC_LEN)
#endif /* CONFIG_USB_ANDROID_SH_UMS */

#ifdef CONFIG_USB_ANDROID_MASS_STORAGE_CD
DESCRIPTOR_STRING_ATTR(cd_iInterface, cd_iInterface, USB_I_INT_STRING_DESC_LEN)
#endif /* CONFIG_USB_ANDROID_MASS_STORAGE_CD */

/* Module */
MODULE_AUTHOR("SHARP CORPORATION");
MODULE_LICENSE("GPL");
