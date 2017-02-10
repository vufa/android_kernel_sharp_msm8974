/* include/sharp/shtps_sy3000_tm2858-001.h
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
#ifndef __SHTPS_SY3000_TM2858_001_H__
#define __SHTPS_SY3000_TM2858_001_H__

/* -----------------------------------------------------------------------------------
 */
#define SH_TOUCH_DEVNAME		"shtps_rmi"
#define SH_TOUCH_IF_DEVNAME 	"shtpsif"
#define SH_TOUCH_IF_DEVPATH 	"/dev/shtpsif"

#define SHTPS_TM_TXNUM_MAX		28
#define SHTPS_TM_RXNUM_MAX		12
#define SHTPS_FINGER_MAX		5

#define TPS_IOC_MAGIC					0xE0

#define TPSDEV_ENABLE					_IO  ( TPS_IOC_MAGIC,  1)
#define TPSDEV_DISABLE					_IO  ( TPS_IOC_MAGIC,  2)
#define TPSDEV_RESET					_IO  ( TPS_IOC_MAGIC,  3)
#define TPSDEV_SOFT_RESET				_IO  ( TPS_IOC_MAGIC,  4)
#define TPSDEV_GET_FW_VERSION			_IOR ( TPS_IOC_MAGIC,  5, unsigned short)
#define TPSDEV_ENTER_BOOTLOADER			_IOR ( TPS_IOC_MAGIC,  6, struct shtps_bootloader_info)
#define TPSDEV_LOCKDOWN_BOOTLOADER		_IOW ( TPS_IOC_MAGIC,  7, struct shtps_ioctl_param)
#define TPSDEV_ERASE_FLASE				_IO  ( TPS_IOC_MAGIC,  8)
#define TPSDEV_WRITE_IMAGE				_IOW ( TPS_IOC_MAGIC,  9, struct shtps_ioctl_param)
#define TPSDEV_WRITE_CONFIG				_IOW ( TPS_IOC_MAGIC, 10, struct shtps_ioctl_param)
#define TPSDEV_GET_TOUCHINFO			_IOR ( TPS_IOC_MAGIC, 11, struct shtps_touch_info)
#define TPSDEV_GET_TOUCHINFO_UNTRANS	_IOR ( TPS_IOC_MAGIC, 12, struct shtps_touch_info)
#define TPSDEV_SET_TOUCHMONITOR_MODE	_IOW ( TPS_IOC_MAGIC, 13, unsigned char)
#define TPSDEV_READ_REG					_IOWR( TPS_IOC_MAGIC, 14, struct shtps_ioctl_param)
#define TPSDEV_READ_ALL_REG				_IOR ( TPS_IOC_MAGIC, 15, struct shtps_ioctl_param)
#define TPSDEV_WRITE_REG				_IOW ( TPS_IOC_MAGIC, 16, struct shtps_ioctl_param)
#define TPSDEV_START_TM					_IOW ( TPS_IOC_MAGIC, 17, struct shtps_ioctl_param)
#define TPSDEV_STOP_TM					_IO  ( TPS_IOC_MAGIC, 18)
#define TPSDEV_GET_BASELINE				_IOR ( TPS_IOC_MAGIC, 19, unsigned short*)
#define TPSDEV_GET_FRAMELINE			_IOR ( TPS_IOC_MAGIC, 20, unsigned char*)
#define TPSDEV_START_FACETOUCHMODE		_IO  ( TPS_IOC_MAGIC, 21)
#define TPSDEV_STOP_FACETOUCHMODE		_IO  ( TPS_IOC_MAGIC, 22)
#define TPSDEV_POLL_FACETOUCHOFF		_IO  ( TPS_IOC_MAGIC, 23)
#define TPSDEV_GET_FWSTATUS				_IOR ( TPS_IOC_MAGIC, 24, unsigned char)
#define TPSDEV_GET_FWDATE				_IOR ( TPS_IOC_MAGIC, 25, unsigned short)
#define TPSDEV_CALIBRATION_PARAM		_IOW ( TPS_IOC_MAGIC, 26, struct shtps_ioctl_param)
#define TPSDEV_DEBUG_REQEVENT			_IOW ( TPS_IOC_MAGIC, 27, int)
#define TPSDEV_SET_DRAGSTEP				_IOW ( TPS_IOC_MAGIC, 28, int)
#define TPSDEV_SET_POLLINGINTERVAL		_IOW ( TPS_IOC_MAGIC, 29, int)
#define TPSDEV_SET_FINGERFIXTIME		_IOW ( TPS_IOC_MAGIC, 30, int)
#define TPSDEV_REZERO					_IO  ( TPS_IOC_MAGIC, 31)
#define TPSDEV_ACK_FACETOUCHOFF			_IO  ( TPS_IOC_MAGIC, 32)
#define TPSDEV_START_TM_F05				_IOW ( TPS_IOC_MAGIC, 33, int)
#define TPSDEV_SET_DRAGSTEP_X			_IOW ( TPS_IOC_MAGIC, 34, int)
#define TPSDEV_SET_DRAGSTEP_Y			_IOW ( TPS_IOC_MAGIC, 35, int)
#define TPSDEV_LOGOUTPUT_ENABLE			_IOW ( TPS_IOC_MAGIC, 36, int)
#define TPSDEV_GET_TOUCHKEYINFO			_IOR ( TPS_IOC_MAGIC, 37, struct shtps_touch_key_info)
#define TPSDEV_GET_FW_VERSION_BUILTIN	_IOR ( TPS_IOC_MAGIC, 38, unsigned short)
#define TPSDEV_GET_SMEM_BASELINE		_IOR ( TPS_IOC_MAGIC, 39, unsigned short*)
#define TPSDEV_SET_LOWPOWER_MODE		_IOW ( TPS_IOC_MAGIC, 40, int)
#define TPSDEV_SET_CONT_LOWPOWER_MODE	_IOW ( TPS_IOC_MAGIC, 41, int)
#define TPSDEV_SET_INVALID_AREA			_IOW ( TPS_IOC_MAGIC, 42, int)
#define TPSDEV_SET_CHARGER_ARMOR			_IOW(TPS_IOC_MAGIC, 43, int)
#define TPSDEV_SET_WIRELESS_CHARGER_ARMOR	_IOW(TPS_IOC_MAGIC, 44, int)
#define TPSDEV_LPWG_ENABLE	            _IOW(TPS_IOC_MAGIC, 45, int)
#define TPSDEV_SET_VEILVIEW_STATE		_IOW ( TPS_IOC_MAGIC, 46, int)
#define TPSDEV_READ_REG_BLOCK			_IOWR( TPS_IOC_MAGIC, 47, struct shtps_ioctl_param)
#define TPSDEV_WRITE_REG_BLOCK			_IOW ( TPS_IOC_MAGIC, 48, struct shtps_ioctl_param)
#define TPSDEV_GET_VEILVIEW_PATTERN		_IOR ( TPS_IOC_MAGIC, 49, int)
#define TPSDEV_READ_REG_PACKET			_IOWR( TPS_IOC_MAGIC, 50, struct shtps_ioctl_param)
#define TPSDEV_WRITE_REG_PACKET			_IOW ( TPS_IOC_MAGIC, 51, struct shtps_ioctl_param)
#define TPSDEV_HOVER_ENABLE				_IOW ( TPS_IOC_MAGIC, 52, int)
#define TPSDEV_GET_BASELINE_RAW			_IOR ( TPS_IOC_MAGIC, 53, unsigned short*)

#define TPSDEV_FACETOUCHOFF_NOCHG	0x00
#define TPSDEV_FACETOUCHOFF_DETECT	0x01

#define TPSDEV_TOUCHINFO_MODE_LCDSIZE	0
#define TPSDEV_TOUCHINFO_MODE_DEVSIZE	1

struct shtps_ioctl_param {
	int				size;
	unsigned char*	data;
};

struct shtps_bootloader_info {
	unsigned long	block_size;
	unsigned long	program_block_num;
	unsigned long	config_block_num;
};

struct shtps_touch_info {
	struct fingers{
		unsigned short	x;
		unsigned short	y;
		unsigned char	state;
		unsigned char	wx;
		unsigned char	wy;
		unsigned char	z;
	} fingers[SHTPS_FINGER_MAX];

	unsigned char		gs1;
	unsigned char		gs2;
	unsigned char		flick_x;
	unsigned char		flick_y;
	unsigned char		flick_time;
	unsigned char		finger_num;
};

struct shtps_touch_key_info {
	unsigned char		menu_key_state;
	unsigned char		home_key_state;
	unsigned char		back_key_state;
	unsigned char		down_key_state;
	unsigned char		up_key_state;
};

/* -----------------------------------------------------------------------------------
 */
#define SHTPS_F01_RMI_CTRL_ADR	0x0b

enum shtps_drag_threshold_mode {
	SHTPS_DRAG_THRESHOLD_ZERO,
	SHTPS_DRAG_THRESHOLD_1ST,
	SHTPS_DRAG_THRESHOLD_2ND
};

enum shtps_proc_mode {
	SHTPS_IN_IDLE,
	SHTPS_IN_NORMAL,
	SHTPS_IN_BOOTLOADER,
};

enum shtps_diag_mode {
	SHTPS_DIAGMODE_NONE,
	SHTPS_DIAGMODE_TOUCHINFO,
	SHTPS_DIAGMODE_TM,
};

enum shtps_diag_tm_mode {
	SHTPS_TMMODE_NONE,
	SHTPS_TMMODE_FRAMELINE,
	SHTPS_TMMODE_BASELINE,
	SHTPS_TMMODE_BASELINE_RAW,
};

struct shtps_platform_data {
	int (*setup)(struct device *);
	void (*teardown)(struct device *);
	int gpio_rst;
};

struct rmi_pdt {
	u8	queryBaseAddr;
	u8	commandBaseAddr;
	u8	controlBaseAddr;
	u8	dataBaseAddr;
	u8	interruptSrcCount;
	u8	functionNumber;
};

struct rmi_reg_type_num_info{
	u8	enable;
	u16	addr;
};

struct shtps_touch_state {
	u8				numOfFingers;
	u8				fingerStatus[SHTPS_FINGER_MAX];
	u8				dragStep[SHTPS_FINGER_MAX][2];
	u8				rezeroRequest;
	unsigned long	drag_timeout[SHTPS_FINGER_MAX][2];
	u16				dragStepReturnTime[SHTPS_FINGER_MAX][2];
};

#define F01_QUERY_MANUFACTURERID(data)			data[0]
#define F01_QUERY_CUSTOMMAP(data)				(data[1] & 0x01)
#define F01_QUERY_NONCOMPLIANT(data)			((data[1] >> 1) & 0x01)
#define F01_QUERY_HASSENSORID(data)				((data[1] >> 3) & 0x01)
#define F01_QUERY_HASAJUSTABLEDOZE(data)		((data[1] >> 5) & 0x01)
#define F01_QUERY_HASADJDOZEHOLDOFF(data)		((data[1] >> 6) & 0x01)
#define F01_QUERY_HASPRODUCTPROPERTIES2(data)	((data[1] >> 7) & 0x01)
#define F01_QUERY_PRODUCTINFO0(data)			(data[2] & 0x7F)
#define F01_QUERY_PRODUCTINFO1(data)			(data[3] & 0x7F)
#define F01_QUERY_DATECODEYEAR(data)			(data[4] & 0x1F)
#define F01_QUERY_DATECODEMONTH(data)			(((data[4] >> 5) & 0x07) | (data[5] & 0x01))
#define F01_QUERY_DATECODEDAY(data)				((data[5] >> 1) & 0x1F)
#define F01_QUERY_CP1(data)						((data[5] >> 6) & 0x01)
#define F01_QUERY_CP2(data)						((data[5] >> 7) & 0x01)
#define F01_QUERY_WAFERLOTID0(data)				((data[7] << 0x08) | data[6])
#define F01_QUERY_WAFERLOTID1(data)				((data[9] << 0x08) | data[8])
#define F01_QUERY_WAFERLOTID2(data)				data[10]
#define F01_QUERY_PRODUCTID(data)				data[11]
#define F01_QUERY_HASDS4QUERIES(data)			(data[21] & 0x01)
#define F01_QUERY_DS4_LENGTH(data)				(data[22] & 0x0F)
#define F01_QUERY_DS4_HASPACKAGEIDQUERY(data)	(data[23] & 0x01)
#define F01_QUERY_DS4_HASPACKRATQUERY(data)		((data[23] >> 1) & 0x01)
#define F01_QUERY_DS4_HASRESETQUERY(data)		((data[23] >> 2) & 0x01)
#define F01_QUERY_DS4_HASMASKREVQUERY(data)		((data[23] >> 3) & 0x01)
#define F01_QUERY_DS4_HASI2CCONTROL(data)		(data[24] & 0x01)
#define F01_QUERY_DS4_HASSPICONTROL(data)		((data[24] >> 1) & 0x01)
#define F01_QUERY_DS4_HASATTNCONTROL(data)		((data[24] >> 2) & 0x01)
#define F01_QUERY_DS4_HASTOOLIDPACKETQUERY(data)(data[25] & 0x01)
#define F01_QUERY_DS4_HASFIRMWAREREVISIONIDPACKETQUERY(data)	((data[25] >> 1) & 0x01)
#define F01_QUERY_RESET_ENABLED(data)			(data[26] & 0x01)
#define F01_QUERY_RESET_POLARITY(data)			((data[26] >> 1) & 0x01)
#define F01_QUERY_RESET_PULLUPENABLED(data)		((data[26] >> 2) & 0x01)

struct rmi_f01Query {
	u8	data[27];
};

struct rmi_f01 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f01Query	query;
};

#define F05_QUERY_NUMOFRCVEL(data)				(data[0] & 0x3F)
#define F05_QUERY_NUMOFTRANSEL(data)			(data[1] & 0x3F)
#define F05_QUERY_HAS16BIT(data)				((data[3] >> 7) & 0x01)
#define F05_QUERY_IMAGEWINDOWSIZE(data)			(data[4])

struct rmi_f05Query {
	u8	data[6];
};

struct rmi_f05 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f05Query	query;
};

#define F11_QUERY_NUMOFSENSORS(data)				(data[0] & 0x07)
#define F11_QUERY_HASQUERY9(data)					((data[0] >> 0x03) & 0x01)
#define F11_QUERY_HASQUERY11(data)					((data[0] >> 0x04) & 0x01)
#define F11_QUERY_HASQUERY12(data)					((data[0] >> 0x05) & 0x01)
#define F11_QUERY_NUMOFFINGERS(data)				(data[1] & 0x07)
#define F11_QUERY_HASREL(data)						((data[1] >> 0x03) & 0x01)
#define F11_QUERY_HASABS(data)						((data[1] >> 0x04) & 0x01)
#define F11_QUERY_HASGESTURES(data)					((data[1] >> 0x05) & 0x01)
#define F11_QUERY_HASSENSADJUST(data)				((data[1] >> 0x06) & 0x01)
#define F11_QUERY_CONFIGURABLE(data)				((data[1] >> 0x07) & 0x01)
#define F11_QUERY_NUMOFXELEC(data)					(data[2] & 0x1F)
#define F11_QUERY_NUMOFYELEC(data)					(data[3] & 0x1F)
#define F11_QUERY_MAXELEC(data)						(data[4] & 0x1F)
#define F11_QUERY_ABSOLUTEDATASIZE(data)			(data[5] & 0x03)
#define F11_QUERY_ANCHOREDFINGER(data)				((data[5] >> 2) & 0x01)
#define F11_QUERY_HASDRIBBLE(data)					((data[5] >> 4) & 0x01)
#define F11_QUERY_HASBENDINGCORRECTION(data)		((data[5] >> 5) & 0x01)
#define F11_QUERY_HASLARGEOBJECTSUPPRESSION(data)	((data[5] >> 6) & 0x01)
#define F11_QUERY_HASJITTERFILTER(data)				((data[5] >> 7) & 0x01)
#define F11_QUERY_HASPEN(data)						(data[6] & 0x01)
#define F11_QUERY_HASFINGERPROXIMITY(data)			((data[6] >> 1) & 0x01)
#define F11_QUERY_HASLARGEOBJECTSENSITIVITY(data)	((data[6] >> 2) & 0x01)
#define F11_QUERY_HASTWOPENTHRESHOLDS(data)			((data[6] >> 4) & 0x01)
#define F11_QUERY_HASPENHOVERDISCRIMINATION(data)	((data[6] >> 6) & 0x01)
#define F11_QUERY_HASNEWPENREGISTERS(data)			((data[6] >> 7) & 0x01)
#define F11_QUERY_HASZTUNING(data)					(data[7] & 0x01)
#define F11_QUERY_HASPOSITIONIPTUNING(data)			((data[7] >> 1) & 0x01)
#define F11_QUERY_HASWTUNING(data)					((data[7] >> 2) & 0x01)
#define F11_QUERY_HASPITCHINFO(data)				((data[7] >> 3) & 0x01)
#define F11_QUERY_HASDEFAULTFINGERWIDTH(data)		((data[7] >> 4) & 0x01)
#define F11_QUERY_HASSEGAGGRESSIVENESS(data)		((data[7] >> 5) & 0x01)
#define F11_QUERY_HASTXRXCLIP(data)					((data[7] >> 6) & 0x01)
#define F11_QUERY_HASDRUMMINGCORRECTION(data)		((data[7] >> 7) & 0x01)
#define F11_QUERY_HAS8BITW(data)					((data[8] >> 2) & 0x01)
#define F11_QUERY_HAS2DAJSTMAPPING(data)			((data[8] >> 3) & 0x01)

struct rmi_f11Query {
	u8	data[9];
};
struct rmi_f11Ctrl {
	u16 maxXPosition;
	u16 maxYPosition;
};

struct rmi_f11 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f11Query	query;
	struct rmi_f11Ctrl	ctrl;
};

#define F12_QUERY_HASREGISTERDESCRIPTOR(data)		(data[0] & 0x01)
#define F12_QUERY_SIZEOFQUERYPRESENCE(data)			(data[1])
#define F12_QUERY_SIZEOFQUERYSTRUCTURE(data)		(data[2])
#define F12_QUERY_QUERYSTRUCTURE(data)				(data[3])
#define F12_QUERY_SIZEOFCONTROLPRESENCE(data)		(data[4])
#define F12_QUERY_SIZEOFCONTROLSTRUCTURE(data)		(data[5])
#define F12_QUERY_CONTROLSTRUCTURE(data)			(data[6])
#define F12_QUERY_SIZEOFDATAPRESENCE(data)			(data[7])
#define F12_QUERY_SIZEOFDATASTRUCTURE(data)			(data[8])
#define F12_QUERY_DATASTRUCTURE(data)				(data[9])

struct rmi_f12Query {
	u8	data[10];
};

struct rmi_f12Ctrl {
	u8  maxFingerNum;
	u16 maxXPosition;
	u16 maxYPosition;
	struct rmi_reg_type_num_info num[29];
};

struct rmi_f12Data {
	struct rmi_reg_type_num_info num[16];
};

struct rmi_f12 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f12Query	query;
	struct rmi_f12Ctrl	ctrl;
	struct rmi_f12Data	data;
};

#define F34_QUERY_BOOTLOADERID0(data)		data[0]
#define F34_QUERY_BOOTLOADERID1(data)		data[1]
#define F34_QUERY_REGMAP(data)				(data[8] & 0x01)
#define F34_QUERY_UNLOCKED(data)			((data[8] >> 1) & 0x01)
#define F34_QUERY_HASCONFIGID(data)			((data[8] >> 2) & 0x01)
#define F34_QUERY_BLOCKSIZE(data)			((data[9] & 0xff) | ((data[10] << 0x08) & 0xff00))
#define F34_QUERY_FIRMBLOCKCOUNT(data)		((data[11] & 0xff) | ((data[12] << 0x08) & 0xff00))
#define F34_QUERY_CONFIGBLOCKCOUNT(data)	((data[13] & 0xff) | ((data[14] << 0x08) & 0xff00))

struct rmi_f34Query {
	u8	data[19];
};

struct rmi_f34 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	struct rmi_f34Query	query;
};

#define F54_QUERY_NUMOFRCVEL(data)						data[0]
#define F54_QUERY_NUMOFTRANSEL(data)					data[1]
#define F54_QUERY_HASBASELINE(data)						((data[2] >> 2) & 0x01)
#define F54_QUERY_HAS8BIT(data)							((data[2] >> 3) & 0x01)
#define F54_QUERY_HAS16BIT(data)						((data[2] >> 6) & 0x01)
#define F54_QUERY_CLOCKRATE(data)						((data[4] << 0x08) | data[3])
#define F54_QUERY_ANALOGHARDWAREFAMILY(data)			data[5]
#define F54_QUERY_HASPIXELTOUCHTHRESHOLDTUNING(data)	(data[6] & 0x01)
#define F54_QUERY_HASARBITRARYSENSORASSIGNMENT(data)	(data[7] & 0x01)
#define F54_QUERY_HASINTERFERENCEMETRIC(data)			((data[7] >> 1) & 0x01)
#define F54_QUERY_HASSENSEFREQCONTROL(data)				((data[7] >> 2) & 0x01)
#define F54_QUERY_HASFWNOISEMITIGATION(data)			((data[7] >> 3) & 0x01)
#define F54_QUERY_HASLOWPOERCTRL(data)					((data[7] >> 4) & 0x01)
#define F54_QUERY_HASTWOBYTEREPORTRATEREPORTING(data)	((data[7] >> 5) & 0x01)
#define F54_QUERY_HASONEBYTEREPORTRATEREPORTING(data)	((data[7] >> 6) & 0x01)
#define F54_QUERY_HASRELAXATIONCTRL(data)				((data[7] >> 7) & 0x01)
#define F54_QUERY_AXISCOMPENSATIONMODE(data)			(data[8] & 0x03)
#define F54_QUERY_HASIIRFILTER(data)					((data[9] >> 1) & 0x01)
#define F54_QUERY_HASCMNREMOVAL(data)					((data[9] >> 2) & 0x01)
#define F54_QUERY_HASCMNCAPSCALEFACTOR(data)			((data[9] >> 3) & 0x01)
#define F54_QUERY_HASPIXCELTHRESHHYSTERESIS(data)		((data[9] >> 4) & 0x01)
#define F54_QUERY_HASEDGECOMPENSATION(data)				((data[9] >> 5) & 0x01)
#define F54_QUERY_HASPERFREQNOISECTRL(data)				((data[9] >> 6) & 0x01)
#define F54_QUERY_HASFORCEFASTRELAXATION(data)			(data[10] & 0x01)
#define F54_QUERY_HASMMSTATEMITIGATION(data)			((data[10] >> 1) & 0x01)
#define F54_QUERY_HASCDM4(data)							((data[10] >> 2) & 0x01)
#define F54_QUERY_HASVARIANCEMETRIC(data)				((data[10] >> 3) & 0x01)
#define F54_QUERY_HAS0DRELAXATIONCTRL(data)				((data[10] >> 4) & 0x01)
#define F54_QUERY_HAS0DACQUISITIONCTRL(data)			((data[10] >> 5) & 0x01)
#define F54_QUERY_MAXNUMOFSENSINGFREQ(data)				((data[14] >> 5) & 0x01)
#define F54_QUERY_BURSTSPERCLUSTER(data)				((data[15] >> 5) & 0x01)

struct rmi_f54Query {
	u8	data[21];
};

struct rmi_f54 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f54Query	query;
};

#define F19_QUERY_HASSENSITIVITYADJUST(data)	((data[0] >> 1) & 0x01)
#define F19_QUERY_HASHYSTERESISTHRESHOLD(data)	((data[0] >> 2) & 0x01)
#define F19_QUERY_BUTTONCOUNT(data)				(data[1] & 0x1F)

struct rmi_f19Query {
	u8	data[2];
};

struct rmi_f19 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f19Query	query;
};

#define F1A_QUERY_MAX_BUTTONCOUNT(data)					(data[0] & 0x07)
#define F1A_QUERY_HASGENERALCONTROL(data)				((data[1] >> 0) & 0x01)
#define F1A_QUERY_HASINTERRUPTENABLE(data)				((data[1] >> 1) & 0x01)
#define F1A_QUERY_HASMULTIBUTTONSELECT(data)			((data[1] >> 2) & 0x01)
#define F1A_QUERY_HASTXRXMAPPING(data)					((data[1] >> 3) & 0x01)
#define F1A_QUERY_HASPERBUTTONTHRESHOLD(data)			((data[1] >> 4) & 0x01)
#define F1A_QUERY_HASRELEASETHRESHOLD(data)				((data[1] >> 5) & 0x01)
#define F1A_QUERY_HASSTRONGESTBUTTONHYSTERESIS(data)	((data[1] >> 6) & 0x01)
#define F1A_QUERY_HASFILTERSTRENGTH(data)				((data[1] >> 7) & 0x01)

struct rmi_f1AQuery {
	u8	data[2];
};

struct rmi_f1A {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f1AQuery	query;
};

#define F51_QUERY_QUERYREGISTERCOUNT(data)		data[0]
#define F51_QUERY_DATAREGISTERCOUNT(data)		data[1]
#define F51_QUERY_CONTROLREGISTERCOUNT(data)	data[2]
#define F51_QUERY_COMMANDREGISTERCOUNT(data)	data[3]

struct rmi_f51Query {
	u8	data[4];
};

struct rmi_f51 {
	u8					enable;
	u32					queryBase;
	u32					ctrlBase;
	u32					dataBase;
	u32					commandBase;
	struct rmi_f51Query	query;
};

struct rmi_map {
	struct rmi_f01	fn01;
	struct rmi_f05	fn05;
	struct rmi_f11	fn11;
	struct rmi_f12	fn12;
	struct rmi_f34	fn34;
	struct rmi_f54	fn54;
	struct rmi_f19	fn19;
	struct rmi_f1A	fn1A;
	struct rmi_f51	fn51;
};

//#define F11_DATA_FINGERSTATE0(data)		data[0]
#define F11_DATA_XPOS(data)				(((data[0] << 0x04) & 0x0FF0) | (data[2] & 0x0F))
#define F11_DATA_YPOS(data)				(((data[1] << 0x04) & 0x0FF0) | ((data[2] >> 0x04) & 0x0F))
#define F11_DATA_WX(data)				(data[3] & 0x0F)
#define F11_DATA_WY(data)				((data[3] >> 0x04) & 0x0F)
#define F11_DATA_Z(data)				data[4]

#define F12_DATA_FINGERSTATE(data)		(data[0])
#define F12_DATA_XPOS(data)				(data[1] | (data[2] << 8))
#define F12_DATA_YPOS(data)				(data[3] | (data[4] << 8))
#define F12_DATA_Z(data)				(data[5])
#define F12_DATA_WX(data)				(data[6])
#define F12_DATA_WY(data)				(data[7])

struct shtps_rmi_fingerState {
	u8	data[6];
};

/* -----------------------------------------------------------------------------------
 */
extern void msm_tps_setsleep(int on);
extern int msm_tps_set_veilview_state_on(void);
extern int msm_tps_set_veilview_state_off(void);
extern int msm_tps_get_veilview_pattern(void);
extern void msm_tps_set_grip_state(int on);

#endif /* __SHTPS_SY3000_TM2858_001_H__ */
