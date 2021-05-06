#ifndef TOUCHPANEL_H__
#define TOUCHPANEL_H__

#include <linux/ioctl.h>

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE         MT6573_POWER_VGP2
#define TPD_I2C_NUMBER           0
#define TPD_WAKEUP_TRIAL         60
#define TPD_WAKEUP_DELAY         100

#define TPD_DELAY                (2*HZ/100)
//#define TPD_RES_X                480
//#define TPD_RES_Y                800
#define TPD_CALIBRATION_MATRIX_ROTATION_FACTORY  {4096, 0, 0, 0, 4096, 0, 0, 0};
//#define TPD_CALIBRATION_MATRIX  {-4096, 0, 4190208, 0, 4096, 0, 0, 0};
#define TPD_CALIBRATION_MATRIX_ROTATION_NORMAL  {6995, 0, 0, 0, 2398, 0, 0, 0};

#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_BUTTON
//#define TPD_HAVE_TOUCH_KEY
#define TPD_HAVE_TREMBLE_ELIMINATION

#define TPD_RESET_ISSUE_WORKAROUND

#define TPD_HAVE_POWER_ON_OFF

#define MAX_TRANSACTION_LENGTH 8
#define I2C_DEVICE_ADDRESS_LEN 2
#define MAX_I2C_TRANSFER_SIZE (MAX_TRANSACTION_LENGTH - I2C_DEVICE_ADDRESS_LEN)
#define MAX_I2C_MAX_TRANSFER_SIZE 8


#define TPD_TOUCH_INFO_REG_BASE 0xF40
#define TPD_KEY_INFO_REG_BASE 0xF41
#define TPD_POINT_INFO_REG_BASE 0xF42
#define TPD_POWER_MODE_REG 0xFF2
#define TPD_I2C_ENABLE_REG 0x0FFF
#define TPD_I2C_DISABLE_REG 0x8000
#define TPD_VERSION_INFO_REG 240
#define TPD_CONFIG_REG_BASE 0xF80

#define TPD_KEY_HOME 0x02
#define TPD_KEY_BACK 0x04
#define TPD_KEY_MENU 0x01

#define INT_TRIGGER 0x01
#define MAX_FINGER_NUM 5
#define TPD_POINT_INFO_LEN 5
#define TPD_TOUCH_INFO_LENGTH 1

#ifdef TPD_HAVE_TOUCH_KEY
	const u8 touchKeyArray[] = { KEY_MENU, KEY_HOMEPAGE, KEY_BACK };
	#define TPD_TOUCH_KEY_NUM ( sizeof( touchKeyArray ) / sizeof( touchKeyArray[0] ) )
#endif

static u8 cfg_data[] =
{	
	0x00,0x0F,0x01,0x10,0x02,0x11,0x03,0x12,
	0x04,0x13,0x05,0x14,0x06,0x15,0x1C,0x0D,
	0x1B,0x0C,0x1A,0x0B,0x19,0x0A,0x18,0x09,
	0x17,0x08,0x16,0x07,0xFF,0xFF,0x02,0x0C,
	0x03,0x0D,0x04,0x0E,0x05,0x0F,0x06,0x10,
	0x07,0x11,0x08,0x12,0x09,0x13,0xFF,0x01,
	0x0B,0x00,0x0F,0x03,0x88,0x88,0x88,0x19,
	0x00,0x00,0x05,0x00,0x00,0x0E,0x50,0x3C,
	0x01,0x03,0x00,0x05,0x00,0x02,0x58,0x04,
	0x00,0x5A,0x5A,0x46,0x46,0x08,0x00,0x03,
	0x19,0x05,0x14,0x10,0x00,0x04,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x00,0x00,0x00,0x00,0x68,0x01			
};

/*
static u8 cfg_data_evt[] =
{
	0x15,0x06,0x14,0x05,0x13,0x04,0x12,0x03,
	0x11,0x02,0x10,0x01,0x0F,0x00,0x07,0x16,
	0x08,0x17,0x09,0x18,0x0A,0x19,0x0B,0x1A,
	0x0C,0x1B,0x0D,0x1C,0x0E,0xFF,0x02,0x0C,
	0x03,0x0D,0x04,0x0E,0x05,0x0F,0x06,0x10,
	0x07,0x11,0x08,0x12,0x09,0x13,0xFF,0x01,
	0x0B,0x00,0x0F,0x03,0x88,0x88,0x88,0x1B,
	0x00,0x00,0x08,0x00,0x00,0x0E,0x50,0x3C,
	0x01,0x03,0x00,0x05,0x00,0x02,0x58,0x04,
	0x00,0x5A,0x5A,0x46,0x46,0x08,0x00,0x03,
	0x19,0x05,0x14,0x10,0x00,0x04,0x00,0x07,
	0x18,0x48,0x70,0x00,0x50,0x3C,0x60,0x20,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x01,0x00,0x00,0x00,0x00,0x68,0x01   
};
*/

/*For factory Mode*/
#define TPD_NAME				"GT827"
#define TPD_IOCTL_MAGIC			't'
#define TPD_IOCTL_ENABLE_I2C 	_IO(TPD_IOCTL_MAGIC, 1)
#define TPD_IOCTL_DISABLE_I2C	_IO(TPD_IOCTL_MAGIC, 2)


#endif /* TOUCHPANEL_H__ */
