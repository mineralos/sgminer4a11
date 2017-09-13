
/*
 * Copyright 2012-2013 Andrew Smith
 * Copyright 2012 Luke Dashjr
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

//=====================================================================//
//***	driver-ltctech.c is for scrypt algorithm mining by using Han-Lab's Raptor-XXX series miner		***//
//=====================================================================//

//=====================================================================//
//  DRIVER_LTCTECH DEFINITION FOR SCRYPT ALGORITHM
//  Support Product:
//		1) Raptor-I		: BAYSAND Scrypt ASIC Chip
//						: 1 base b'd, 10 miner b'd, 1 miner b'd includes 4EA ASIC Chip
//		2) Raptor-II		: BAYSAND Scrypt ASIC Chip
//						: 2 miner b'd(operating independently), 1 miner b'd includes 20EA ASIC Chip
//=====================================================================//


#include "config.h"

#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>

#include "logging.h"
#include "miner.h"
#include "usbutils.h"
#include "util.h"
#include "driver-ltctech.h"
#include "compat.h"


#define LTCTECH_TIMEOUT		(5)


#define LTCTECH_STX				(0x12)
#define LTCTECH_ETX				(0x13)

#define STX_POS					(0)
#define CMD_POS					(1)
#define LEN_POS					(2)
#define DATA_POS				(3)
#define ETX_POS					(DATA_POS + len)

#define WAIT_TIME				(10)
#define INFO_SIZE				(28)
#define ALGO_SIZE				(2)
#define CORE_SIZE				(16)
#define CLOCK_SIZE				(6)
#define MWORK_SIZE				(2)

#define WORK_SIZE				(80)
#define DEVICE_TARGET_SIZE		(32)
#define NONCE_POS				(76)
#define NONCE_SIZE				(4)
#define TARGET_POS				(80)
#define TARGET_SIZE				(4)
#define MINER_ID_POS			(84)
#define MINER_ID_SIZE			(1)
#define WORK_ID_POS			(85)
#define WORK_ID_SIZE			(1)
#define FIND_NONCE_SIZE		(6)				// For receive value from miner: 4-Bytes nonce, 1-Byte miner_id, 1-Byte work_id

#define REPLY_SIZE				(2)
#define BUF_SIZE					(128)
#define TEMP_UPDATE_TIME		(5 * 1000)		/* 30sec -> 5sec*/

// Commands
enum ltctech_cmd
{
	LTCTECH_RESET				= 0x00,
	LTCTECH_GET_STATUS		= 0x01,
	LTCTECH_GET_INFO			= 0x02,
	LTCTECH_SET_CLK			= 0x03,
	LTCTECH_GET_CLK			= 0x04,
	LTCTECH_SET_TEMP			= 0x05,
	LTCTECH_GET_TEMP			= 0x06,
	LTCTECH_RESERVED07		= 0x07,
	LTCTECH_RESERVED08		= 0x08,
	LTCTECH_SEND_WORK		= 0x09,
	LTCTECH_RESERVED0A		= 0x0A,
	LTCTECH_RESERVED0B		= 0x0B,
	LTCTECH_RESERVED0C		= 0x0C,
	LTCTECH_RESERVED0D		= 0x0D,
	LTCTECH_RESERVED0E		= 0x0E,
	LTCTECH_RESERVED0F		= 0x0F,
	LTCTECH_GET_WORK			= 0x10,
	LTCTECH_SET_LED			= 0x11,
	LTCTECH_SET_IDLE			= 0x12,
	LTCTECH_SET_ALGO			= 0x13,
	LTCTECH_GET_CORE			= 0x14,
	LTCTECH_SET_MUTI			= 0x15,
	LTCTECH_HW_RESET			= 0x16,
	LTCTECH_STOP_MINER		= 0x17,
	LTCTECH_START_MINER		= 0x18
};

enum usb_cmds ltctech_usb_cmd[] =
{
	C_RESET,                		// LTCTECH_RESET			= 0x00
	C_GET_STATUS, 	             	// LTCTECH_GET_STATUS,		= 0x01
	C_GET_INFO,   	        	// LTCTECH_GET_INFO,		= 0x02
	C_SETCLOCK,             		// LTCTECH_SET_CLK		= 0x03
	C_REPLYSETCLOCK,        	// LTCTECH_GET_CLK		= 0x04
	C_REQUESTTEMPERATURE,	// LTCTECH_SET_TEMP		= 0x05
	C_GETTEMPERATURE,       	// LTCTECH_GET_TEMP		= 0x06
	C_MAX,                      		// NOT EXIST				= 0x07
	C_MAX,                      		// NOT EXIST				= 0x08
	C_SENDWORK,             	// LTCTECH_SEND_WORK		= 0x09
	C_MAX,                  		// NOT EXIST				= 0x0A
	C_MAX,                  		// NOT EXIST				= 0x0B
	C_MAX,                  		// NOT EXIST				= 0x0C
	C_MAX,                  		// NOT EXIST				= 0x0D
	C_MAX,                  		// NOT EXIST				= 0x0E
	C_MAX,                  		// NOT EXIST				= 0x0F
	C_GETWORKSTATUS,   		// LTCTECH_GET_WORK		= 0x10
	C_REQUESTIDENTIFY,		// LTCTECH_SET_LED		= 0x11
	C_SET_IDLE,                   	// LTCTECH_SET_IDLE		= 0x12
	C_SET_ALGO, 			// LTCTECH_SET_ALGO		= 0x13
	C_GET_CORE,				// LTCTECH_GET_CORE		= 0x14
	C_MAX,					// NOT EXIST				= 0x15
	C_SET_HWRESET,			// LTCTECH_HW_RESET		= 0x16
	C_SET_STOP_MINER,		// LTCTECH_STOP_MINER		= 0x17
	C_SET_START_MINER		// LTCTECH_START_MINER	= 0x18
};


typedef enum
{
	TEMP_90C,
	TEMP_80C,
	TEMP_70C,
	TEMP_60C,
	TEMP_50C,
	TEMP_40C,
	TEMP_30C,
	TEMP_MINUS_40C,
	TEMP_MAX_NUM
}TC_TEMP;


// Commands result
enum ltctech_result
{
    LTCTECH_RESULT_OK = 0,
    LTCTECH_SEND_FAIL = 1,
    LTCTECH_RECV_FAIL = 2,
    LTCTECH_CMD_FAIL  = 3
};



const int16_t		ltctech_core_clock[LTCTECH_CORE_CLOCK_SET_NUM] = { 400, 412, 425, 433, 437, 450, 462, 466, 483, 487, 500 };

#if LTCTECH_TEST_MODE
static void ltctech_set_testdata(struct work *work);
static void ltctech_print_hash(struct work *work, uint32_t nonce);
#endif

static void ltctech_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce);

static bool ltctech_set_algorithm(struct cgpu_info *ltctech);

void ltctech_usb_clear(struct cgpu_info *ltctech)
{
	char		buf[512];
	int		bufsize = 512;
	int		amount;

	do{
		usb_read_once(ltctech, buf, bufsize, &amount, C_CLEAR);
	} while (amount);
}

static int ltctech_encode(unsigned char *packet, unsigned char cmd, unsigned char *data, int len)
{
	packet[STX_POS] = LTCTECH_STX;
	packet[CMD_POS] = cmd;
	packet[LEN_POS] = len;

	if(len > 0)
	{
		memcpy(&packet[DATA_POS], data, len);
	}

	packet[ETX_POS] = LTCTECH_ETX;

	return (ETX_POS + 1);
}


static bool ltctech_send(struct cgpu_info *ltctech, uint8_t cmd, char *data, int len)
{
	char		buf[BUF_SIZE];
	int		err, amount, decode_len;

	usb_buffer_clear(ltctech);

	decode_len = ltctech_encode(buf, cmd, data, len);
	if((err = usb_write(ltctech, buf, decode_len, &amount, ltctech_usb_cmd[cmd])) < 0 || amount != decode_len)
	{
		return (false);
	}

	return (true);
}


static int ltctech_recv(struct cgpu_info *ltctech, uint8_t cmd, char *data, int len)
{
	int		remain;
	int		result, amount;
	char		*pos = data;

	remain = len;

	while (remain > 0)
	{
		if((result = usb_read_once(ltctech, pos, remain, &amount, ltctech_usb_cmd[cmd])) < 0 || amount < 1)
		{
			return (result);
		}

		if(result && result != LIBUSB_ERROR_TIMEOUT)
		{
			applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_recv():result=%d", ltctech->drv->name, ltctech->device_id, result);
			return (result);
		}

		remain -= amount;
		pos += amount;
	}

	return (len);
}


static int ltctech_send_cmd(struct cgpu_info *ltctech, uint8_t cmd, uint8_t *data, uint32_t data_len, uint8_t *status)
{
	int		retry = 0;
	char		buf[BUF_SIZE];

	mutex_lock(ltctech->device_mutex);

	if(ltctech_send(ltctech, cmd, data, data_len) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		return (LTCTECH_SEND_FAIL);
	}

	if(ltctech_recv(ltctech, cmd, buf, REPLY_SIZE) != REPLY_SIZE)
	{
		mutex_unlock(ltctech->device_mutex);
		return (LTCTECH_RECV_FAIL);
	}

	mutex_unlock(ltctech->device_mutex);

	if(buf[0] != cmd)
	{
		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_send_cmd():cmd=0x%02X,buf[0]=0x%02X", ltctech->drv->name, ltctech->device_id, cmd, buf[0]);
		return (LTCTECH_CMD_FAIL);
	}

	*status = buf[1];

	return (LTCTECH_RESULT_OK);
}


static void ltctech_info_clear(struct ltctech_info *info)
{
	int		miner_index;
	
	if(!info)
	{
		return;
	}
	
	if(info->getwork_thr)
	{
		pthread_join(*info->getwork_thr, NULL);
	}

	for(miner_index = 0; miner_index < LTCTECH_MINER_MAX; miner_index++)
	{
		if(info->ltcwork[miner_index])
		{
			free_work(info->ltcwork[miner_index]);
		}
	}
}


static bool ltctech_getinfo(struct cgpu_info *ltctech)
{
	char		buf[BUF_SIZE];
	uint8_t	cmd = LTCTECH_GET_INFO;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	mutex_lock(ltctech->device_mutex);

	if(ltctech_send(ltctech, cmd, NULL, 0) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}

	if(ltctech_recv(ltctech, cmd, buf, INFO_SIZE) != INFO_SIZE)
	{
		mutex_unlock(ltctech->device_mutex);
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_getinfo():size", ltctech->drv->name);
		return (false);
	}

	mutex_unlock(ltctech->device_mutex);

	/* TODO : exception */
	if((buf[0] != cmd) || (buf[1] != LTCTECH_RESULT_OK))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_getinfo():cmd=0x%02X,buf[0]=0x%02X,buf[1]=0x%02X", ltctech->drv->name, cmd, buf[0], buf[1]);
		return (false);
	}

	memcpy(info->product, &buf[2], INFO_SIZE - 2);
	ltctech->name = strdup(info->product);

	if(memcmp(LTCTECH_SERILA_NUMBER_RAPTOR_II, info->serial, LTCTECH_SERIAL_NUMBER_SIZE) == 0)
	{
		info->product_type		= LTCTECH_PRODUCT_TYPE_RAPTOR_II;
		strcpy(info->product_name, LTCTECH_PRODUCT_NAME_RAPTOR_II);
		info->miner_core_num		= LTCTECH_MINER_CORE_NUM_RAPTOR_II;
		info->algorithm_clock		= LTCTECH_ALGORITHM_CLOCK_RAPTOR_II;
		info->working_time_offset	= LTCTECH_WORKING_TIME_OFFSET_RAPTOR_II;
		info->hashrate_offset		= LTCTECH_HASH_RATE_OFFSET_RAPTOR_II;
	}
	else
	{
		info->product_type		= LTCTECH_PRODUCT_TYPE_RAPTOR_I;
		strcpy(info->product_name, LTCTECH_PRODUCT_NAME_RAPTOR_I);
		info->miner_core_num		= LTCTECH_MINER_CORE_NUM_RAPTOR_I;
		info->algorithm_clock		= LTCTECH_ALGORITHM_CLOCK_RAPTOR_I;
		info->working_time_offset	= LTCTECH_WORKING_TIME_OFFSET_RAPTOR_I;
		info->hashrate_offset		= LTCTECH_HASH_RATE_OFFSET_RAPTOR_I;
	}

	ltctech->hw_ver = info->hw_ver;
	ltctech->fw_ver = info->fw_ver;
	ltctech->core_ver = info->core_ver;

	if(info->algo_type != kernel)
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_getinfo():Algorithm mismached", ltctech->drv->name);
		return (false);
	}				
 
	return (true);
}


static bool ltctech_reset(struct cgpu_info *ltctech)
{
	int		result;
	int		retry;
	uint8_t	status;

	for(retry = 0; retry < 5; retry++)		//  Retry for Atmel SAM4E8E MCU
	{
		result = ltctech_send_cmd(ltctech, LTCTECH_RESET, NULL, 0, &status);
		if((result == LTCTECH_RESULT_OK) && (status == LTCTECH_RESULT_OK))
		{
			return (true);
		}

		cgsleep_ms(1);
	}

	applog(LOG_ERR, "[%s X]:ERROR  - ltctech_reset():result=%d,status=%d", ltctech->drv->name, result, status);
	return (false);
}


static void ltctech_identify(struct cgpu_info *ltctech)
{
	int		result;
	uint8_t	status;

	result = ltctech_send_cmd(ltctech, LTCTECH_SET_LED, NULL, 0, &status);
	if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
	{
		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_identify(): result=%d, status=%d", ltctech->drv->name, ltctech->device_id, result, status);
	}
}

static bool ltctech_set_diff(struct cgpu_info *ltctech, double diff)
{
	struct ltctech_info *info = ltctech->device_data;

	/* set device working diff */
	info->working_diff = diff;

	return (true);
}

static bool ltectech_get_status(struct cgpu_info *ltctech)
{
	int		len;
	uint8_t	cmd = LTCTECH_GET_STATUS;

	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data; 	

	if(!info)
	{
		return (false);
	}

	mutex_lock(ltctech->device_mutex);

	if(ltctech_send(ltctech, cmd, NULL, 0) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}

	if(info->product_type == LTCTECH_PRODUCT_TYPE_RAPTOR_I)
	{
		len = sizeof(struct miner_info_raptor_i) * LTCTECH_MINER_NUM_RAPTOR_I;		// 10 -> LTCTECH_MINER_NUM_RAPTOR_I
		if(ltctech_recv(ltctech, cmd, (char*)&info->miner_raptor_i, len) != len)
		{
			mutex_unlock(ltctech->device_mutex);
			applog(LOG_ERR, "[%s X]:ERROR  - ltectech_get_status():size", ltctech->drv->name);
			return (false);
		}
	}
	else
	{
		len = sizeof(struct miner_info_raptor_ii) * LTCTECH_MINER_NUM_RAPTOR_II;

		if(ltctech_recv(ltctech, cmd, (char*)&info->miner_raptor_ii, len) != len)
		{
			mutex_unlock(ltctech->device_mutex);
			applog(LOG_ERR, "[%s X]:ERROR  - ltectech_get_status():size", ltctech->drv->name);
			return (false);
		}
	}

	mutex_unlock(ltctech->device_mutex);
		
	return (true);
}

static bool ltctech_set_clock(struct cgpu_info *ltctech, uint32_t clock)
{
	int		result;
	uint8_t	status;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	if(info->product_type == LTCTECH_PRODUCT_TYPE_RAPTOR_I)
		result = ltctech_send_cmd(ltctech, LTCTECH_SET_CLK, (uint8_t*)&clock, 1, &status);
	else
		result = ltctech_send_cmd(ltctech, LTCTECH_SET_CLK, (uint8_t*)&clock, 2, &status);

	if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_set_clock():result=%d,status=%d", ltctech->drv->name, result, status);
		return false;
	}

	info->clock = clock;

	return (true);
}

static bool ltctech_get_clock(struct cgpu_info *ltctech)
{
	char		buf[BUF_SIZE];
	uint8_t	cmd = LTCTECH_GET_CLK;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	mutex_lock(ltctech->device_mutex);

	if(ltctech_send(ltctech, cmd, NULL, 0) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}

	if(ltctech_recv(ltctech, cmd, buf, CLOCK_SIZE) != CLOCK_SIZE)
	{
		mutex_unlock(ltctech->device_mutex);
		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_get_clock():size", ltctech->drv->name, ltctech->device_id);
		return (false);
	}

	mutex_unlock(ltctech->device_mutex);

	if((buf[0] != cmd) || (buf[1] != LTCTECH_RESULT_OK))
	{
		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_get_clock():cmd=0x%02X,buf[0]=0x%02X,buf[1]=0x%02X", ltctech->drv->name, ltctech->device_id, cmd, buf[0], buf[1]);
		return (false);
	}

	memcpy(&info->clock, &buf[2], CLOCK_SIZE - 2);

	return (true);
}


static bool ltctech_cal_working_time(struct cgpu_info *ltctech)
{
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	// It is msecond value
	info->working_time	= ((0xFFFFFFFF / (((info->clock * 1000000) / info->algorithm_clock) * info->core_total_cnt)) * 1000) - info->working_time_offset;

	return (true);
}


static bool ltctech_set_temp(struct cgpu_info *ltctech, uint8_t temp)
{
	int		result;
	uint8_t	status;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	if((temp < LTCTECH_TEMP_MIN) || (temp > LTCTECH_TEMP_MAX))
	{
		temp = LTCTECH_TEMP_DEF;
	}

	result = ltctech_send_cmd(ltctech, LTCTECH_SET_TEMP, &temp, 1, &status);
	if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_set_temp():result=%d,status=%d", ltctech->drv->name, result, status);
		return false;
	}

	info->settemp = temp;

	return (true);
}

 
static bool ltctech_get_temp(struct cgpu_info *ltctech)
{
	int		result;
	uint8_t	temp;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	result = ltctech_send_cmd(ltctech, LTCTECH_GET_TEMP, NULL, 0, &temp);

	if(result != LTCTECH_RESULT_OK)
	{
		temp = LTCTECH_TEMP_DISP;
//		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_get_temp():result=%d", ltctech->drv->name, ltctech->device_id, result);
	}
	else if(temp > LTCTECH_TEMP_MAX)
	{
		temp = LTCTECH_TEMP_MAX;
	}

	info->temp = temp;

	if(info->temp_max < info->temp)
	{
		info->temp_max = info->temp;
	}

	ltctech->temp = (float)temp;
		
	return (true);
}


static bool ltctech_set_algorithm(struct cgpu_info *ltctech)
{
	int		result;
	char		buf[BUF_SIZE];
	uint8_t	cmd = LTCTECH_SET_ALGO;
	unsigned char algo = (unsigned char)kernel;
	
	mutex_lock(ltctech->device_mutex);

	if(ltctech_send(ltctech, cmd, &algo, 1) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}
	
	if(ltctech_recv(ltctech, cmd, buf, ALGO_SIZE) != ALGO_SIZE)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}

	mutex_unlock(ltctech->device_mutex);

	if((buf[0] != LTCTECH_SET_ALGO) || (buf[1] != 0))
	{
		return (false);
	}

	return (true);
}


static bool ltctech_get_core(struct cgpu_info *ltctech)
{
	int		miner_index;
	char		buf[BUF_SIZE];
	uint8_t	cmd = LTCTECH_GET_CORE;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;
	
	mutex_lock(ltctech->device_mutex);

	if(ltctech_send(ltctech, cmd, NULL, 0) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}
	
	if(ltctech_recv(ltctech, cmd, buf, CORE_SIZE) != CORE_SIZE)
	{
		mutex_unlock(ltctech->device_mutex);
		return (false);
	}

	mutex_unlock(ltctech->device_mutex);

	if(buf[0] != LTCTECH_GET_CORE)
	{
		return (false);
	}

	for(miner_index = 0; miner_index < LTCTECH_MINER_MAX; miner_index++)
	{
		if(info->miner_sta & (0x1 << miner_index))
		{
			info->corenum[miner_index] = buf[miner_index + 1];

			if((int8_t)buf[1 + miner_index] > info->miner_core_num)
			{
				applog(LOG_ERR, "[%s X]:ERROR  - ltctech_get_core():Miner_ID=%d,Target  CoreNum=%d", ltctech->drv->name, miner_index, info->miner_core_num);
				applog(LOG_ERR, "[%s X]:ERROR  - ltctech_get_core():Miner_ID=%d,Receive CoreNum=%d", ltctech->drv->name, miner_index, info->corenum[miner_index]);
				return (false);
			}
		}
	}

	return (true);
}

static bool ltctech_hw_reset(struct cgpu_info *ltctech)
{
	int		result;
	uint8_t	status;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	if(info->product_type != LTCTECH_PRODUCT_TYPE_RAPTOR_I)		// Raptor-I's firmware have not this function
	{
		ltctech_usb_clear(ltctech);
	
		result = ltctech_send_cmd(ltctech, LTCTECH_HW_RESET, NULL, 0, &status);
		if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
		{
//			applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_hw_reset():result=%d,status=%d", ltctech->drv->name, ltctech->device_id, result, status);
			return (false);
		}
	}

	return (true);
}

static bool ltctech_stop_miner(struct cgpu_info *ltctech)
{
	int		result;
	uint8_t	status;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	if(info->product_type != LTCTECH_PRODUCT_TYPE_RAPTOR_I)		// Raptor-I's firmware have not this function
	{
		result = ltctech_send_cmd(ltctech, LTCTECH_STOP_MINER, NULL, 0, &status);
		if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
		{
//			applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_stop_miner():result=%d,status=%d", ltctech->drv->name, ltctech->device_id, result, status);
			return (false);
		}
	}

	return (true);
}

static bool ltctech_start_miner(struct cgpu_info *ltctech)
{
	int		result;
	uint8_t	status;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	if(info->product_type != LTCTECH_PRODUCT_TYPE_RAPTOR_I)		// Raptor-I's firmware have not this function
	{
		result = ltctech_send_cmd(ltctech, LTCTECH_START_MINER, NULL, 0, &status);
		if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
		{
//			applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_start_miner():result=%d,status=%d", ltctech->drv->name, ltctech->device_id, result, status);
			return (false);
		}
	}

	return (true);
}

static bool ltctech_initialize(struct cgpu_info *ltctech)
{
	struct ltctech_info *info;

	ltctech->device_mutex = NULL;
	ltctech->device_data = NULL;

	info = calloc(sizeof(struct ltctech_info), 1);
	if(!info)
	{
		return (false);
	}

	memset(info, 0, sizeof(struct ltctech_info));

	ltctech->device_data = info;

	ltctech->device_mutex = calloc(1, sizeof(*(ltctech->device_mutex)));

	if(!ltctech->device_mutex)
	{
		return (false);
	}

	mutex_init(ltctech->device_mutex);

	ltctech_usb_clear(ltctech);

	return (true);
}


static bool ltctech_finalize(struct cgpu_info *ltctech)
{
	int		miner_index;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	if(info)
	{
		for(miner_index = 0; miner_index < LTCTECH_MINER_MAX; miner_index++)
		{
			if(info->ltcgpu[miner_index])
			{
				info->ltcgpu[miner_index] = usb_free_cgpu(info->ltcgpu[miner_index]);
			}
		}
	}

	if(ltctech->device_data)
	{
		free(ltctech->device_data);
		ltctech->device_data = NULL;
	}

	if(ltctech->device_mutex)
	{
		free(ltctech->device_mutex);
		ltctech->device_mutex = NULL;
	}

	if(ltctech->name)
	{
		free(ltctech->name);
		ltctech->name = NULL;
	}
}
 
static bool ltctech_add_gpu(struct cgpu_info *ltctech)
{
	int		miner_index;
	int		miner;
	uint16_t	miner_sta;	
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;
	
	miner = 0;
	miner_sta = info->miner_sta;

	for(miner_index = 0; miner_index < LTCTECH_MINER_MAX; miner_index++)
	{
		if(miner_sta & (0x1 << miner_index))
		{
			miner++;
		}
	}

	if(miner == 0)
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_add_gpu():MinerNum=%d", ltctech->drv->name, miner);
		return false;
	}
		
	if (!add_cgpu(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_add_gpu():add_cgpu()", ltctech->drv->name);
		return false;
	}

	update_usb_stats(ltctech);

	info->miner_cnt = miner;
	
	return true;
}


static struct cgpu_info *ltctech_detect_one(struct libusb_device *dev, struct usb_find_devices *found)
{
	int		clock	= LTCTECH_CORE_CLOCK_DEF; 
	int		temp	= LTCTECH_TEMP_DEF;
	double	diff		= LTCTECH_DIFF_DEF; 

	char		serial[LTCTECH_SERIAL_NUMBER_SIZE + 1];

	struct	cgpu_info *ltctech;
	struct	ltctech_info *info;

	applog(LOG_NOTICE, "---------------------------------------------------------");
	memset(serial, 0, sizeof(serial));
	ltctech = usb_alloc_cgpu(&ltctech_drv, 1);
	if(!ltctech)
	{
		quit(1, "[%s X]:Quit   - ltctech_detect_one():Failed to usb_alloc_cgpu()", ltctech_drv.name);
		applog(LOG_NOTICE, "---------------------------------------------------------");
	}

	drv_name	= ltctech_drv.dname;
	drv_ver		= ltctech_drv.drv_ver;
	drv_date	= ltctech_drv.drv_date;

	if(!usb_init(ltctech, dev, found))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - usb_init():", ltctech_drv.name);
		goto out;
	}

	applog(LOG_NOTICE, "[%s X]:NOTICE - ltctech_detect_one():", ltctech_drv.name);

	if(!ltctech_initialize(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_initialize():", ltctech_drv.name);
		goto dev_err;
	}

	if(!ltctech_reset(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_reset():", ltctech_drv.name);
		goto dev_err;
	}

	if(!ltctech_getinfo(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_getinfo():", ltctech_drv.name);
		goto dev_err;
	}

	info = (struct ltctech_info *)ltctech->device_data;

	memcpy(serial, info->serial, LTCTECH_SERIAL_NUMBER_SIZE);

	applog(LOG_NOTICE, "[%s X]:NOTICE - S:%s, P:%s detected", ltctech->drv->name, serial, info->product_name);

	if(!ltctech_get_core(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_get_core():", ltctech_drv.name);
		goto dev_err;
	}

	if(opt_device_options != NULL)
	{
		sscanf(opt_device_options, "%d:%d", &clock, &temp);
		applog(LOG_NOTICE, "[%s X]:NOTICE - S-Options[scantime=%d,clock=%d,temp=%d]", ltctech->drv->name, opt_scantime, clock, temp);
	}

	if(!ltctech_set_clock(ltctech, clock))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_set_clock():", ltctech_drv.name);
		goto dev_err;
	}

	if(!ltctech_set_temp(ltctech, temp))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_set_temp():", ltctech_drv.name);
		goto dev_err;
	}

	if(!ltctech_set_diff(ltctech, diff))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_set_diff():", ltctech_drv.name);
		goto out;
	}

	if(!ltectech_get_status(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltectech_get_status():", ltctech_drv.name);
		goto dev_err;
	}

	if(!ltctech_add_gpu(ltctech))
	{
		applog(LOG_ERR, "[%s X]:ERROR  - ltctech_add_gpu():", ltctech_drv.name);
		goto dev_err;
	}

	if(info->product_type == LTCTECH_PRODUCT_TYPE_RAPTOR_II)
	{
		ltctech_get_clock(ltctech);
	}
	else			// For Raptor-I which have not this function
	{

	}

	ltctech_cal_working_time(ltctech);

	applog(LOG_NOTICE, "[%s %d]:NOTICE - ltctech_detect_one(): O.K. and added", ltctech->drv->name, ltctech->device_id);
	applog(LOG_NOTICE, "---------------------------------------------------------");

	return ltctech;

dev_err:
	ltctech_finalize(ltctech);
	applog(LOG_NOTICE, "[%s X]:ERROR  - ltctech_detect_one():Fail to initialize", ltctech_drv.name);

out:
	usb_uninit(ltctech);
	ltctech = usb_free_cgpu(ltctech);
	applog(LOG_NOTICE, "---------------------------------------------------------");

	return (NULL);
}


static void ltctech_detect(bool __maybe_unused hotplug)
{
	usb_detect(&ltctech_drv, ltctech_detect_one);
}

static struct thr_info* ltctech_get_miner_thread(struct cgpu_info *ltctech, int miner_id)
{
	int		thread_index;
	struct	cgpu_info *cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;
	struct	thr_info *thr_find = NULL;
	
	for(thread_index = 0; thread_index < mining_threads; thread_index++)
	{
		thr_find = get_thread(thread_index);
		if(!thr_find)
		{
			continue;
		}
		
		cgpu = thr_find->cgpu;
		if(!cgpu)
		{
			continue;
		}

		if(cgpu == ltctech)
		{
			return thr_find;
		}
	}

	return NULL;
}

static bool ltctech_init(struct thr_info *thr)
{
	struct	cgpu_info *ltctech = thr->cgpu;

	applog(LOG_INFO, "[%s %d]:INFO   - ltctech_init():", ltctech->drv->name, ltctech->device_id);
	return (true);
}


static void ltctech_reinit(struct cgpu_info *ltctech)
{
	applog(LOG_INFO, "[%s %d]INFO   - ltctech_reinit():", ltctech->drv->name, ltctech->device_id);
}


static struct api_data* ltctech_api_stats(struct cgpu_info *cgpu)
{
	struct	ltctech_info *info = (struct ltctech_info *)cgpu->device_data;
	struct	api_data *root = NULL;
	int		hw_ver, core_ver, fw_ver, core_total_cnt;

	hw_ver = info->hw_ver;
	root = api_add_int(root, "hw_version", &hw_ver, true);

	fw_ver = info->fw_ver;
	root = api_add_int(root, "fw_version", &fw_ver, true);

	core_ver = info->core_ver;
	root = api_add_int(root, "core_version", &core_ver, true);

	core_total_cnt = info->core_total_cnt;
	root = api_add_int(root, "core_count", &core_total_cnt, true);

	root = api_add_int(root, "clock", &(info->clock), false);
	root = api_add_int(root, "temp", &(info->temp), false);

	return (root);
}


static char* ltctech_set_device(struct cgpu_info *ltctech, char *option, char *setting, char *replybuf)
{
	int		val;

	applog(LOG_INFO, "[%s %d]:INFO   - ltctech_set_device():", ltctech->drv->name, ltctech->device_id);

	if(strcasecmp(option, "help") == 0)
	{
		sprintf(replybuf, "clock: range %d-%d", LTCTECH_CORE_CLOCK_MIN, LTCTECH_CORE_CLOCK_MAX);
		return (replybuf);
	}

	if(strcasecmp(option, "clock") == 0)
	{
		if(!setting || !*setting)
		{
			sprintf(replybuf, "missing clock setting");
			return (replybuf);
		}

		val = atoi(setting);
		if(val < LTCTECH_CORE_CLOCK_MIN || val > LTCTECH_CORE_CLOCK_MAX)
		{
			sprintf(replybuf, "invalid clock: '%s' valid range %d-%d", setting, LTCTECH_CORE_CLOCK_MIN, LTCTECH_CORE_CLOCK_MAX);
			return (replybuf);
		}

		if(ltctech_set_clock(ltctech, val))
		{
			return (NULL);
		}
		else
		{
			sprintf(replybuf, "Set clock failed");
			return (replybuf);
		}

		return (NULL);
	}

	sprintf(replybuf, "Unknown option: %s", option);

	return (replybuf);
}


static void ltctech_get_statline_before(char *buf, size_t bufsiz, struct cgpu_info *ltctech)
{
	int		num = 0, minernum = 0;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	tailsprintf(buf, bufsiz, "[MI#%2d CO#%2d][H:%d.%d F:%d.%d C:%d.%d][%3uMHz %2d(%2d)C]",
		info->miner_cnt, info->core_total_cnt,
		(ltctech->hw_ver >> 8) & 0xFF, (ltctech->hw_ver) & 0xFF, (ltctech->fw_ver >> 8) & 0xFF, (ltctech->fw_ver) & 0xFF, (ltctech->core_ver >> 8) & 0xFF, (ltctech->core_ver) & 0xFF,
		info->clock, info->temp, info->temp_max);
}

static void ltctech_shutdown(struct thr_info *thr)
{
	int		result;
	uint8_t	status;

	struct	cgpu_info *ltctech = thr->cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;
	
	applog(LOG_NOTICE, "[%s %d]:NOTICE - ltctech_shutdown():", ltctech->drv->name, ltctech->device_id);
	
	result = ltctech_send_cmd(thr->cgpu, LTCTECH_SET_IDLE, NULL, 0, &status);
	if((result != LTCTECH_RESULT_OK) || (status != LTCTECH_RESULT_OK))
	{
//		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_shutdown():result=%d,status=%d", ltctech->drv->name, ltctech->device_id, result, status);
	}

	ltctech->shutdown = true;
	ltctech_info_clear(info);
}

static void *ltctech_get_result(void *cgpu)
{
	int		amount;
	unsigned char work_id;
	char find[FIND_NONCE_SIZE];
	uint32_t nonce;
	struct	cgpu_info *ltctech = (struct cgpu_info *)cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;
	struct	thr_info *find_thr;
	char		threadname[24];
	struct timeval now;
	int elapsed; 
	
	snprintf(threadname, sizeof(threadname), "ltctech_recv/%d", ltctech->device_id);
	RenameThread(threadname);
	
	while(likely(!ltctech->shutdown && !ltctech->usbinfo.nodev))
	{
		amount = ltctech_recv(ltctech, LTCTECH_GET_WORK, find, FIND_NONCE_SIZE);

		if((amount < 0) && (amount != LIBUSB_ERROR_TIMEOUT))
		{
			applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_get_result():amount=%d", ltctech->drv->name, ltctech->device_id, amount);
			break;
		}
		
		if(amount >= FIND_NONCE_SIZE)
		{
			nonce = *((uint32_t*)find);
			work_id = (unsigned char)find[5];

			if((info->ltcwork[0] == NULL) || (work_id != info->ltcwork[0]->work_id))
			{
#if	0	// Blocked by HKS: Do not display message for performance
				applog(LOG_NOTICE, "---------------------------------------------------------");
				applog(LOG_NOTICE, "[%s %d]:NOTICE - Work_ID:Send=%d,Receive=%d", ltctech->drv->name, ltctech->device_id, info->ltcwork[0]->work_id, work_id);
				applog(LOG_NOTICE, "---------------------------------------------------------");
#endif
				workid_mismatch++;
			}
			else
			{	
				find_thr = ltctech_get_miner_thread(ltctech, 0);
				if(!find_thr)
				{
					applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_get_result():Not found mining thread", ltctech->drv->name, ltctech->device_id);
				}
				else
				{	
					if(submit_nonce(find_thr, info->ltcwork[0], nonce) == false)
					{
//						ltctech_print_hw_error(ltctech->drv->name, ltctech->device_id, info->ltcwork[0], nonce);
					}
				}
			}
		}

		cgtime(&now);
		elapsed = ms_tdiff(&now, &info->tv_update); 
		if(elapsed > TEMP_UPDATE_TIME)
		{
			ltctech_get_temp(ltctech);

			info->tv_update = now;	
		}				
	}

	return NULL;
}


static bool ltctech_prepare(struct thr_info *thr)
{
	struct	cgpu_info *ltctech = thr->cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;	
	struct	timeval now;

	applog(LOG_INFO, "[%s %d]:INFO   - ltctech_prepare():", ltctech->drv->name, ltctech->device_id);
	
	cgtime(&now);
	get_datestamp(ltctech->init, sizeof(ltctech->init), &now);
	
	if(info->getwork_thr == NULL)
	{
		info->getwork_thr = calloc(sizeof(pthread_t), 1);
		if (pthread_create(info->getwork_thr, NULL, ltctech_get_result, (void*)ltctech))
		{
			quit(1, "[%s %d]:Quit   - ltctech_prepare(): Failed to create get work thread", ltctech->drv->name, ltctech->device_id);
		}
	}

	return (true);
}


static uint64_t ltctech_send_work(struct thr_info *thr, struct work *work)
{
	struct	cgpu_info *ltctech = thr->cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	char		buf[BUF_SIZE];
	int		len;
	int		target = 0;

	applog(LOG_INFO, "[%s %d]:INFO   - ltctech_send_work():", ltctech->drv->name, ltctech->device_id);

	// Changed by HKS: For difficult tunning. It is very important for high difficulty performance, so do not change this option
	work->device_diff = MIN(info->working_diff, work->work_difficulty);
	set_target(work->device_target, work->device_diff);

	if(info->ltcwork[0])
	{
		free_work(info->ltcwork[0]);
	}
	
	info->ltcwork[0] = copy_work(work);
	info->ltcwork[0]->work_id = (unsigned char)(info->ltcwork[0]->id & 0xFF);

	*((uint32_t *)(info->ltcwork[0]->data + NONCE_POS)) = info->ltcwork[0]->nonce;

	memcpy(&buf[0], info->ltcwork[0]->data, WORK_SIZE);

	target = *((unsigned int*)(&info->ltcwork[0]->device_target[28]));
	target = htobe32(target);

	memcpy(&buf[TARGET_POS], &target, TARGET_SIZE);

	buf[MINER_ID_POS]	= LTCTECH_MINER_ID_ALL;
	buf[WORK_ID_POS]	= info->ltcwork[0]->work_id;
	len					= WORK_SIZE + TARGET_SIZE + MINER_ID_SIZE + WORK_ID_SIZE;

	mutex_lock(ltctech->device_mutex);
	if (ltctech_send(ltctech, LTCTECH_SEND_WORK, buf, len) != true)
	{
		mutex_unlock(ltctech->device_mutex);
		applog(LOG_ERR, "[%s %d]:ERROR  - ltctech_send_work():", ltctech->drv->name, ltctech->device_id);
		return (false);
	}

	cgtime(&info->tv_workstart);
	mutex_unlock(ltctech->device_mutex);

	return (true);
}


static bool ltctech_prepare_work(struct thr_info *thr, struct work *work)
{
	struct	cgpu_info *ltctech = thr->cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	applog(LOG_INFO, "[%s %d]:INFO   - ltctech_prepare_work():work_restart=%d,work_update=%d", ltctech->drv->name, ltctech->device_id, thr->work_restart, thr->work_update);
	
	if((ltctech->usbinfo.nodev == true) || (ltctech->shutdown == true))
	{
		return (false);
	}

	return (ltctech_send_work(thr, work));
}


static int64_t ltctech_scanhash(struct thr_info *thr, struct work *work, int64_t max_nonce)
{
	struct	cgpu_info *ltctech = thr->cgpu;
	struct	ltctech_info *info = (struct ltctech_info *)ltctech->device_data;

	struct	timeval now;
	int64_t	elapsed;
	int64_t	hashes = 0;

	applog(LOG_INFO, "[%s %d]:INFO   - ltctech_scanhash():", ltctech->drv->name, ltctech->device_id);

	if(work->pool->sdiff != work->work_difficulty)
	{
		work->pool->swork.clean = true;
	}	

	cgsleep_ms(10);		// Changed by HKS: 100 -> 10, it is need to calcaute hashrate

	cgtime(&now);	

#if	1	// Inserted by HKS: For testing to decreas the WIDE rate
	elapsed = ms_tdiff(&now, &info->tv_workstart);

	if(info->working_time < elapsed)
	{
		struct	work	*work_update;

		applog(LOG_INFO, "[%s %d]:INFO   - ltctech_scanhash():Work Update:working_time=%dmin %dsec, elapsed=%dmin %dsec",
					ltctech->drv->name, ltctech->device_id, info->working_time / (60 * 1000), info->working_time % (60 * 1000), (elapsed / 1000) / 60, (elapsed / 1000) % 60);

		work_update = get_work(thr, thr->id);

		work_update->nonce = 0;

		ltctech_send_work(thr, work_update);
	}
#endif

	elapsed = ms_tdiff(&now, &info->tv_hashrate);

	if(elapsed > 1000)
	{
		if((info->tv_hashrate.tv_usec == 0) && (info->tv_hashrate.tv_sec == 0))
		{
			info->tv_hashrate = now;
			return 0;
		}

		info->tv_hashrate = now;

		// Changed by HKS: Calibrate the hash rate to display for adjusting the real core performance
		hashes = ((15 * info->clock * info->core_total_cnt * elapsed) / 2) +
				((((15 * info->clock * info->core_total_cnt * elapsed) / 2) / 100) *  info->hashrate_offset);
	}

	return hashes;
}


struct device_drv ltctech_drv =
{
	.drv_id				= DRIVER_ltctech,
	.dname				= "HLT_Ltctech",
	.name				= "HLT",
	.drv_ver				= LTCTECH_DRIVER_VER,
	.drv_date				= LTCTECH_DRIVER_DATE,
	.drv_detect			= ltctech_detect,
	.get_statline_before	= ltctech_get_statline_before,
	.get_api_stats			= ltctech_api_stats,
	.identify_device		= ltctech_identify,
	.set_device			= ltctech_set_device,
	.thread_prepare		= ltctech_prepare,
	.thread_init			= ltctech_init,
	.reinit_device			= ltctech_reinit,	
	.thread_shutdown		= ltctech_shutdown,
	.prepare_work		= ltctech_prepare_work,
	.scanhash			= ltctech_scanhash,
	.hw_reset			= ltctech_hw_reset,
	.stop_miner			= ltctech_stop_miner,
	.start_miner			= ltctech_start_miner,
	.max_diff				= 65536
};

static void ltctech_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce)
{
	char		*twork;
	char		twork_data[BUF_SIZE];
	int		twork_index;
	int		display_size = 16;		// 16 Bytes

	applog(LOG_ERR, "---------------------------------------------------------");
	applog(LOG_ERR, "[%s %d]:ERROR  - Nonce = 0x%X,Work_ID = %3d", drv_name, device_id, nonce, work->work_id);
	twork = bin2hex(work->data, WORK_SIZE);						// Multiply 2 for making string in bin2hex()
	for(twork_index = 0; twork_index < (WORK_SIZE * 2); twork_index += (display_size * 2))
	{
		snprintf(twork_data, (display_size * 2) + 1, "%s", &twork[twork_index]);
		applog(LOG_ERR, "Work Data      = %s", twork_data);
	}
	free(twork);
	twork = bin2hex(work->device_target, DEVICE_TARGET_SIZE);		// Multiply 2 for making string in bin2hex()
	for(twork_index = 0; twork_index < (DEVICE_TARGET_SIZE * 2); twork_index += (display_size * 2))
	{
		snprintf(twork_data, (display_size * 2) + 1, "%s", &twork[twork_index]);
		applog(LOG_ERR, "Device Target  = %s", twork_data);
	}
	free(twork);
	applog(LOG_ERR, "---------------------------------------------------------");
}


#if LTCTECH_TEST_MODE
char dataScrypt[80] =  { 0x00, 0x00, 0x00, 0x01, 0xa0, 0x53, 0x30, 0x66, 0x64, 0x26, 0xe5, 0xdc, 0x2c, 0x5f, 0xed, 0xad, 0x53, 0x14, 0x2a, 0x82, 0xff, 0x7f, 0xb8, 0x86, 0x4f, 0x74, 0xf3, 0x0b, 0xc0, 0x3d, 0x5c, 0x25, 0x97, 0x03, 0x3c, 0x0e, 0xad, 0x46, 0x1d, 0x2a, 0x0b, 0xf3, 0x9c, 0xc4, 0x9f, 0xb5, 0xed, 0x9d, 0x34, 0xd4, 0xa0, 0xcf, 0x6e, 0xe2, 0x7d, 0x2e, 0xba, 0x18, 0x74, 0x92, 0x77, 0xa4, 0xd2, 0xa6, 0xa5, 0xcb, 0xf1, 0xc0, 0x52, 0xf3, 0x2c, 0x57, 0x1c, 0x12, 0x66, 0xc9, 0x5e, 0x37, 0x0b, 0x00 };
char targetScrypt[4] = { 0xff, 0x03, 0x00, 0x00 };
uint32_t nonceScrypt = 735070;

char dataX11[80] =  { 0x00,0x00,0x00,0x02,0x75,0xd6,0x91,0x99,0x65,0x07,0x7e,0x96,0xb8,0x04,0xb1,0xbf,0x77,0x8a,0xe9,0x2a,0x6d,0xe7,0x5f,0xeb,0x56,0x91,0x11,0x00,0x00,0x04,0xbe,0x72,0x00,0x00,0x00,0x00,0x62,0x22,0x72,0xad,0xf6,0x99,0x66,0x75,0xf2,0xa0,0xf9,0xe5,0x54,0xb3,0x67,0x54,0xfb,0x40,0xf8,0x1f,0x2d,0xad,0x5e,0xd8,0x4a,0x34,0x56,0x09,0xd7,0x58,0x2c,0xe9,0x53,0x95,	0x16,0xe7,0x1b,0x14,0x76,0x0a,0x00,0x10,0x6b,0x0d };
char targetX11[4] = { 0x00,0x00,0x00,0xff };
uint32_t nonceX11 = 225120256;	// 0xd6b1000

char dataX13[80] =  { 0x00,0x00,0x00,0x02,0x5b,0x4a,0xbb,0x46,0x95,0x9d,0x93,0xd0,0x49,0x1a,0x8c,0x97,0xb0,0x02,0x37,0x29,0x5d,0x1e,0xf8,0xfd,0xe0,0x74,0x2c,0xf7,0x00,0xdd,0x5c,0xb2,0x00,0x00,0x00,0x00,0x56,0xc0,0x2f,0x12,0x82,0x24,0xd3,0xb8,0xe9,0x37,0x67,0x9f,0x9d,0x00,0x10,0x00,0x2e,0x32,0x02,0x6b,0xf2,0x9d,0x22,0xd5,0x30,0x68,0xcb,0x13,0xc0,0x14,0x4d,0xa5,0x53,0x95,	0x89,0xad,0x1c,0x02,0xac,0x3d,0x00,0x0a,0x1e,0xf1 };
char targetX13[4] = { 0xff,0x00,0x00,0x00 };
uint32_t nonceX13 = 4045277696;	// 0xf11e0a00

static void ltctech_set_testdata(struct work *work)
{
	char		*data;
	char		*target;
	uint32_t	nonce;

	switch(kernel)
	{
		case KL_SCRYPT:
			data = dataScrypt;
			target = targetScrypt;
			nonce = nonceScrypt;
			break;
		case KL_X11MOD:
			data = dataX11;
			target = targetX11;
			nonce = nonceX11;
			break;
		case KL_X13MOD:
			data = dataX13;
			target = targetX13;
			nonce = nonceX13;
			break;
		case KL_X15MOD:
		case KL_NONE:
		default:
			return;
	}

	memcpy(&work->data[0], data, 80);
	memcpy(&work->device_target[28], target, 4);
	work->nonce = nonce;
}


static void ltctech_print_hash(struct work *work, uint32_t nonce)
{
	uint32_t	*work_nonce = (uint32_t *)(work->data + 64 + 12);
	uint32_t	*ohash = (uint32_t *)(work->hash);
	unsigned char hash_swap[32], target_swap[32];
	char		*hash_str, *target_str;

	*work_nonce = htole32(nonce);

	switch(kernel)
	{
		case KL_SCRYPT:
			scrypt_regenhash(work);
			break;
		case KL_X11MOD:
			darkcoin_regenhash(work);
			break;
		case KL_X13MOD:
			marucoin_regenhash(work);
			break;
		case KL_X15MOD:
		case KL_NONE:
		default:
			regen_hash(work);
			break;
	}

	swab256(hash_swap, work->hash);
	swab256(target_swap, work->target);
	hash_str = bin2hex(hash_swap, 32);
	target_str = bin2hex(target_swap, 32);

	applog(LOG_ERR, "nonce : %x(%u) => Hash : %08x : %08x : %08x : %08x : %08x : %08x : %08x : %08x",
	   					nonce, nonce, ohash[0], ohash[1], ohash[2], ohash[3], ohash[4], ohash[5], ohash[6], ohash[7]);
	applog(LOG_DEBUG, "LTCTech Hash  : %s", hash_str);
	applog(LOG_DEBUG, "LTCTech Target: %s", target_str);

	free(hash_str);
	free(target_str);
}
#endif

