#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "asic_inno.h"
#include "asic_inno_cmd.h"
#include "asic_inno_clock.h"

#define CMD0D_LEN    		 7
#define CMD0D_RD_DATA_LEN    (CMD0D_LEN-2)


const unsigned short wCRCTalbeAbs[] =
{
	0x0000, 0xCC01, 0xD801, 0x1400, 
	0xF001, 0x3C00, 0x2800, 0xE401, 
	0xA001, 0x6C00, 0x7800, 0xB401, 
	0x5000, 0x9C01, 0x8801, 0x4400,
};


unsigned short CRC16_2(unsigned char* pchMsg, unsigned short wDataLen)
{
	volatile unsigned short wCRC = 0xFFFF;
	unsigned short i;
	unsigned char chChar;

	for (i = 0; i < wDataLen; i++){
		chChar = *pchMsg++;
		wCRC = wCRCTalbeAbs[(chChar ^ wCRC) & 15] ^ (wCRC >> 4);
		wCRC = wCRCTalbeAbs[((chChar >> 4) ^ wCRC) & 15] ^ (wCRC >> 4);
	}

	return wCRC;
}



static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[512];
	char *pos = line;
	int i;
	if (len < 1)
	{
		return;
	}

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) 
	{
		if (i > 0 && (i % 32) == 0) 
		{
			applog(LOG_INFO, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_WARNING);
}

void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

void flush_spi(struct A1_chain *pChain)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));

	spi_write_data(ctx, spi_tx, 64);
}

bool spi_send_zero(struct spi_ctx *ctx, uint8_t *txbuf, int len)
{
	bool ret;
	int index = 0;
	uint8_t spi_tx[256];
	uint8_t spi_rx[256];

	memset(spi_tx, 0, sizeof(spi_tx));
	memcpy(spi_tx, txbuf, len);
	
	do{
		memset(spi_rx, 0, sizeof(spi_rx));
		ret = spi_write_data(ctx, spi_tx + index, 2);
		if(!ret)
		{
			return false;
		}		
		
		index = index + 2;
	}while(index < len);
	
	return true;
}


bool spi_send_data(struct spi_ctx *ctx, uint8_t *txbuf, int len)
{
	bool ret;
	int index = 0;
	uint8_t spi_tx[256];
	uint8_t spi_rx[256];

	memset(spi_tx, 0, sizeof(spi_tx));
	memcpy(spi_tx, txbuf, len);
	applog(LOG_DEBUG,"%s,%d\n",__FUNCTION__,__LINE__);
	do{
		memset(spi_rx, 0, sizeof(spi_rx));
		ret = spi_write_data(ctx, spi_tx + index, 2);
		if(!ret){
			return false;
		}		
		
		index = index + 2;
	}while(index < len);
	
	return true;
}


bool spi_send_command(struct A1_chain *pChain, uint8_t cmd, uint8_t chip_id, uint8_t *buff, int len)
{
	int tx_len;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	applog(LOG_DEBUG,"%s,%d\n",__FUNCTION__,__LINE__);
	
	assert(buff != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));
	
	spi_tx[0] = cmd;
	spi_tx[1] = chip_id;
	
	if(len > 0){
		memcpy(spi_tx + 2, buff, len);
	}
	
	tx_len = (2 + len + 1) & ~1;
	//hexdump("send: TX", spi_tx, tx_len);

	if(spi_send_data(ctx, spi_tx, tx_len)){
		return true;
	}else{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}
}


bool spi_poll_result(struct A1_chain *pChain, uint8_t cmd, uint8_t chip_id, uint8_t *buff, int len)
{
	int ret1, ret2;
	int tx_len;
	int tmp_len;
	int index,ret;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	
	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));
	
	//tx_len = ASIC_CHIP_NUM*4;
	
	if(chip_id!=0)//individual command
		tx_len =  2*(chip_id*2 - 1);
	else		  //broadcast command
	{
		if(pChain->num_chips == 0)
			tx_len =  ASIC_CHIP_NUM*4;
		else
			tx_len =  pChain->num_chips*4;
			
	}
	
	for(tmp_len = 0; tmp_len < tx_len; tmp_len += 2){
		if(!spi_read_data(ctx, spi_rx, 2)){
			applog(LOG_WARNING, "poll result: transfer fail !");
			return false;
		}
		//hexdump("poll: RX", spi_rx, 2);
		if(spi_rx[0] == cmd){
			index = 0;	
			do{
				ret = spi_read_data(ctx, spi_rx + 2 + index, 2);
				if(!ret){
					return false;
				}					
				index = index + 2;
			}while(index < len);

			//hexdump("poll: RX", spi_rx + 2, len);
			memcpy(buff, spi_rx, len);
			return true;
		}
	}
	
	return false;
}


bool inno_cmd_reset(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	//uint8_t tmp_reg[12] = {0x02,0x50,0x41,0xc2,0x00,0x00,0x00,0xa7,0xff,0x24,0x00,0x00};
	
	applog(LOG_INFO,"send command [reset] \n");

	memset(spi_tx, 0, sizeof(spi_tx));

 	//inno_cmd_write_reg(pChain, ADDR_BROADCAST, tmp_reg);

	if(!spi_send_command(pChain, CMD_RESET, chip_id, spi_tx, 2)){
		applog(LOG_WARNING, "cmd reset: send fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_RESET|0xB0, chip_id, spi_rx, 4)){
		applog(LOG_WARNING, "cmd reset: poll fail !");
		return false;
	}

	return true;
}

bool inno_cmd_resetjob(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t i,tx_len;
	uint8_t buffer[64];	

	memset(spi_tx, 0, sizeof(spi_tx));
	memset(spi_rx, 0, sizeof(spi_rx));

	spi_tx[0] = CMD_RESET;
	spi_tx[1] = chip_id;
	spi_tx[2] = 0xed;
	spi_tx[3] = 0xed;

	//tx_len = (2 + len + 1) & ~1;
	//hexdump("send: TX", spi_tx, tx_len);

	if(!spi_write_data(pChain->spi_ctx, spi_tx, 6))
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
/*
	if(!spi_poll_result(pChain, CMD_RESET, chip_id, spi_rx, 4))
	{
		applog(LOG_WARNING, "cmd reset: poll fail !");
		return false;
	}
*/
	spi_poll_result(pChain, CMD_RESET, chip_id, spi_rx, 4);

	if(inno_cmd_isBusy(pChain, chip_id) != WORK_FREE)
	{	
		return false;
	}

	return true;
}



bool inno_cmd_bist_start(struct A1_chain *pChain, uint8_t chip_id, uint8_t *num)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	applog(LOG_WARNING,"send command [bist_start] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_BIST_START, chip_id, spi_tx, 2)){
		applog(LOG_WARNING, "cmd bist start: send fail !");
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_BIST_START, chip_id, num, 4))
	{
		applog(LOG_WARNING, "cmd bist start: poll fail !");
		return false;
	}

	return true;
}

bool inno_cmd_bist_collect(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	printf("send command [bist_collect] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_BIST_COLLECT, chip_id, spi_tx, 2)){
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_BIST_COLLECT, chip_id, spi_rx, 4)){
		return false;
	}

	return true;
}


bool inno_cmd_bist_fix(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	
	applog(LOG_WARNING,"send command [bist_fix] \n");

	memset(spi_tx, 0, sizeof(spi_tx));
	if(!spi_send_command(pChain, CMD_BIST_FIX, chip_id, spi_tx, 2))
	{
		return false;
	}

	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_BIST_FIX, chip_id, spi_rx, 4))
	{
		return false;
	}

	return true;
}


bool inno_cmd_write_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg)
{
	int tx_len;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t tmp_buf[MAX_CMD_LENGTH];
	uint16_t clc_crc;
	uint8_t j;
	
	applog(LOG_INFO,"send command [write_reg] \n");
	assert(reg != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	
 	spi_tx[0] = CMD_WRITE_REG;
 	spi_tx[1] = chip_id;
 	memcpy(spi_tx+2, reg, REG_LENGTH-2);

	memset(tmp_buf, 0, sizeof(tmp_buf));
	for(j = 0; j < REG_LENGTH; j = j + 2)
	{
		tmp_buf[j + 0] = spi_tx[j + 1];
		tmp_buf[j + 1] = spi_tx[j + 0]; 	
	}
	clc_crc = CRC16_2(tmp_buf, REG_LENGTH);

	spi_tx[REG_LENGTH+0] = (uint8_t)(clc_crc >> 8);
	spi_tx[REG_LENGTH+1] = (uint8_t)(clc_crc);

	//hexdump("write reg", spi_tx, REG_LENGTH+2);
	if(!spi_write_data(pChain->spi_ctx, spi_tx, 16))
	{
		applog(LOG_WARNING, "send command fail !");
		return false;
	}
//printf("reg:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",spi_tx[0],spi_tx[1],spi_tx[2],spi_tx[3],spi_tx[4],spi_tx[5],spi_tx[6],spi_tx[7],spi_tx[8],spi_tx[9],spi_tx[10],spi_tx[11],spi_tx[12],spi_tx[13],spi_tx[14],spi_tx[15],spi_tx[16],spi_tx[17]);
	memset(spi_rx, 0, sizeof(spi_rx));
	if(!spi_poll_result(pChain, CMD_WRITE_REG, chip_id, spi_rx, REG_LENGTH+4))
	{
		applog(LOG_WARNING, "cmd write reg: poll fail !");
		return false;
	}

	return true;
}

bool inno_cmd_write_sec_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg)
{
		int tx_len;
		uint8_t spi_tx[MAX_CMD_LENGTH];
		uint8_t spi_rx[MAX_CMD_LENGTH];
		uint8_t tmp_buf[MAX_CMD_LENGTH];
		uint16_t clc_crc;
		uint8_t j;
		
		applog(LOG_INFO,"send command [write_reg] \n");
		assert(reg != NULL);
	
		memset(spi_tx, 0, sizeof(spi_tx));
		
		spi_tx[0] = CMD_READ_SEC_REG;
		spi_tx[1] = chip_id;
		memcpy(spi_tx+2, reg, REG_LENGTH-2);
	
		memset(tmp_buf, 0, sizeof(tmp_buf));
		#if 0
		for(j = 0; j < REG_LENGTH; j = j + 2)
		{
			tmp_buf[j + 0] = spi_tx[j + 1];
			tmp_buf[j + 1] = spi_tx[j + 0]; 	
		}
		#endif
		clc_crc = CRC16_2(tmp_buf, REG_LENGTH);
	
		spi_tx[REG_LENGTH+0] = (uint8_t)(clc_crc >> 8);
		spi_tx[REG_LENGTH+1] = (uint8_t)(clc_crc);
	
		//hexdump("write reg", spi_tx, REG_LENGTH+2);
		if(!spi_write_data(pChain->spi_ctx, spi_tx, 16))
		{
			applog(LOG_WARNING, "send command fail !");
			return false;
		}
		//printf("reg:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",spi_tx[0],spi_tx[1],spi_tx[2],spi_tx[3],spi_tx[4],spi_tx[5],spi_tx[6],spi_tx[7],spi_tx[8],spi_tx[9],spi_tx[10],spi_tx[11],spi_tx[12],spi_tx[13],spi_tx[14],spi_tx[15],spi_tx[16],spi_tx[17]);
		memset(spi_rx, 0, sizeof(spi_rx));
		if(!spi_poll_result(pChain, CMD_READ_SEC_REG, chip_id, spi_rx, REG_LENGTH+4))
		{
			applog(LOG_WARNING, "cmd write reg: poll fail !");
			return false;
		}
	
		return true;
}


bool inno_cmd_read_reg(struct A1_chain *pChain, uint8_t chip_id, uint8_t *reg)
{
	int i,j;
	int tx_len,clc_crc;
	int ret,index; 
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	uint8_t tmp_rx[MAX_CMD_LENGTH];
	
	struct spi_ctx *ctx = pChain->spi_ctx;
	//printf("send command [read_reg] \r\n");
	assert(reg != NULL);

	memset(spi_tx, 0, sizeof(spi_tx));
	spi_tx[0] = CMD_READ_REG;
	spi_tx[1] = chip_id;
	
	if(!spi_write_data(ctx, spi_tx, 2)){
		return false;
	}

	tx_len = ASIC_CHIP_NUM*4;
	memset(spi_rx, 0, sizeof(spi_rx));
	for(i = 0; i < tx_len; i = i + 2){
		if(!spi_read_data(ctx, spi_rx, 2)){
			applog(LOG_WARNING, "poll result: transfer fail !");
			return false;
		}
	//	hexdump("poll: RX", spi_rx, 2);
		if(spi_rx[0] == CMD_READ_REG_RESP){		
			index = 0;	
			do{
				ret = spi_read_data(ctx, spi_rx + 2 + index, 2);
				if(!ret){
					return false;
				}					
				index = index + 2;
			}while(index < REG_LENGTH);
			//hexdump("poll: reg", spi_rx + 2, REG_LENGTH);
		
			//printf("rd reg:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",spi_rx[0],spi_rx[1],spi_rx[2],spi_rx[3],spi_rx[4],spi_rx[5],spi_rx[6],spi_rx[7],spi_rx[8],spi_rx[9],spi_rx[10],spi_rx[11],spi_rx[12],spi_rx[13],spi_rx[14],spi_rx[15],spi_rx[16],spi_rx[17]);
            
			memset(tmp_rx, 0, sizeof(tmp_rx));
			for(j = 0; j < REG_LENGTH+2; j = j + 2){
				tmp_rx[j + 0] = spi_rx[j + 1];
				tmp_rx[j + 1] = spi_rx[j + 0];
			}
			//printf("tmp_rx:0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",tmp_rx[0],tmp_rx[1],tmp_rx[2],tmp_rx[3],tmp_rx[4],tmp_rx[5],tmp_rx[6],tmp_rx[7],tmp_rx[8],tmp_rx[9],tmp_rx[10],tmp_rx[11],tmp_rx[12],tmp_rx[13],tmp_rx[14],tmp_rx[15],tmp_rx[16],tmp_rx[17]);
			clc_crc = CRC16_2(tmp_rx, REG_LENGTH);
			if(clc_crc != ((spi_rx[14]<<8)|spi_rx[15])){
				printf("crc:%x,%x\n",clc_crc,(spi_rx[14]<<8)|spi_rx[15]);
			 	return false;
			}
			
			memcpy(reg, spi_rx + 2, REG_LENGTH);
			return true;
		}
	}
	
	return false;
}
bool inno_cmd_read_result(struct A1_chain *pChain, uint8_t chip_id, uint8_t *res)
{
	int i,j;
	bool ret;
	int tx_len,index;		
	uint16_t clc_crc; 
	uint16_t res_crc;
	uint8_t tmp_buf[64];
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *ctx = pChain->spi_ctx;
	//applog(LOG_DEBUG,"%s,%d\n",__FUNCTION__,__LINE__);

	applog(LOG_INFO,"send command [read_result] \r\n");
	assert(res != NULL);
	
	memset(spi_tx, 0, sizeof(spi_tx));
	spi_tx[0] = CMD_READ_RESULT;
	spi_tx[1] = chip_id;
	
	if(!spi_write_data(ctx, spi_tx, 2))
	{
		return false;
	}

	tx_len = 4 * ASIC_CHIP_NUM;

	memset(spi_rx, 0, sizeof(spi_rx));
	for(i = 0; i < tx_len; i += 2)
	{
		if(!spi_read_data(ctx, spi_rx, 2))		
		//if(!spi_transfer(ctx, spi_tx, spi_rx, 2))
		{
			return false;
		}
		if(((spi_rx[0] & 0x0f) == CMD_READ_RESULT) && (spi_rx[1] != 0))
		{
			//applog(LOG_INFO, "GET GOOD RESULT");
			index = 0;	
			do{
				ret = spi_read_data(ctx, spi_rx + 2 + index, 2);
				if(!ret)
				{
					return false;
				}					
				index = index + 2;
			}while(index < ASIC_RESULT_LEN);

			memset(tmp_buf, 0, sizeof(tmp_buf));
			for(j = 0; j < READ_RESULT_LEN; j = j + 2)
			{
				tmp_buf[j + 0] = spi_rx[j + 1];
				tmp_buf[j + 1] = spi_rx[j + 0];
			}
	//printf("spi:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",spi_rx[0],spi_rx[1],spi_rx[2],spi_rx[3],spi_rx[4],spi_rx[5],spi_rx[6],spi_rx[7]);
	//printf("id:0x%02x,0x%02x,0x%02x,0x%02x,0x%02x,0x%02x\n",tmp_buf[0],tmp_buf[1],tmp_buf[2],tmp_buf[3],tmp_buf[4],tmp_buf[5]);
			clc_crc = CRC16_2(tmp_buf, ASIC_RESULT_LEN);
			res_crc = (spi_rx[ASIC_RESULT_LEN] << 8) + (spi_rx[ASIC_RESULT_LEN+1] << 0);

			//hexdump("result: RX", spi_rx, READ_RESULT_LEN);
			if(clc_crc == res_crc)
			{
				memcpy(res, spi_rx, READ_RESULT_LEN);
				return true;
			}
			else
			{
				printf("crc is error! \r\n");
				printf("the calculate crc is 0x%4x \r\n", clc_crc);
				return false;
			}				
		}
	}

	return false;

}

uint8_t inno_cmd_isBusy(struct A1_chain *pChain, uint8_t chip_id)
{
	uint8_t buffer[REG_LENGTH];

	  
	if(!inno_cmd_read_reg(pChain, chip_id, buffer))
	{
		applog(LOG_WARNING, "read chip %d busy status error", chip_id);
		return -1;
	}
	//printf("[check busy] \r\n");
	//hexdump("reg:", buffer, REG_LENGTH);

	if((buffer[9] & 0x01) == 1)
	{
		//applog(LOG_WARNING, "chip %d is busy now", chip_id);
		return WORK_BUSY;
	}
	else
	{
		//applog(LOG_WARNING, "chip %d is free now", chip_id);
		return WORK_FREE;
	}

}



bool inno_cmd_write_job(struct A1_chain *pChain, uint8_t chip_id, uint8_t *job)
{
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
    uint8_t i;
 	struct spi_ctx *ctx = pChain->spi_ctx;
 	
	memset(spi_tx, 0, sizeof(spi_tx));
	memcpy(spi_tx, job, JOB_LENGTH);

#if DEBUG
	printf("job:0x%02x, 0x%02x\n",job[0],job[1]);
	
   for(i=2; i<96; i+=4)
   {
    printf(" %02x ",job[i+0]);
	printf(" %02x ",job[i+1]);
	printf(" %02x ",job[i+2]);
    printf(" %02x",job[i+3]);
   }
   printf("\n");
#endif

   if(!spi_write_data(ctx, spi_tx, JOB_LENGTH))
   {
   	   printf("spi_write_data JOB Failed......\n");
	   return false;
   }

   
	//printf("[write job] \r\n");
	//hexdump("job:", spi_tx, JOB_LENGTH);

	//usleep(100000);

	if(inno_cmd_isBusy(pChain, chip_id) != WORK_BUSY)
	{
		printf("wirte Job ,but inno_cmd_isBusy......\n");
		return false;
	}

	return true;

}


