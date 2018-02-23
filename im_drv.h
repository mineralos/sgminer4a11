#ifndef _IM_DRV_H_
#define _IM_DRV_H_

#define PLATFORM_ZYNQ_SPI_G9    (0x01)
#define PLATFORM_ZYNQ_SPI_G19   (0x02)
#define PLATFORM_ZYNQ_HUB_G9    (0x03)
#define PLATFORM_ZYNQ_HUB_G19   (0x04)
#define PLATFORM_SOC            (0x10)

#define SPI_SPEED_390K          (0)
#define SPI_SPEED_781K          (1)
#define SPI_SPEED_1M            (2)
#define SPI_SPEED_3M            (3)
#define SPI_SPEED_6M            (4)
#define SPI_SPEED_9M            (5)


extern int g_platform;
extern int g_miner_type;
extern int g_chain_num;
extern int g_chip_num;


extern bool sys_platform_init(int platform, int miner_type, int chain_num, int chip_num);

extern void sys_platform_exit();


extern bool inno_set_spi_speed(unsigned char chain_id, int index);

extern bool inno_cmd_reset(unsigned char chain_id, unsigned char chip_id, unsigned char *in, unsigned char *out);

extern uint8_t inno_cmd_bist_start(unsigned char chain_id, unsigned char chip_id);

extern bool inno_cmd_bist_collect(unsigned char chain_id, unsigned char chip_id);

extern bool inno_cmd_bist_fix(unsigned char chain_id, unsigned char chip_id);

extern bool inno_cmd_write_register(unsigned char chain_id, unsigned char chip_id, unsigned char *reg, int len);

extern bool inno_cmd_read_register(unsigned char chain_id, unsigned char chip_id, unsigned char *reg, int len);

extern bool inno_cmd_read_write_reg0d(unsigned char chain_id, unsigned char chip_id, unsigned char *in, int len, unsigned char *out);

extern bool inno_cmd_read_result(unsigned char chain_id, unsigned char chip_id, unsigned char *res, int len);

extern bool inno_cmd_write_job(unsigned char chain_id, unsigned char chip_id, unsigned char *job, int len);

extern bool inno_cmd_auto_nonce(unsigned char chain_id, int mode, int len);

extern bool inno_cmd_read_nonce(unsigned char chain_id, unsigned char *res, int len);


extern void inno_set_power_en(unsigned char chain_id, int val);

extern void inno_set_start_en(unsigned char chain_id, int val);

extern void inno_set_reset(unsigned char chain_id, int val);

extern void inno_set_led(unsigned char chain_id, int val);

extern int inno_get_plug(unsigned char chain_id);

extern void inno_set_vid(unsigned char chain_id, int val);




#endif
