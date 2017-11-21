#ifndef _ASIC_INNO_
#define _ASIC_INNO_

#define ASIC_CHAIN_NUM		8
#define ASIC_CHIP_NUM		33
#define ACTIVE_STAT        3
#define FAN_CNT            2
//set for fan stage
#define FAN_FIRST_STAGE  25
#define FAN_SECOND_STAGE 50
#define FAN_THIRD_STAGE  75
#define FAN_FOUR_STAGE   100
#define FAN_DELTA         15

#define ASIC_CHIP_A_BUCKET              (ASIC_CHAIN_NUM * ASIC_CHIP_NUM)
#define ASIC_INNO_FAN_PWM0_DEVICE_NAME  ("/dev/pwmgen0.0")
#define ASIC_INNO_FAN_PWM1_DEVICE_NAME  ("/dev/pwmgen1.0")
#define ASIC_INNO_FAN_PWM2_DEVICE_NAME  ("/dev/pwmgen2.0")

#define ASIC_INNO_FAN_PWM_STEP          (10)
#define ASIC_INNO_FAN_PWM_DUTY_MAX      (100)

#define ASIC_INNO_FAN_PWM_FREQ_TARGET   (7000)
#define ASIC_INNO_FAN_PWM_FREQ          (50000000 / ASIC_INNO_FAN_PWM_FREQ_TARGET)
#define ASIC_INNO_FAN_TEMP_MAX_THRESHOLD (100.0f)

#define ASIC_INNO_FAN_TEMP_UP_THRESHOLD (55.0f)
#define ASIC_INNO_FAN_TEMP_DOWN_THRESHOLD (35.0f)

#define ASIC_INNO_FAN_TEMP_MARGIN_RATE  (5.0f / 100.0f)
#define ASIC_INNO_FAN_CTLR_FREQ_DIV     (0)

#define WEAK_CHIP_THRESHOLD	1
#define BROKEN_CHIP_THRESHOLD 1
#define WEAK_CHIP_SYS_CLK	(600 * 1000)
#define BROKEN_CHIP_SYS_CLK	(400 * 1000)

#define CHIP_A7 

#define INNO_MINER_TYPE_FILE			"/tmp/type"
#define INNO_HARDWARE_VERSION_FILE		"/tmp/hwver"

#define HARDWARE_VERSION_G9		(9)
#define HARDWARE_VERSION_G19	(19)

#define MINER_TYPE_T1			(1)
#define MINER_TYPE_T2			(2)
#define MINER_TYPE_T3			(3)
#define MINER_TYPE_T4			(4)



#include "asic_inno_cmd.h"

//add 0922
typedef struct{
   float highest_vol[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];    /* chip temp bits */;
   float lowest_vol[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];    /* chip temp bits */;
   float avarge_vol[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];    /* chip temp bits */; 
   int stat_cnt[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];
}inno_reg_ctrl_t;

bool inno_check_voltage(struct A1_chain *a1, int chip_id, inno_reg_ctrl_t *s_reg_ctrl);
void inno_configure_tvsensor(struct A1_chain *a1, int chip_id,bool is_tsensor);



bool check_chip(struct A1_chain *a1, int i);
int chain_detect(struct A1_chain *a1);
bool abort_work(struct A1_chain *a1);

int get_current_ms(void);
bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id);
void disable_chip(struct A1_chain *a1, uint8_t chip_id);

bool get_nonce(struct A1_chain *a1, uint8_t *nonce, uint8_t *chip_id, uint8_t *job_id);
bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work, uint8_t queue_states);
void check_disabled_chips(struct A1_chain *a1, int pllnum);
int prechain_detect(struct A1_chain *a1, int idxpll);

#endif

