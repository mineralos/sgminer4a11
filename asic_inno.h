#ifndef _ASIC_INNO_
#define _ASIC_INNO_

#define ASIC_CHAIN_NUM		1
#define ASIC_CHIP_NUM		72

#define ASIC_CHIP_A_BUCKET              (ASIC_CHAIN_NUM * ASIC_CHIP_NUM)
#define ASIC_INNO_FAN_DUTY_INIT_VAL     (0)
#define ASIC_INNO_FAN_PWM0_DEVICE_NAME  ("/dev/pwmgen0.0")
//#define ASIC_INNO_FAN_PWM1_DEVICE_NAME  ("/dev/pwmgen0.1")
#define ASIC_INNO_FAN_PWM_STEP          (10)
#define ASIC_INNO_FAN_PWM_DUTY_MAX      (100)
/* 此宏表示PWM载波频率:7KHz,经验值 */
#define ASIC_INNO_FAN_PWM_FREQ_TARGET   (7000)
/* 此宏表示分频比 分频比=50M/PWM载波频率 */
#define ASIC_INNO_FAN_PWM_FREQ          (50000000 / ASIC_INNO_FAN_PWM_FREQ_TARGET)
/* 475为80°,算力板达到此温度后,风扇开到最大 */
#define ASIC_INNO_FAN_TEMP_THRESHOLD    (475)

#define WEAK_CHIP_THRESHOLD	1
#define BROKEN_CHIP_THRESHOLD 1
#define WEAK_CHIP_SYS_CLK	(600 * 1000)
#define BROKEN_CHIP_SYS_CLK	(400 * 1000)
//#define CHIP_A6 1
#define CHIP_A7 

#include "asic_inno_cmd.h"

bool check_chip(struct A1_chain *a1, int i);
int chain_detect(struct A1_chain *a1, int idxpll);
bool abort_work(struct A1_chain *a1);

int get_current_ms(void);
bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id);
void disable_chip(struct A1_chain *a1, uint8_t chip_id);

bool get_nonce(struct A1_chain *a1, uint8_t *nonce, uint8_t *chip_id, uint8_t *job_id);
bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work, uint8_t queue_states);
void check_disabled_chips(struct A1_chain *a1);

#endif

