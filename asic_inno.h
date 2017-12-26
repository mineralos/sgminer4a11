#ifndef _ASIC_INNO_
#define _ASIC_INNO_


#include "asic_inno_cmd.h"
#include "inno_fan.h"

#define WEAK_CHIP_THRESHOLD 1
#define BROKEN_CHIP_THRESHOLD 1
#define WEAK_CHIP_SYS_CLK   (600 * 1000)
#define BROKEN_CHIP_SYS_CLK (400 * 1000)

#define CHIP_A7 

#define INNO_MINER_TYPE_FILE            "/tmp/type"
#define INNO_HARDWARE_VERSION_FILE      "/tmp/hwver"

typedef enum{
HARDWARE_VERSION_NONE = 0x00,
HARDWARE_VERSION_G9 = 0x09,
HARDWARE_VERSION_G19 = 0x13,

}hardware_version_e;

/*
typedef enum{
MINER_TYPE_NONE = 0x00,
MINER_TYPE_T0,
MINER_TYPE_T1,
MINER_TYPE_T2,
MINER_TYPE_T3,
MINER_TYPE_T4,
MINER_TYPE_T5,
MINER_TYPE_SUM,

}miner_type_e;
*/

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

