#ifndef _ASIC_B29_
#define _ASIC_B29_

#include <stdint.h>
#include "elist.h"
#include "miner.h"
#include "mcompat_drv.h"
#include "mcompat_lib.h"

#define CHIP_A11
#define NO_FAN_CTRL

#ifdef CHIP_A11
#define ASIC_CHAIN_NUM          (3)
#define ASIC_CHIP_NUM           (51)    // 45
#define ASIC_CORE_NUM           (28)    // 63
#define MAX_CHIP_NUM            (ASIC_CHIP_NUM)
#define MAX_CORES               (MAX_CHIP_NUM * ASIC_CORE_NUM)

#endif


#ifdef CHIP_A11
#define JOB_LENGTH              (220)
#endif

#define NONCE_LEN               (4)

#ifdef CHIP_A12
#define BLOCK_HEADER_LEN        (80)
#define TARGET_LEN              (32)
#else
#define BLOCK_HEADER_LEN        (112)
#define TARGET_LEN              (4)
#endif

#define MAX_CMD_LENGTH          (JOB_LENGTH + ASIC_CHIP_NUM * 2 * 2)
#define REG_LENGTH              (12)


#define WEAK_CHIP_THRESHOLD     (1)
#define BROKEN_CHIP_THRESHOLD   (1)
#define WEAK_CHIP_SYS_CLK       (600 * 1000)
#define BROKEN_CHIP_SYS_CLK     (400 * 1000)
#define TEMP_UPDATE_INT_MS  5000


#ifdef CHIP_A11
#define CHIP_PLL_DEF            (30)
#define CHIP_VID_DEF            (8)
#define CHIP_VID_RUN            (5)
#define CHIP_VOL_MAX            (0.62)
#define CHIP_VOL_MIN            (0.45)
#else
#define CHIP_PLL_DEF            (120)
#define CHIP_VID_DEF            (8)
#define CHIP_VID_RUN            (12)
#define CHIP_VOL_MAX            (0.55)
#define CHIP_VOL_MIN            (0.45)
#endif

#define CHAIN_DEAD_TIME			(600)			// s

//#define USE_AUTONONCE
//#define USE_AUTOCMD0A

#define B29_MINER_TYPE_FILE            "/tmp/type"
#define B29_HARDWARE_VERSION_FILE      "/tmp/hwver"

#define DIFF_DEF				(1024)

#define LED_ON					(0)
#define LED_OFF					(1)


typedef enum{
    HARDWARE_VERSION_NONE = 0x00,
    HARDWARE_VERSION_G9 = 0x09,
    HARDWARE_VERSION_G19 = 0x13,
} hardware_version_e;
    
typedef enum{
    B29_TYPE_NONE = 0x00,
    B29_TYPE_A4,
    B29_TYPE_A5,
    B29_TYPE_A6,
    B29_TYPE_A7,
    B29_TYPE_A8,
    B29_TYPE_A9,
	B29_TYPE_A11,
    B29_TYPE_A12,
}b29_type_e;

typedef struct{
   double highest_vol[ASIC_CHAIN_NUM];    /* chip temp bits */;
   double lowest_vol[ASIC_CHAIN_NUM];    /* chip temp bits */;
   double average_vol[ASIC_CHAIN_NUM];    /* chip temp bits */; 
   int stat_val[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];
   int stat_cnt[ASIC_CHAIN_NUM][ASIC_CHIP_NUM];
}b29_reg_ctrl_t;

struct work_ent {
    struct work *work;
    struct list_head head;
};

struct work_queue {
    int num_elems;
    struct list_head head;
};

struct A1_chip {
    uint8_t reg[12];
    int num_cores;
    int last_queued_id;
    struct work *work[4];
    /* stats */
    int hw_errors;
    int stales;
    int nonces_found;
    int nonce_ranges_done;

    /* systime in ms when chip was disabled */
    int cooldown_begin;
    /* number of consecutive failures to access the chip */
    int fail_count;
    int fail_reset;
    /* mark chip disabled, do not try to re-enable it */
    bool disabled;

    /* temp */
    int temp;

    int nVol;
    
	int tunedir; // Tune direction, +/- 1

	int pll;
	int cycles;
	double product; // Hashrate product of cycles / time
	bool pllOptimal; // We've stopped tuning frequency
};

struct A1_chain {
    int chain_id;
    struct cgpu_info *cgpu;
    struct mcp4x *trimpot;
    int num_chips;
    int num_cores;
    int num_active_chips;
    int chain_skew;
    //int vid;
    uint8_t spi_tx[MAX_CMD_LENGTH];
    uint8_t spi_rx[MAX_CMD_LENGTH];
    struct spi_ctx *spi_ctx;
    struct A1_chip *chips;
    pthread_mutex_t lock;

    struct work_queue active_wq;
	bool throttle; /* Needs throttling */
	int cycles; /* Cycles used for iVid tuning */
	int tunedir; // Tune direction, -1..+1
	int pll; /* Current chain speed */
	int base_pll; /* Initial chain speed */

	int vid; /* Current actual iVid */
	double product; // Hashrate product of cycles / time
	bool VidOptimal; // We've stopped tuning voltage
	bool pllOptimal; // We've stopped tuning frequency
	bool voltagebalanced; // We've balanced voltage b/w chips

    /* mark chain disabled, do not try to re-enable it */
    bool disabled;
    uint8_t temp;
    int last_temp_time;
    int pre_heat;

	time_t lastshare;

    struct timeval tvScryptLast;
    struct timeval tvScryptCurr;
    struct timeval tvScryptDiff;
    int work_start_delay;
};

bool b29_check_voltage(struct A1_chain *a1, int chip_id, b29_reg_ctrl_t *s_reg_ctrl);
void b29_configure_tvsensor(struct A1_chain *a1, int chip_id,bool is_tsensor);
int b29_get_voltage_stats(struct A1_chain *a1, b29_reg_ctrl_t *s_reg_ctrl);

extern hardware_version_e b29_get_hwver(void);
extern b29_type_e b29_get_miner_type(void);

int get_current_ms(void);
bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id);
void disable_chip(struct A1_chain *a1, uint8_t chip_id);
void check_disabled_chips(struct A1_chain *a1);
bool check_chip(struct A1_chain *a1, int cid);
int prechain_detect(struct A1_chain *a1, int idxpll,int last_pll);
int chain_detect(struct A1_chain *a1);

bool get_nonce(struct A1_chain *a1, uint8_t *nonce, uint8_t *chip_id, uint8_t *job_id);
bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work, uint8_t queue_states);
bool abort_work(struct A1_chain *a1);

#endif

