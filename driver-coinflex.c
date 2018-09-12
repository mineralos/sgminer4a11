
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
//***   driver-coinflex.c is for X11 algorithm mining by using Han-Lab's Pantheon-XXX series miner      ***//
//=====================================================================//

//=====================================================================//
//  DRIVER_COINFLEX DEFINITION FOR X11 ALGORITHM
//  Support Product:
//      1) Pantheon-A   : Altera Stratix V E9 FPGA Chip
//                      : 1 base b'd, 10 miner b'd, 1 miner b'd includes 4EA FPGA Chip
//      2) Pantheon-AFS4    : Altera Stratix IV 530 FPGA Chip
//                      : 2 miner b'd(operating independently), 1 miner b'd includes 10EA FPGA Chip
//      3) Pantheon-CMF1 : Altera Stratix V E9 FPGA Chip
//                      :  1 base b'd, 1 core b'd, 1 core b'd includes 1EA FPGA Chip
//=====================================================================//


#include "config.h"

#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>
#include <pthread.h>
#include <error.h>

#include "driver-coinflex.h"
#include "compat.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "asic_b29.h"

#include "mcompat_chain.h"
#include "mcompat_tempctrl.h"
#include "mcompat_fanctrl.h"

#define WORK_SIZE               (80)
#define DEVICE_TARGET_SIZE      (32)
#define TARGET_POS              (80)
#define TARGET_SIZE             (4)
#define MINER_ID_POS            (84)
#define MINER_ID_SIZE           (1)
#define WORK_ID_POS             (85)
#define WORK_ID_SIZE            (1)
#define FIND_NONCE_SIZE         (6)             // For receive value from miner: 4-Bytes nonce, 1-Byte miner_id, 1-Byte work_id

#define REPLY_SIZE              (2)
#define BUF_SIZE                (128)
//#define TEMP_UPDATE_INT_MS  10000
#define CHECK_DISABLE_TIME  0

struct A1_chain *chain[MAX_CHAIN_NUM];

static uint32_t check_disbale_flag[MAX_CHAIN_NUM];

int g_reset_delay = 0xffff;
int g_miner_state = 0;
b29_reg_ctrl_t s_reg_ctrl;

/* added by yex in 20170907 */
/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */

/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
struct A1_config_options A1_config_options = {
    .ref_clk_khz = 16000, .sys_clk_khz = 800000, .spi_clk_khz = 2000,
};

/* override values with --bitmine-a1-options ref:sys:spi: - use 0 for default */
static struct A1_config_options *parsed_config_options;


static void coinflex_set_chip_reject(struct cgpu_info *cgpu)
{
	int i = 0;
    struct A1_chain *a1 = cgpu->device_data;
	a1->chips[a1->chip_id_for_nonce].reject_cnt++;
	applog(LOG_ERR, "Chain[%d] Chip[%d] reject count: %d",a1->chain_id,a1->chip_id_for_nonce,a1->chips[a1->chip_id_for_nonce-1].reject_cnt);
	//applog(LOG_ERR, "************** Reject Table **************");

	//for(i=0;i<ASIC_CHIP_NUM;i++)
		//applog(LOG_ERR, "Chain[%d] Chip[%d] reject count: %d",a1->chain_id,i+1,a1->chips[i].reject_cnt);


	//applog(LOG_ERR, "******************************************");
}

#if COINFLEX_TEST_MODE
static void coinflex_set_testdata(struct work *work);
static void coinflex_print_hash(struct work *work, uint32_t nonce);
#endif

//static void coinflex_print_hw_error(char *drv_name, int device_id, struct work *work, uint32_t nonce);
//static bool coinflex_set_algorithm(struct cgpu_info *coinflex);

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
{
    if (work == NULL)
        return false;
    struct work_ent *we = malloc(sizeof(*we));
    assert(we != NULL);

    we->work = work;
    INIT_LIST_HEAD(&we->head);
    list_add_tail(&we->head, &wq->head);
    wq->num_elems++;
    return true;
}

static struct work *wq_dequeue(struct work_queue *wq)
{
    if (wq == NULL)
        return NULL;
    if (wq->num_elems == 0)
        return NULL;
    struct work_ent *we;
    we = list_entry(wq->head.next, struct work_ent, head);
    struct work *work = we->work;

    list_del(&we->head);
    free(we);
    wq->num_elems--;
    return work;
}


/* queue two work items per chip in chain */
static bool coinflex_queue_full(struct cgpu_info *cgpu)
{
    struct A1_chain *a1 = cgpu->device_data;
    int queue_full = false;

    mutex_lock(&a1->lock);
    //  applog(LOG_NOTICE, "%d, A1 running queue_full: %d/%d",
    //     a1->chain_id, a1->active_wq.num_elems, a1->num_active_chips);

    if (a1->active_wq.num_elems >= a1->num_active_chips * 2)
        queue_full = true;
    else
        wq_enqueue(&a1->active_wq, get_queued(cgpu));

    mutex_unlock(&a1->lock);

    return queue_full;
}

static void performance_cfg(void)
{
	int i, vid;

	if (opt_A1auto) {
		/* different pll depending on performance strategy. */
		if (opt_A1_factory) {
			/* Factory mode */
			opt_A1Pll1 = CHIP_PLL_BAL;
			vid = CHIP_VID_BAL;
		} else if (opt_A1_performance) {
			/* Performance mode */
			opt_A1Pll1 = CHIP_PLL_PER;      
			vid = CHIP_VID_PER;
		} else if (opt_A1_efficient) {
			/* Efficient mode */
			opt_A1Pll1 = CHIP_PLL_EFF;
			vid = CHIP_VID_EFF;
		} else {
			/* Default */
			opt_A1Pll1 = CHIP_PLL_PER;
			vid = CHIP_VID_PER;
		}

		for (i = 0; i < MAX_CHAIN_NUM; ++i)
			opt_voltage[i] = vid;
	}
}

static void get_temperatures(struct A1_chain *a1)
{
	int i;
	int temp[MAX_CHIP_NUM] = {0};
	c_temp temp_stat;

	/* Get temperature for each chip */
	mcompat_get_chip_temp(a1->chain_id, temp);
	for (i = 0; i < a1->num_active_chips; i++)
		a1->chips[i].temp = temp[i];

	/* Get temperature stat. */
	mcompat_stat_chain_temp(temp, &temp_stat);
	a1->temp     = temp_stat.tmp_avg;
	a1->temp_max = temp_stat.tmp_hi;
	a1->temp_min = temp_stat.tmp_lo;
}

static void get_voltages(struct A1_chain *a1)
{
	int i;
	int volt[MAX_CHIP_NUM] = {0};
	c_volt volt_stat;

	/* configure for vsensor */
	mcompat_configure_tvsensor(a1->chain_id, CMD_ADDR_BROADCAST, 0);

	/* Get voltage for each chip */
	mcompat_get_chip_volt(a1->chain_id, volt);
	for (i = 0; i < a1->num_active_chips; i++)
    {
        applog(LOG_NOTICE,"chain:%d,chip:%d,vol:%d",a1->chain_id,i+1,volt[i]);
		a1->chips[i].nVol = volt[i];
       }
	/* Get voltage stat. */
	mcompat_stat_chain_volt(volt, &volt_stat);
	a1->volt	 = volt_stat.vol_avg;
	a1->volt_max = volt_stat.vol_hi;
	a1->volt_min = volt_stat.vol_lo;

	/* configure for tsensor */
	mcompat_configure_tvsensor(a1->chain_id, CMD_ADDR_BROADCAST, 1);
}

static bool check_chips(struct A1_chain *a1)
{
	int i;
	int cores[MAX_CHIP_NUM] = {0};
	char errmsg[64];

	if (mcompat_chain_get_chip_cores(a1->chain_id, a1->num_active_chips, cores)) {
		a1->num_cores = 0;
		for (i = 0; i < a1->num_active_chips; i++) {
			if (cores[i] > ASIC_CORE_NUM) {
				applog(LOG_WARNING, "chain%d: chip%d invalid core number(%d), set to default(%d)",
					a1->chain_id, i + 1, cores[i], ASIC_CORE_NUM);
				cores[i] = ASIC_CORE_NUM;
			} else if (cores[i] < ASIC_CORE_NUM) {
				applog(LOG_WARNING, "chain%d: chip%d core number(%d) < %d",
					a1->chain_id, i + 1, cores[i], ASIC_CORE_NUM);
			}

			a1->chips[i].num_cores = cores[i];
			a1->num_cores += cores[i];

			if (a1->chips[i].num_cores < BROKEN_CHIP_THRESHOLD) {
				applog(LOG_WARNING, "%d: broken chip %d with %d active cores (threshold = %d)",
					a1->chain_id, i + 1, a1->chips[i].num_cores, BROKEN_CHIP_THRESHOLD);
				a1->chips[i].disabled = true;
				a1->num_cores -= a1->chips[i].num_cores;

				sprintf(errmsg, "%d:%d", a1->chain_id, i + 1);
				if (mcompat_set_errcode(ERRCODE_INV_CORENUM, errmsg, false))
					mcompat_save_errcode();

				goto failure;
			}

			if (a1->chips[i].num_cores < WEAK_CHIP_THRESHOLD) {
				applog(LOG_WARNING, "%d: weak chip %d with %d active cores (threshold = %d)",
					a1->chain_id, i + 1, a1->chips[i].num_cores, WEAK_CHIP_THRESHOLD);

				sprintf(errmsg, "%d:%d", a1->chain_id, i + 1);
				if (mcompat_set_errcode(ERRCODE_INV_CORENUM, errmsg, false))
					mcompat_save_errcode();

				goto failure;
			}

			/* Reenable this in case we have cycled through this function more than
			 * once. */
			a1->chips[i].disabled = false;
		}

		return true;
	} else {
		sprintf(errmsg, "%d", a1->chain_id);
		if (mcompat_set_errcode(ERRCODE_INV_CORENUM, errmsg, false))
			mcompat_save_errcode();
		goto failure;
	}

failure:
	g_chain_alive[a1->chain_id] = 0;
	mcompat_chain_power_down(a1->chain_id);
	return false;
}

static bool chain_restart(struct A1_chain *a1, int retry_cnt, int retry_intval)
{
	int retries = 0;
	c_chain_param chain_param;

	applog(LOG_NOTICE, "chain%d: restarting", a1->chain_id);

	chain_param.chain_id = a1->chain_id;
	chain_param.spi_speed = SPI_SPEED_RUN;
	chain_param.pll = opt_A1Pll1;
	chain_param.vol = 0;
	chain_param.vid = opt_voltage[a1->chain_id];
	chain_param.tuning = false;
	applog(LOG_NOTICE, "\tparam->pll: %d", chain_param.pll);
	applog(LOG_NOTICE, "\tparam->vol: %d", chain_param.vol);
	applog(LOG_NOTICE, "\tparam->vid: %d", chain_param.vid);

	while(retries < retry_cnt) {
		mcompat_chain_detect_thread((void*)&chain_param);
		if (g_chain_alive[a1->chain_id] && check_chips(a1)) {
			applog(LOG_NOTICE, "chain%d: detected %d chips / %d cores",
				a1->chain_id, a1->num_active_chips, a1->num_cores);

			get_voltages(a1);

			struct timeval now;
			cgtime(&now);
			a1->lastshare = now.tv_sec;

			break;
		}

		applog(LOG_ERR, "chain%d: failed to restart, retry(%d) after %d seconds",
			a1->chain_id, ++retries, retry_intval);
		sleep(retry_intval);
	}

	return g_chain_alive[a1->chain_id];
}

static bool chain_detect_all()
{
	int i, j, cid;
	c_chain_param chain_params[MAX_CHAIN_NUM];

	applog(LOG_DEBUG, "D9 detect");

	/* Init temp ctrl */
	c_temp_cfg tmp_cfg;
	mcompat_tempctrl_get_defcfg(&tmp_cfg);
	/* Set initial target temperature lower for more reliable startup */
	tmp_cfg.tmp_target	 = 70;
	tmp_cfg.tmp_thr_pd	 = 105;
	tmp_cfg.tmp_thr_warn = 110;		/* avoid throttle during startup */
	tmp_cfg.tmp_exp_time = 2000;
	mcompat_tempctrl_init(&tmp_cfg);

	/* Init fan ctrl thread */
	c_fan_cfg fan_cfg;
	mcompat_fanctrl_get_defcfg(&fan_cfg);
	fan_cfg.preheat = false;		/* disable preheat */
	fan_cfg.fan_mode = g_auto_fan ? FAN_MODE_AUTO : FAN_MODE_MANUAL;
	fan_cfg.fan_speed_manual = g_fan_speed;
	fan_cfg.fan_speed_target = 50;
	mcompat_fanctrl_init(&fan_cfg);
//	mcompat_fanctrl_init(NULL); 	// using default cfg
	pthread_t tid;
	pthread_create(&tid, NULL, mcompat_fanctrl_thread, NULL);

	/* Register PLL map config */
	mcompat_chain_set_pllcfg(g_pll_list, g_pll_regs, PLL_LV_NUM);

	/* Determine working PLL & VID */
	performance_cfg();

	for (i = 0; i < g_chain_num; ++i) {
		cid = g_chain_id[i];
		chain_params[i].chain_id = cid;
		chain_params[i].spi_speed = SPI_SPEED_RUN;
		chain_params[i].pll = opt_A1Pll1;
		chain_params[i].vol = 0;		/* Set to 0 to disable voltage calibration */
		chain_params[i].vid = opt_voltage[cid];
		chain_params[i].tuning = false;
	}

	/* Detect all chains sequentially with fan bypass disabled during startup */
	if (0 == mcompat_chain_detect_all(0xff, chain_params, false, false))
		return false;

	for (i = 0; i < g_chain_num; ++i) {
		cid = g_chain_id[i];

		if (!g_chain_alive[cid])
			continue;

		/* Init chain object */
		chain[i] = malloc(sizeof(struct A1_chain));
		assert(chain[i] != NULL);
		memset(chain[i], 0, sizeof(struct A1_chain));

		chain[i]->chain_id = cid;
		chain[i]->pll = chain[i]->base_pll = mcompat_chain_pll_to_level(chain_params[i].pll);
		chain[i]->vid = chain_params[i].vid;
		chain[i]->num_active_chips = chain[i]->num_chips = chain_params[i].chip_num;

		/* Init chips */
		chain[i]->chips = calloc(chain[i]->num_active_chips, sizeof(struct A1_chip));
		assert (chain[i]->chips != NULL);

		/* Get core number for each chip */
		if (!check_chips(chain[i])) {
			free(chain[i]->chips);
			free(chain[i]);
			chain[i] = NULL;
			continue;
		}

		/* Init chip voltages */
		get_voltages(chain[i]);

		applog(LOG_NOTICE, "chain%d: detected %d chips / %d cores",
			cid, chain[i]->num_active_chips, chain[i]->num_cores);

		/* Init cgpu object */
		struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
		assert(cgpu != NULL);
		memset(cgpu, 0, sizeof(*cgpu));
		cgpu->drv = &coinflex_drv;
		cgpu->name = "D9.SingleChain";
		cgpu->threads = 1;
		cgpu->chainNum = cid;
		cgpu->device_data = chain[i];
		if ((chain[i]->num_chips <= MAX_CHIP_NUM) && (chain[i]->num_cores <= MAX_CORES))
			cgpu->mhs_av = (double)opt_A1Pll1 * 2ull * (chain[i]->num_cores);
		else
			cgpu->mhs_av = 0;
		cgtime(&cgpu->dev_start_tv);
		chain[i]->lastshare = cgpu->dev_start_tv.tv_sec;
		chain[i]->cgpu = cgpu;
		add_cgpu(cgpu);

		/* Other init */
		mutex_init(&chain[i]->lock);
		INIT_LIST_HEAD(&chain[i]->active_wq.head);
	}

	/* Now adjust temperature config for runtime setting */
	g_tmp_cfg.tmp_thr_warn = 100;
	g_tmp_cfg.tmp_thr_pd = 105;

	return true;
}

static void coinflex_detect(bool __maybe_unused hotplug)
{
    if (hotplug){
        return;
    }

    struct timeval test_tv;
    int j = 0;

    /* parse bimine-a1-options */
    if ((opt_bitmine_a1_options != NULL) && (parsed_config_options == NULL)) {
        int ref_clk = 0;
        int sys_clk = 0;
        int spi_clk = 0;
        int override_chip_num = 0;
        int wiper = 0;

        sscanf(opt_bitmine_a1_options, "%d:%d:%d:%d:%d",&ref_clk, &sys_clk, &spi_clk, &override_chip_num,&wiper);
        if (ref_clk != 0){
            A1_config_options.ref_clk_khz = ref_clk;
        }
        if (sys_clk != 0) {
            if (sys_clk < 100000){
                quit(1, "system clock must be above 100MHz");
            }
            A1_config_options.sys_clk_khz = sys_clk;
        }
        if (spi_clk != 0){
            A1_config_options.spi_clk_khz = spi_clk;
        }
        if (override_chip_num != 0){
            A1_config_options.override_chip_num = override_chip_num;
        }
        if (wiper != 0){
            A1_config_options.wiper = wiper;
        }

        /* config options are global, scan them once */
        parsed_config_options = &A1_config_options;
    }

    // update time
    for(j = 0; j < 64; j++) {
        cgtime(&test_tv);
        if(test_tv.tv_sec > 1000000000)
            break;

        usleep(500000);
    }

	/* Chain detect */
	if (chain_detect_all()) {
		applog(LOG_NOTICE, "D9 detect finish");
		return;
	}

	applog(LOG_ERR, "D9 detect failed");

}


static void coinflex_get_statline_before(char *buf, size_t bufsiz, struct cgpu_info *coinflex)
{
    struct A1_chain *a1 = coinflex->device_data;
    char temp[10];
    if (a1->temp != 0)
        snprintf(temp, 9, "%2dC", a1->temp);
    tailsprintf(buf, bufsiz, " %2d:%2d/%3d %s",
            a1->chain_id, a1->num_active_chips, a1->num_cores,
            a1->temp == 0 ? "   " : temp);
}



static void coinflex_flush_work(struct cgpu_info *coinflex)
{
    struct A1_chain *a1 = coinflex->device_data;
    int cid = a1->chain_id;
    //board_selector->select(cid);
    int i;
    uint8_t buffer[4] = {0};

    mutex_lock(&a1->lock);
    /* stop chips hashing current work */
  //  if (!abort_work(a1)) 
   // {
  //      applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
  //  }
    /* flush the work chips were currently hashing */
    for (i = 0; i < a1->num_active_chips; i++) 
    {
        int j;
        struct A1_chip *chip = &a1->chips[i];
        for (j = 0; j < 4; j++) 
        {
            struct work *work = chip->work[j];
            if (work == NULL)
                continue;
            //applog(LOG_DEBUG, "%d: flushing chip %d, work %d: 0x%p",
            //       cid, i, j + 1, work);
            work_completed(coinflex, work);
            chip->work[j] = NULL;
        }

        chip->last_queued_id = 0;

       // if(!mcompat_cmd_resetjob(a1->chain_id, i+1, buffer))
       // {
      //      applog(LOG_WARNING, "chip %d clear work failed", i);
      //      continue;
      //  }

        //applog(LOG_INFO, "chip :%d flushing queued work success", i);
    }
    /* flush queued work */
    //applog(LOG_DEBUG, "%d: flushing queued work...", cid);
    while (a1->active_wq.num_elems > 0) 
    {
        struct work *work = wq_dequeue(&a1->active_wq);
        assert(work != NULL);
        work_completed(coinflex, work);
    }
    mutex_unlock(&a1->lock);

}

static void overheated_blinking(int cid)
{
	// block thread and blink led
	while (42) {
		mcompat_set_led(cid, LED_OFF);
		cgsleep_ms(500);
		mcompat_set_led(cid, LED_ON);
		cgsleep_ms(500);
	}
}

volatile int g_nonce_read_err = 0;

#define MAX_CMD_FAILS		(0)
#define MAX_CMD_RESETS		(50)

static int g_cmd_fails[MCOMPAT_CONFIG_MAX_CHAIN_NUM];
static int g_cmd_resets[MCOMPAT_CONFIG_MAX_CHAIN_NUM];

static int64_t coinflex_scanwork(struct thr_info *thr)
{
    int i;
    uint8_t reg[128];
    struct cgpu_info *cgpu = thr->cgpu;
    struct A1_chain *a1 = cgpu->device_data;
    int32_t nonce_ranges_processed = 0;
	int64_t hashes = 0;

    uint32_t nonce;
    uint8_t chip_id;
    uint8_t job_id;
    bool work_updated = false;
	struct timeval now;

	char errmsg[32] = {0};

    if (a1->num_cores == 0) {
        cgpu->deven = DEV_DISABLED;
        return 0;
    }

    mutex_lock(&a1->lock);
    int cid = a1->chain_id;

    if (a1->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms()) {
        check_disbale_flag[cid]++;

        cgpu->chip_num = a1->num_active_chips;
        cgpu->core_num = a1->num_cores;

        a1->last_temp_time = get_current_ms();
    }

	cgtime(&now);

    /* poll queued results */
    while (true){
        if (!get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id)){
            break;
        }

        work_updated = true;

        if (job_id < 1 || job_id > 4){
            applog(LOG_WARNING, "%d: chip %d: result has wrong ""job_id %d", cid, chip_id, job_id);
            continue;
        }

        struct A1_chip *chip = &a1->chips[chip_id - 1];
        struct work *work = chip->work[job_id - 1];
        if (work == NULL){
            /* already been flushed => stale */
            applog(LOG_WARNING, "%d: chip %d: stale nonce 0x%08x", cid, chip_id, nonce);
            chip->stales++;
            continue;
        }
        if (!submit_nonce(thr, work, nonce)){
            applog(LOG_WARNING, "%d: chip %d: invalid nonce 0x%08x", cid, chip_id, nonce);
            chip->hw_errors++;
            /* add a penalty of a full nonce range on HW errors */
            nonce_ranges_processed--;
            continue;
        }

//        applog(LOG_INFO, "Got nonce for chain %d / chip %d / job_id %d", a1->chain_id, chip_id, job_id);

        chip->nonces_found++;
		hashes += work->device_diff;
		a1->lastshare = now.tv_sec;
    }

#ifdef USE_AUTONONCE
    mcompat_cmd_auto_nonce(a1->chain_id, 0, REG_LENGTH);   // disable autononce
#endif

	if (unlikely(now.tv_sec - a1->lastshare > CHAIN_DEAD_TIME)) {
#ifdef SINGLE_CHAIN_RESTART
		/* Restart chain */
		applog(LOG_EMERG, "A1 chain %d not producing shares for more than %d mins, restart",
			   cid, CHAIN_DEAD_TIME / 60);
		if (!chain_restart(a1, CHAIN_RESTART_RETRIES, 10)) {
			applog(LOG_ERR, "chain%d: failed to restart for %d times, SHUTDOWN", CHAIN_RESTART_RETRIES);
			cgpu->shutdown = true;
			cgpu->status = LIFE_DEAD;
			cgtime(&thr->sick);
			/* FIXME: should not be infinit loop here */
			while (42)
				sleep(10);
		}
#else
		applog(LOG_EMERG, "A1 chain %d not producing shares for more than %d mins, shutting down.",
		       cid, (CHAIN_DEAD_TIME / 60)*g_job_resets[cid]);
		/* Exit cgminer, allowing systemd watchdog to restart */
		mcompat_chain_power_down_all();
		exit(1);
#endif
	}

    /* check for completed works */
    if(a1->work_start_delay > 0)
    {
        applog(LOG_INFO, "wait for pll stable");
        a1->work_start_delay--;
    }
    else
    {
    	
		/* Clean spi buffer before read 0a reg */
		hub_spi_clean_chain(cid);

		// mcompat_cmd_reset_reg(cid);
        for (i = a1->num_active_chips; i > 0; i--) {
			if (mcompat_cmd_read_register(a1->chain_id, i, reg, REG_LENGTH)) {
				struct A1_chip *chip = NULL;
				struct work *work = NULL;

				/* Clear counter */
				g_cmd_fails[cid] = 0;
				g_cmd_resets[cid] = 0;

				uint8_t qstate = reg[9] & 0x03;
				if (qstate != 0x03)
				{
					work_updated = true;
					if(qstate == 0x0){
						chip = &a1->chips[i - 1];
						work = wq_dequeue(&a1->active_wq);

						if (work == NULL)
						  continue;

						if (set_work(a1, i, work, 0)) {
						  nonce_ranges_processed++;
						  chip->nonce_ranges_done++;
						}
					}

					chip = &a1->chips[i - 1];
					work = wq_dequeue(&a1->active_wq);

					if (work == NULL)
						continue;

					if (set_work(a1, i, work, 0)) {
						nonce_ranges_processed++;
						chip->nonce_ranges_done++;
					}
				}
			} else {
				g_cmd_fails[cid]++;
				if (g_cmd_fails[cid] > MAX_CMD_FAILS) {
					applog(LOG_ERR, "Chain %d reset spihub", cid);
					// TODO: replaced with mcompat_spi_reset()
					hub_spi_clean_chain(cid);
					g_cmd_resets[cid]++;
					if (g_cmd_resets[cid] > MAX_CMD_RESETS) {
						sprintf(errmsg, "%d", cid);
						if (mcompat_set_errcode(ERRCODE_SPI_FAIL, errmsg, false))
							mcompat_save_errcode();
#ifdef SINGLE_CHAIN_RESTART
						/* Restart chain */
						applog(LOG_ERR, "chain%d: not working due to multiple resets, restart", cid);
						if (!chain_restart(a1, CHAIN_RESTART_RETRIES, 10)) {
							applog(LOG_ERR, "chain%d: failed to restart for %d times, SHUTDOWN", CHAIN_RESTART_RETRIES);
							cgpu->shutdown = true;
							cgpu->status = LIFE_DEAD;
							cgtime(&thr->sick);
							/* FIXME: should not be infinit loop here */
							while (42)
								sleep(10);
						}
#else
						/* Exit cgminer, allowing systemd watchdog to
						 * restart */
						mcompat_chain_power_down_all();
						exit(1);
#endif
					}
				}
			}
		} 
	}

	/* Temperature control */
	int chain_temp_status = mcompat_tempctrl_update_chain_temp(cid);

	cgpu->temp_min = a1->temp_min = g_chain_tmp[cid].tmp_lo;
	cgpu->temp_max = a1->temp_max = g_chain_tmp[cid].tmp_hi;
	cgpu->temp     = a1->temp     = g_chain_tmp[cid].tmp_avg;

	if (chain_temp_status == TEMP_SHUTDOWN) {
		cgpu->status = LIFE_DEAD;
		cgtime(&thr->sick);
		/* Function doesn't currently return */
		overheated_blinking(cid);
	}

#ifdef USE_AUTONONCE
    mcompat_cmd_auto_nonce(a1->chain_id, 1, REG_LENGTH);   // enable autononce
#endif
    mutex_unlock(&a1->lock);

    if (nonce_ranges_processed < 0){
        applog(LOG_INFO, "nonce_ranges_processed less than 0");
        nonce_ranges_processed = 0;
    }else{
        applog(LOG_DEBUG, "%d, nonces processed %d", cid, nonce_ranges_processed);
    }

    cgtime(&a1->tvScryptCurr);
    timersub(&a1->tvScryptCurr, &a1->tvScryptLast, &a1->tvScryptDiff);
    cgtime(&a1->tvScryptLast);

    /* in case of no progress, prevent busy looping */
    if (!work_updated) // after work updated, also delay 10ms
        cgsleep_ms(5);

	return hashes * 0x100000000ull;
}



static struct api_data *coinflex_api_stats(struct cgpu_info *cgpu)
{
    struct A1_chain *t1 = cgpu->device_data;
	int fan_speed = g_fan_cfg.fan_speed;
    unsigned long long int chipmap = 0;
    struct api_data *root = NULL;
    char s[32];
    int i;

    t1->VidOptimal = true;
    t1->pllOptimal = true;

    ROOT_ADD_API(int, "Chain ID", t1->chain_id, false);
    ROOT_ADD_API(int, "Num chips", t1->num_chips, false);
    ROOT_ADD_API(int, "Num cores", t1->num_cores, false);
    ROOT_ADD_API(int, "Num active chips", t1->num_active_chips, false);
    ROOT_ADD_API(int, "Chain skew", t1->chain_skew, false);
    ROOT_ADD_API(int, "Temp max", t1->temp_max, false);
    ROOT_ADD_API(int, "Temp min", t1->temp_min, false);
	ROOT_ADD_API(int, "Temp", t1->temp, false);

    ROOT_ADD_API(int, "Fan duty", fan_speed, true);
//	ROOT_ADD_API(bool, "FanOptimal", g_fan_ctrl.optimal, false);
	ROOT_ADD_API(int, "iVid", t1->vid, false);
    ROOT_ADD_API(int, "PLL", t1->pll, false);
	ROOT_ADD_API(int, "Voltage Max", t1->volt_max, false);
	ROOT_ADD_API(int, "Voltage Min", t1->volt_min, false);
	ROOT_ADD_API(int, "Voltage Avg", t1->volt, false);
	ROOT_ADD_API(bool, "VidOptimal", t1->VidOptimal, false);
	ROOT_ADD_API(bool, "pllOptimal", t1->pllOptimal, false);
	ROOT_ADD_API(bool, "VoltageBalanced", t1->voltagebalanced, false);
	ROOT_ADD_API(int, "Chain num", cgpu->chainNum, false);
	ROOT_ADD_API(double, "MHS av", cgpu->mhs_av, false);
	ROOT_ADD_API(bool, "Disabled", t1->disabled, false);
	for (i = 0; i < t1->num_chips; i++) {
		if (!t1->chips[i].disabled)
			chipmap |= 1 << i;
	}
	sprintf(s, "%Lx", chipmap);
	ROOT_ADD_API(string, "Enabled chips", s[0], true);

	for (i = 0; i < t1->num_chips; i++) {
		sprintf(s, "%02d HW errors", i);
		ROOT_ADD_API(int, s, t1->chips[i].hw_errors, true);
		sprintf(s, "%02d Stales", i);
		ROOT_ADD_API(int, s, t1->chips[i].stales, true);
		sprintf(s, "%02d Nonces found", i);
		ROOT_ADD_API(int, s, t1->chips[i].nonces_found, true);
		sprintf(s, "%02d Nonce ranges", i);
		ROOT_ADD_API(int, s, t1->chips[i].nonce_ranges_done, true);
		sprintf(s, "%02d Cooldown", i);
		ROOT_ADD_API(int, s, t1->chips[i].cooldown_begin, true);
		sprintf(s, "%02d Fail count", i);
		ROOT_ADD_API(int, s, t1->chips[i].fail_count, true);
		sprintf(s, "%02d Fail reset", i);
		ROOT_ADD_API(int, s, t1->chips[i].fail_reset, true);
		sprintf(s, "%02d Temp", i);
		ROOT_ADD_API(int, s, t1->chips[i].temp, true);
		sprintf(s, "%02d nVol", i);
		ROOT_ADD_API(int, s, t1->chips[i].nVol, true);
		sprintf(s, "%02d PLL", i);
		ROOT_ADD_API(int, s, t1->chips[i].pll, true);
		sprintf(s, "%02d pllOptimal", i);
		ROOT_ADD_API(bool, s, t1->chips[i].pllOptimal, true);
	}
	return root;
}


static struct api_data *coinflex_chip_status(struct cgpu_info *cgpu)
{
	char buffer[12]={0};
    struct A1_chain *a1 = cgpu->device_data;

	//printf("\n\n\nGet chip status\n\n\n");
	int i=0;
	//configure for vsensor

    if(g_temp_status[a1->chain_id] == TEMP_SHUTDOWN)
     return NULL;

	mutex_lock(&a1->lock);

    b29_configure_tvsensor(a1,CMD_ADDR_BROADCAST,0);
	for (i = 0; i < a1->num_active_chips; i++){
    b29_check_voltage(a1, i+1, &s_reg_ctrl);
	}

	//configure for tsensor
    b29_configure_tvsensor(a1,CMD_ADDR_BROADCAST,1);
	//b29_get_voltage_stats(a1, &s_reg_ctrl);

    usleep(20000);

	for (i = 0; i < a1->num_active_chips; i++){
		memset(buffer, 0, sizeof(buffer));
		if (!mcompat_cmd_read_register(a1->chain_id, i+1, buffer, REG_LENGTH)) {
			applog(LOG_NOTICE, "%d: Failed to read register for chip %d -> disabling", a1->chain_id, i+1);
			a1->chips[i].num_cores = 0;
			a1->chips[i].disabled = 1;
			return false;
		}

		//keep ASIC register value
		memcpy(a1->chips[i].reg, buffer, REG_LENGTH);
		a1->chips[i].temp= mcompat_temp_to_centigrade(0x000003ff & ((buffer[7]<<8) | buffer[8]));
	}

	mutex_unlock(&a1->lock);

    //applog(LOG_NOTICE, "T1 api command dbgstats recieved");
	return coinflex_api_stats(cgpu);

}


struct device_drv coinflex_drv = 
{
	.drv_id                 = DRIVER_coinflex,
	.dname                  = "HLT_Coinflex",
	.name                   = "HLT",
	.drv_ver                = COINFLEX_DRIVER_VER,
	.drv_date               = COINFLEX_DRIVER_DATE,
	.drv_detect             = coinflex_detect,
	.get_statline_before    = coinflex_get_statline_before,
	.queue_full             = coinflex_queue_full,
	.get_api_stats          = coinflex_api_stats,
	.get_api_debug          = coinflex_chip_status,
	.identify_device        = NULL,
	.set_device             = NULL,
	.thread_prepare         = NULL,
	.thread_shutdown        = NULL,
	.hw_reset               = NULL,
	.hash_work              = hash_queued_work,
	.update_work            = NULL,
	.flush_work             = coinflex_flush_work,          // new block detected or work restart 
	.scanwork               = coinflex_scanwork,            // scan hash
	.set_chip_reject		= coinflex_set_chip_reject,
		.max_diff				= DIFF_DEF
    };

