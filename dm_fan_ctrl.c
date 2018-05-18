/*
 * Copyright 2018 Duan Hao
 * Copyright 2018 Con Kolivas <kernel@kolivas.org>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

/******************************************************************************
 * Description:	fan control using simple PID
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>

#include "asic_b29.h"

#include "dm_temp_ctrl.h"
#include "dm_fan_ctrl.h"

/******************************************************************************
 * Macros & Constants
 ******************************************************************************/
#define FAN_MODE_DEF			FAN_MODE_AUTO		// default fan control mode
#define FAN_SPEED_MAX			(100)				// max fan speed (%)
#define FAN_SPEED_MIN			(0)					// min fan speed (%)
#define FAN_SPEED_DEF			(80)				// default fan speed (%)
#define FAN_SPEED_TARGET_DEF	(50)				// default target fan speed (%)
#define FAN_SPEED_PREHEAT		(10)				// preheat fan speed

#define DEV_TMP_CNT				(16)				// number of temperatures bufferrd
#define DEV_TMP_CNT_MASK		(DEV_TMP_CNT - 1)

#define WORK_CYCLE_DEF			(2)					// default time interval between temperature checks
#define DEV_TMP_CHK_CNT			(3)
#define DEV_TMP_CHK_SPAN		(6)
#define TIMEOUT_GET_TMP			(3)
#define TEMP_TOLERANCE			(2)

#define CYC_INC(cur, inc, mask)	(((cur) + (inc)) & (mask))				// increase index of cycle array
#define CYC_DEC(cur, dec, mask)	(((cur) - (dec) + (mask) + 1) & (mask))	// decrease index of cycle array

/******************************************************************************
 * Global variables
 ******************************************************************************/
volatile c_fan_cfg	g_fan_cfg;				// fan config
volatile int		g_fan_profile;			// fan profile: normal / overheat / preheat

static int			g_dev_tmp_inited = 0;	// if all the elements in cycle array 'g_dev_temp' are initilaized
static int			g_dev_tmp_idx = 0;		// current index of g_dev_temp
static c_temp		g_dev_tmp[DEV_TMP_CNT];	// device temperature sequence

extern int			chain_flag[ASIC_CHAIN_NUM];

/******************************************************************************
 * Prototypes
 ******************************************************************************/
static bool dm_fanctrl_get_tmp(void);
static void dm_fanctrl_set_fan_speed_target(char target);
static void dm_fanctrl_update_fan_speed(void);
static bool dm_fanctrl_check_overheat(void);
static bool dm_fanctrl_check_preheat(void);


/******************************************************************************
 * Implementations
 ******************************************************************************/
void dm_fanctrl_get_defcfg(c_fan_cfg *p_cfg)
{
	p_cfg->fan_mode = FAN_MODE_DEF;
	p_cfg->fan_speed = -1; // do not set any default value //FAN_SPEED_DEF;
	p_cfg->fan_speed_target = FAN_SPEED_TARGET_DEF;
	p_cfg->fan_speed_preheat = FAN_SPEED_PREHEAT;
	p_cfg->fan_ctrl_cycle = WORK_CYCLE_DEF;
	p_cfg->tmp_chk_span = DEV_TMP_CHK_SPAN / p_cfg->fan_ctrl_cycle;
	p_cfg->preheat = true;
}

void dm_fanctrl_init(c_fan_cfg *p_cfg)
{
	// FIXME: add mutex here
	if (NULL == p_cfg)
	{
		c_fan_cfg cfg;
		dm_fanctrl_get_defcfg(&cfg);  // avoid to pass volatile pointer directly
		g_fan_cfg = cfg;
	}
	else
	{
		g_fan_cfg = *p_cfg;
	}

	g_fan_profile = FAN_PF_NORMAL;
}

void *dm_fanctrl_thread(void *argv)
{
	int timeout_get_tmp = 0;

	// set default fan speed
//	dm_fanctrl_set_fan_speed(g_fan_cfg.fan_speed);

	while(true) {
		if (dm_fanctrl_get_tmp()) {
			dm_fanctrl_update_fan_speed();
			timeout_get_tmp = 0;
		} else
			timeout_get_tmp++;

		// force fan speed to 100% when failed to get temperature
		if (timeout_get_tmp >= TIMEOUT_GET_TMP && g_fan_cfg.fan_speed < FAN_SPEED_MAX) {
			applog(LOG_WARNING,
				"WARNING: unable to read temperature, force fan speed to %d", FAN_SPEED_MAX);
			dm_fanctrl_set_fan_speed(FAN_SPEED_MAX);
			timeout_get_tmp = 0;
		}

		sleep(g_fan_cfg.fan_ctrl_cycle);
	}

	return NULL;
}

void dm_fanctrl_set_fan_speed(char speed)
{
	// use 5% step to avoid changing fan speed frequently
	speed = speed / 5 * 5;

	if (speed > FAN_SPEED_MAX)
		speed = FAN_SPEED_MAX;
	else if (speed < g_fan_cfg.fan_speed_preheat)
		speed = g_fan_cfg.fan_speed_preheat;

	if (speed != g_fan_cfg.fan_speed)
	{
		// FIXME: add mutex here
		g_fan_cfg.fan_speed = speed;
		mcompat_fan_speed_set(0, g_fan_cfg.fan_speed);   // fan id is ignored
		applog(LOG_ERR, "fan speed set to %d, TARGET is %d",
			g_fan_cfg.fan_speed, g_fan_cfg.fan_speed_target);
	}
}

static void dm_fanctrl_set_fan_speed_target(char target)
{
	// fan speed target step is 5%
//	target = target / 5 * 5;

	if (target > FAN_SPEED_MAX)
		target = FAN_SPEED_MAX;
	else if (target < g_fan_cfg.fan_speed_preheat)
		target = g_fan_cfg.fan_speed_preheat;

	if (target != g_fan_cfg.fan_speed_target)
	{
		// FIXME: add mutex here
		g_fan_cfg.fan_speed_target = target;
		applog(LOG_NOTICE, "FAN SPEED TARGET set to %d", g_fan_cfg.fan_speed_target);
	}
}

static bool dm_fanctrl_get_tmp(void)
{
	bool retval = false;
	int  i, chain_num = 0;
	c_temp dev_temp;

	// cycle increase by 1
	g_dev_tmp_idx = CYC_INC(g_dev_tmp_idx, 1, DEV_TMP_CNT_MASK);

	// init
	chain_num = 0;
	dev_temp.tmp_hi	 = g_tmp_cfg.tmp_min;
	dev_temp.tmp_lo  = g_tmp_cfg.tmp_max;
	dev_temp.tmp_avg = 0;

	// FIXME: add mutex here
	for (i = 0; i < ASIC_CHAIN_NUM; ++i) {
		if (chain_flag[i]
			&& g_chain_tmp[i].tmp_avg > g_tmp_cfg.tmp_min
			&& g_chain_tmp[i].tmp_avg < g_tmp_cfg.tmp_max) {
			// temperature stat.
			dev_temp.tmp_lo = MIN(dev_temp.tmp_lo, g_chain_tmp[i].tmp_lo);
			dev_temp.tmp_hi = MAX(dev_temp.tmp_hi, g_chain_tmp[i].tmp_hi);
			dev_temp.tmp_avg = MAX(dev_temp.tmp_avg, g_chain_tmp[i].tmp_avg);
			chain_num++;
		}
	}

	if (chain_num > 0) {
//		dev_temp.tmp_avg /= chain_num;
		g_dev_tmp[g_dev_tmp_idx] = dev_temp;

		// fill the temperature sequence
		if (!g_dev_tmp_inited)
		{
			for(i = 0; i < DEV_TMP_CNT; ++i)
			{
				g_dev_tmp[i] = g_dev_tmp[g_dev_tmp_idx];
			}

			g_dev_tmp_inited = 1;
		}

		retval = true;
	} else {
		// copy the previous one
		g_dev_tmp[g_dev_tmp_idx] = 
			g_dev_tmp[CYC_DEC(g_dev_tmp_idx, 1, DEV_TMP_CNT_MASK)];

		retval = false;
	}

	return retval;
}

static bool dm_fanctrl_check_overheat(void)
{
	int tmp_tolerance = 0;

	// if already in overheat mode, apply a small tolerance
	if (FAN_PF_OVERHEAT == g_fan_profile)
		tmp_tolerance = TEMP_TOLERANCE;

	// overheat mode: force to max fan speed while tmp_hi >= tmp_thr_hi
	if (g_dev_tmp[g_dev_tmp_idx].tmp_hi >= g_tmp_cfg.tmp_thr_hi - tmp_tolerance) {
		dm_fanctrl_set_fan_speed(FAN_SPEED_MAX);
		if (FAN_PF_OVERHEAT != g_fan_profile) {
			g_fan_profile = FAN_PF_OVERHEAT;
			applog(LOG_ERR, "OVERHEAT: temp_hi over %d, force fan speed to %d", 
				g_tmp_cfg.tmp_thr_hi, FAN_SPEED_MAX);
		}
		return true;
	}

	g_fan_profile = FAN_PF_NORMAL;

	return false;
}

static bool dm_fanctrl_check_preheat(void)
{
	int tmp_tolerance = 0;

	// preheat mode: do preheating when tmp_avg < tmp_thr_lo
	if (FAN_PF_PREHEAT != g_fan_profile)
		tmp_tolerance = TEMP_TOLERANCE;

	if (g_dev_tmp[g_dev_tmp_idx].tmp_avg < g_tmp_cfg.tmp_thr_lo - tmp_tolerance) {
		dm_fanctrl_set_fan_speed(FAN_SPEED_PREHEAT);
		g_fan_profile = FAN_PF_PREHEAT;
		applog(LOG_ERR, "PREHEAT: tmp_avg under %d, force fan speed to %d", 
			g_tmp_cfg.tmp_thr_lo, FAN_SPEED_PREHEAT);
		return true;
	}

	g_fan_profile = FAN_PF_NORMAL;

	return false;
}

static void dm_fanctrl_update_fan_speed(void)
{
	int i, idx1, idx2;
	int fan_speed;
	int fan_speed_target;
	int fan_speed_target_factor;
	int delta_tmp_avg = g_dev_tmp[g_dev_tmp_idx].tmp_avg - g_tmp_cfg.tmp_target;
	double tmp_rise;
	double k, b;

	// detect overheat first
	if (dm_fanctrl_check_overheat())
		return;
	
	// preheat
	if (g_fan_cfg.preheat && dm_fanctrl_check_preheat())
		return;

	// do nothing util g_dev_tmp has been fully initialized
	if (!g_dev_tmp_inited)
		return;

	// check average temperature rising to determining fan speed target
	tmp_rise = 0;
	for (i = 0; i < DEV_TMP_CHK_CNT; ++i) {
		idx1 = CYC_DEC(g_dev_tmp_idx, i, DEV_TMP_CNT_MASK);
		idx2 = CYC_DEC(idx1, g_fan_cfg.tmp_chk_span, DEV_TMP_CNT_MASK);
		tmp_rise += (g_dev_tmp[idx1].tmp_avg - g_dev_tmp[idx2].tmp_avg);
	}
	tmp_rise /= DEV_TMP_CHK_CNT;
	// if current tmp higher than target tmp and still rising up
	// or current tmp lower than target and still coming down
	// then set speed target higher or lower
	if ((delta_tmp_avg > TEMP_TOLERANCE && tmp_rise > -0.1)
		|| (delta_tmp_avg < -TEMP_TOLERANCE && tmp_rise < 0.1)) {
		// accelerate the speed target changing when temperature is higher than target
		if (g_dev_tmp[g_dev_tmp_idx].tmp_avg < g_tmp_cfg.tmp_target)
			fan_speed_target_factor = 1;
		else
			fan_speed_target_factor = 2;

		fan_speed_target = g_fan_cfg.fan_speed_target + tmp_rise * fan_speed_target_factor;
		dm_fanctrl_set_fan_speed_target(fan_speed_target);
	}

	// calc fan speed factor k and b (slope and intercept)
	if (delta_tmp_avg < 0) {
		// if current temperature lower than target
		k = (float)(g_fan_cfg.fan_speed_target - g_fan_cfg.fan_speed_preheat) 
			/ (g_tmp_cfg.tmp_target - g_tmp_cfg.tmp_thr_lo);
		b = (float)g_fan_cfg.fan_speed_preheat - k * g_tmp_cfg.tmp_thr_lo;
	} else {
		// if current temperature higher than target
		k = (float)(g_fan_cfg.fan_speed_target - FAN_SPEED_MAX) 
			/ (g_tmp_cfg.tmp_target - g_tmp_cfg.tmp_thr_hi);
		b = (float)FAN_SPEED_MAX - k * g_tmp_cfg.tmp_thr_hi;
	}

	// calc fan speed
	fan_speed = k * g_dev_tmp[g_dev_tmp_idx].tmp_avg + b;

	// set fan speed
	dm_fanctrl_set_fan_speed(fan_speed);
}

