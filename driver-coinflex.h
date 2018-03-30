
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
//***   driver-coinflex.h is for X11 algorithm mining by using Han-Lab's Pantheon-XXX series miner  ***//
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


#ifndef __DRIVER_COINFLEX_H__
#define __DRIVER_COINFLEX_H__


#include "miner.h"
#include "dragonmint_a11_cmd.h"

#define COINFLEX_DRIVER_NAME                            ("Coinflex")
#define COINFLEX_DRIVER_VER                             ("1.0")
#define COINFLEX_DRIVER_DATE                            ("2017.09.09")
#define COINFLEX_SCANTIME_DEF                           (5)                // refer to static bool coinflex_cal_working_time(struct cgpu_info *coinflex)



#endif /* __DRIVER_COINFLEX_H__ */
