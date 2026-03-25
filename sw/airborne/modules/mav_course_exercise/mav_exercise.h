/*
 * Copyright (C) 2021 Matteo Barbera <matteo.barbera97@gmail.com>
 *
 * This file is part of Paparazzi.
 *
 * Paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 */

#ifndef PAPARAZZI_MAV_EXERCISE_H
#define PAPARAZZI_MAV_EXERCISE_H

extern void mav_exercise_init(void);
extern void mav_exercise_periodic(void);

/* existing obstacle-avoidance tunables */
extern float oa_heading_increment;
extern float divergence_threshold;

/* gate-guidance enable switch */
extern int gg_enabled;

#endif // PAPARAZZI_MAV_EXERCISE_H
