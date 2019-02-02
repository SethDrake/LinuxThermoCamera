#pragma once
#ifndef __MAIN_H
#define __MAIN_H

#include "framebuffer.h"

#define THERMAL_I2C		   "/dev/i2c-0"
#define THERMAL_RES		   240

#define FRAMEBUFFER_ADDR   "/dev/fb0"

#define CAM_FRAME_WIDTH    240
#define CAM_FRAME_HEIGHT   240

#define INFO_PANEL_WIDTH   80
#define INFO_PANEL_HEIGHT  240

extern Framebuffer infoPanelFb;

#endif /* __MAIN_H */
