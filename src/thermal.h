#pragma once
#ifndef __THERMAL_H
#define __THERMAL_H

#include "common.h"

#define GRID_EYE_ADDR 0x68
#define THERM_COEFF 0.0625
#define TEMP_COEFF 0.25

class IRSensor {
public:
	IRSensor();
	~IRSensor();
	bool init(const char* i2cDevName, uint8_t* fbAddr, const uint16_t fbResX, const uint16_t fbResY, const uint8_t* colorScheme);
	void setColorScheme(const uint8_t* colorScheme);
	float readThermistor();
	void readImage();
	float* getTempMap();
	float getMaxTemp();
	float getMinTemp();
	uint8_t getHotDotIndex();
	uint8_t getColdDotIndex();
	uint16_t temperatureToRGB565(float temperature, float minTemp, float maxTemp);
	uint32_t temperatureToABGR(float temperature, float minTemp, float maxTemp);
	void visualizeImage(uint8_t resX, uint8_t resY, const uint8_t method);
	void drawGradient(uint8_t startX, uint8_t startY, uint8_t stopX, uint8_t stopY);
protected:
	void findMinAndMaxTemp();
	uint16_t rgb2color(uint8_t R, uint8_t G, uint8_t B);
	uint32_t abgr2color(uint8_t R, uint8_t G, uint8_t B, uint8_t A);
	uint8_t calculateRGB(uint8_t rgb1, uint8_t rgb2, float t1, float step, float t);
private:
	volatile bool isOk;
	volatile uint8_t* fbAddr;
	const char* i2cDevName;
	volatile int file;
	const uint8_t* colorScheme;
	float dots[64];
	uint32_t colors[64];
	uint8_t coldDotIndex;
	uint8_t hotDotIndex;
	float minTemp;
	float maxTemp;
	float rawHLtoTemp(uint8_t rawL, uint8_t rawH, float coeff);
	const float minTempCorr = -0.5;
	const float maxTempCorr = + 0.5;
	void i2cWrite(uint8_t addr, uint8_t value);
	uint8_t i2cRead(uint8_t addr);
};


static const uint8_t DEFAULT_COLOR_SCHEME[] = {
	 28, 1, 108 ,
	 31, 17, 218 ,
	 50, 111, 238 ,
	 63, 196, 229 ,
	 64, 222, 135 ,
	 192, 240, 14 ,
	 223, 172, 18 ,
	 209, 111, 14 ,
	 210, 50, 28 ,
	 194, 26, 0 ,
	 132, 26, 0 
};

static const uint8_t ALTERNATE_COLOR_SCHEME[] = {
	 0, 0, 5 ,
	 7, 1, 97 ,
	 51, 1, 194 ,
	 110, 2, 212 ,
	 158, 6, 150 ,
	 197, 30, 58 ,
	 218, 66, 0 ,
	 237, 137, 0 ,
	 246, 199, 23 ,
	 251, 248, 117 ,
	 252, 254, 253 
};

#endif /* __THERMAL_H */


