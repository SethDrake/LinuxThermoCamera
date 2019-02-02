#pragma once
#ifndef __FRAMEBUFFER_H
#define __FRAMEBUFFER_H

#include "common.h"

#define COLOR_TRANSP 0x00000000
#define COLOR_WHITE  0xFFFFFFFF
#define COLOR_BLACK  0xFF000000
#define COLOR_BLUE	 0xFFFF0000
#define COLOR_GREEN	 0xFF00FF00
#define COLOR_RED	 0xFF0000FF

typedef enum
{ 
	PORTRAIT = 0,
	LANDSCAPE
} FB_ORIENTATION;

class Framebuffer {
public:
	Framebuffer();
	~Framebuffer();
	void init(const uint32_t fb_addr, const uint16_t fb_sizeX, const uint16_t fb_sizeY, const uint32_t color, const uint32_t bg_color);
	void setFbAddr(const uint32_t fb_addr);
	void setWindowPos(const uint16_t startX, const uint16_t startY);
	void setTextColor(const uint32_t color, const uint32_t bg_color);
	void setOrientation(const FB_ORIENTATION orientation);
	void clear(const uint32_t color);
	uint16_t getFBSizeX();
	uint16_t getFBSizeY();
	void printf(const uint16_t x, const uint16_t y, const uint32_t charColor, const uint32_t bkgColor, const char *format, ...);
	void printf(const uint16_t x, const uint16_t y, const char *format, ...);
	void putString(const char str[], uint16_t x, const uint16_t y, const uint32_t charColor, const uint32_t bkgColor);
	void pixelDraw(const uint16_t xpos, const uint16_t ypos, const uint32_t color);
protected:
	void putChar(const uint16_t x, uint16_t y, const uint8_t chr, const uint32_t charColor, const uint32_t bkgColor);
private:
	uint16_t fb_sizeX;
	uint16_t fb_sizeY;
	uint16_t startX;
	uint16_t startY;
	uint32_t fb_addr;
	uint32_t color;
	uint32_t bg_color;
	volatile uint8_t* font;
	FB_ORIENTATION orientation;
};

#endif /* __FRAMEBUFFER_H */