#include "thermal.h"
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>

IRSensor::IRSensor()
{
	this->minTemp = 0;
	this->maxTemp = 0;
	this->fbAddr = nullptr;
	this->i2cDevName = nullptr;
	this->file = 0;
	this->colorScheme = nullptr;
	this->coldDotIndex = 0;
	this->hotDotIndex = 0;
	this->isOk = false;
}

IRSensor::~IRSensor()
{
	if (file != 0)
	{
		close(file);
	}
}

void IRSensor::i2cWrite(const uint8_t addr, const uint8_t value)
{
	uint8_t buf[2] = {addr, value};
	write(file, &buf, 2);
}

uint8_t IRSensor::i2cRead(const uint8_t addr)
{
	uint8_t buf[1] = { 0x00 };
	write(file, &addr, 1);
	read(file, &buf, 1);
	return buf[0];
}


float IRSensor::rawHLtoTemp(const uint8_t rawL, const uint8_t rawH, const float coeff)
{
	const uint16_t therm = ((rawH & 0x07) << 8) | rawL;
	float temp = therm * coeff;
	if ((rawH >> 3) != 0)
	{
		temp *= -1;
	}
	return temp;
}

bool IRSensor::init(const char* i2cDevName, uint8_t* fbAddr, const uint16_t fbResX, const uint16_t fbResY, const uint8_t* colorScheme)
{
	this->i2cDevName = i2cDevName;
	this->setColorScheme(colorScheme);
	this->fbAddr = fbAddr;

	// Open up the I2C bus
	file = open(i2cDevName, O_RDWR);
	if (file == -1)
	{
		return false;
	}
	// Specify the address of the slave device.
	if (ioctl(file, I2C_SLAVE, GRID_EYE_ADDR) < 0)
	{
		return false;
	}

	i2cWrite(0x00, 0x00); //set normal mode
	i2cWrite(0x02, 0x00); //set 10 FPS mode
	i2cWrite(0x03, 0x00); //disable INT

	this->isOk = true;
	return true;
}

void IRSensor::setColorScheme(const uint8_t* colorScheme)
{
	this->colorScheme = colorScheme;
}

float IRSensor::readThermistor()
{
	const uint8_t thermL = i2cRead(0x0E);
	const uint8_t thermH = i2cRead(0x0F);
	return this->rawHLtoTemp(thermL, thermH, THERM_COEFF);
}

void IRSensor::readImage()
{
	uint8_t taddr = 0x80;
	for (uint8_t i = 0; i < 64; i++)
	{
		const uint8_t rawL = i2cRead(taddr); //low
		taddr++;
		const uint8_t rawH = i2cRead(taddr); //high
		taddr++;
		this->dots[i] = this->rawHLtoTemp(rawL, rawH, TEMP_COEFF);
	}
	this->findMinAndMaxTemp();
}

float* IRSensor::getTempMap()
{
	return this->dots;
}

float IRSensor::getMaxTemp()
{
	return this->maxTemp;
}

float IRSensor::getMinTemp()
{
	return this->minTemp;
}

uint8_t IRSensor::getHotDotIndex()
{
	return this->hotDotIndex;
}

uint8_t IRSensor::getColdDotIndex()
{
	return this->coldDotIndex;
}

void IRSensor::drawGradient(const uint8_t startX, const uint8_t startY, const uint8_t stopX, const uint8_t stopY)
{
	uint32_t line[240];
	const uint8_t height = stopY - startY;
	const uint8_t width = stopX - startX;
	const float diff = (maxTemp + minTempCorr - minTemp + maxTempCorr) / height;
	for (uint8_t j = 0; j < height; j++)
	{
		const float temp = minTemp + (diff * j);
		line[j] = temperatureToABGR(temp, minTemp + minTempCorr, maxTemp + maxTempCorr);
	}

	for (uint8_t i = 0; i < width; i++)
	{
		volatile uint32_t *pSdramAddress = (uint32_t *)(this->fbAddr + (240 * (startX + i) + startY) * sizeof(uint32_t));
		for (uint8_t j = 0; j < height; j++)
		{
			*(volatile uint32_t *)pSdramAddress = line[j];	
			pSdramAddress++;
		}
	}
}

void IRSensor::visualizeImage(const uint8_t resX, const uint8_t resY, const uint8_t method)
{
	uint8_t line = 0;
	uint8_t row = 0;

	volatile uint32_t* pSdramAddress = (uint32_t *)this->fbAddr;

	if (method == 0)
	{
		for (uint8_t i = 0; i < 64; i++)
		{
			colors[i] = this->temperatureToABGR(dots[i], minTemp + minTempCorr, maxTemp + maxTempCorr);
		}
		
		const uint8_t lrepeat = resX / 8;
		const uint8_t rrepeat = resY / 8;
		while (line < 8)
		{
			for (uint8_t t = 0; t < lrepeat; t++) //repeat
			{
				while (row < 8)
				{
					for (uint8_t k = 0; k < rrepeat; k++) //repeat
					{
						*(volatile uint32_t *)pSdramAddress = colors[line * 8 + row];
						pSdramAddress++;
					}
					row++;
				}
				row = 0;
			}
			line++;
		}
	}
	else if (method == 1)
	{
		float tmp, u, t, d1, d2, d3, d4;
		uint32_t p1, p2, p3, p4;
		
		for (uint8_t i = 0; i < 64; i++)
		{
			colors[i] = this->temperatureToABGR(dots[i], minTemp + minTempCorr, maxTemp + maxTempCorr);
		}
		
		for (uint16_t j = 0; j < resY; j++) {
			tmp = (float)(j) / (float)(resY - 1) * (8 - 1);
			int16_t h = (int16_t)tmp;
			if (h >= 8 - 1) {
				h = 8 - 2;
			}
			u = tmp - h;

			pSdramAddress = (uint32_t *)(this->fbAddr + (j * resX) * sizeof(uint32_t));

			for (uint16_t i = 0; i < resX; i++) {

				tmp = (float)(i) / (float)(resX - 1) * (8 - 1);
				int16_t w = (int16_t)tmp;
				if (w >= 8 - 1) {
					w = 8 - 2;
				}
				t = tmp - w;

				d1 = (1 - t) * (1 - u);
				d2 = t * (1 - u);
				d3 = t * u;
				d4 = (1 - t) * u;

				p1 = colors[h * 8 + w];
				p2 = colors[h * 8 + w + 1];
				p3 = colors[(h + 1) * 8 + w + 1];
				p4 = colors[(h + 1) * 8 + w];

				//RGB565
				/*uint8_t blue = ((uint8_t)(p1 & 0x001f)*d1 + (uint8_t)(p2 & 0x001f)*d2 + (uint8_t)(p3 & 0x001f)*d3 + (uint8_t)(p4 & 0x001f)*d4);
				uint8_t green = (uint8_t)((p1 >> 5) & 0x003f) * d1 + (uint8_t)((p2 >> 5) & 0x003f) * d2 + (uint8_t)((p3 >> 5) & 0x003f) * d3 + (uint8_t)((p4 >> 5) & 0x003f) * d4;
				uint8_t red = (uint8_t)(p1 >> 11) * d1 + (uint8_t)(p2 >> 11) * d2 + (uint8_t)(p3 >> 11) * d3 + (uint8_t)(p4 >> 11) * d4;*/

				//ABGR8888
				uint8_t blue = ((uint8_t)(p1 >> 16)*d1 + (uint8_t)(p2 >> 16)*d2 + (uint8_t)(p3 >> 16)*d3 + (uint8_t)(p4 >> 16)*d4);
				uint8_t green = (uint8_t)(p1 >> 8) * d1 + (uint8_t)(p2 >> 8) * d2 + (uint8_t)(p3 >> 8) * d3 + (uint8_t)(p4 >> 8) * d4;
				uint8_t red = (uint8_t)(p1 >> 0) * d1 + (uint8_t)(p2 >> 0) * d2 + (uint8_t)(p3 >> 0) * d3 + (uint8_t)(p4 >> 0) * d4;

				*(volatile uint32_t *)pSdramAddress = abgr2color(red, green, blue, 0xff);
				pSdramAddress++;
			}
		}
	}
	else if (method == 2)
	{
		float tmp, u, t, d1, d2, d3, d4;
		float p1, p2, p3, p4;
		
		for (uint16_t j = 0; j < resY; j++) {
			tmp = (float)(j) / (float)(resY - 1) * (8 - 1);
			int16_t h = (int16_t)tmp;
			if (h >= 8 - 1) {
				h = 8 - 2;
			}
			u = tmp - h;

			pSdramAddress = (uint32_t *)(this->fbAddr + (j * resX) * sizeof(uint32_t));

			for (uint16_t i = 0; i < resX; i++) {
				tmp = (float)(i) / (float)(resX - 1) * (8 - 1);
				int16_t w = (int16_t)tmp;
				if (w >= 8 - 1) {
					w = 8 - 2;
				}
				t = tmp - w;

				d1 = (1 - t) * (1 - u);
				d2 = t * (1 - u);
				d3 = t * u;
				d4 = (1 - t) * u;

				p1 = dots[h*8+w];
				p2 = dots[h * 8 + w + 1];
				p3 = dots[(h + 1) * 8 + w + 1];
				p4 = dots[(h + 1) * 8 + w];

				const float temp = p1*d1 + p2*d2 + p3*d3 + p4*d4;

				*(volatile uint32_t *)pSdramAddress = this->temperatureToABGR(temp, minTemp + minTempCorr, maxTemp + maxTempCorr);
				pSdramAddress++;
			}
		}
	}
}

void IRSensor::findMinAndMaxTemp()
{
	this->minTemp = 1000;
	this->maxTemp = -100;
	for (uint8_t i = 0; i < 64; i++)
	{
		if (dots[i] < minTemp)
		{
			minTemp = dots[i];	
			coldDotIndex = i;
		}
		if (dots[i] > maxTemp)
		{
			maxTemp = dots[i];
			hotDotIndex = i;
		}
	}
}

uint16_t IRSensor::rgb2color(const uint8_t R, const uint8_t G, const uint8_t B)
{
	return ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);
}

uint32_t IRSensor::abgr2color(const uint8_t R, const uint8_t G, const uint8_t B, const uint8_t A)
{
	return (A << 24) | (B << 16) | (G << 8) | R;
}

uint8_t IRSensor::calculateRGB(const uint8_t rgb1, const uint8_t rgb2, const float t1, const float step, const float t) {
	return (uint8_t)(rgb1 + (((t - t1) / step) * (rgb2 - rgb1)));
}

uint16_t IRSensor::temperatureToRGB565(const float temperature, const float minTemp, const float maxTemp) {
	uint16_t val;
	if (temperature < minTemp) {
		val = rgb2color(colorScheme[0], colorScheme[1], colorScheme[2]);
	}
	else if (temperature >= maxTemp) {
		const short colorSchemeSize = sizeof(DEFAULT_COLOR_SCHEME);
		val = rgb2color(colorScheme[(colorSchemeSize - 1) * 3 + 0], colorScheme[(colorSchemeSize - 1) * 3 + 1], colorScheme[(colorSchemeSize - 1) * 3 + 2]);
	}
	else {
		const float step = (maxTemp - minTemp) / 10.0;
		const uint8_t step1 = (uint8_t)((temperature - minTemp) / step);
		const uint8_t step2 = step1 + 1;
		const uint8_t red = calculateRGB(colorScheme[step1 * 3 + 0], colorScheme[step2 * 3 + 0], (minTemp + step1 * step), step, temperature);
		const uint8_t green = calculateRGB(colorScheme[step1 * 3 + 1], colorScheme[step2 * 3 + 1], (minTemp + step1 * step), step, temperature);
		const uint8_t blue = calculateRGB(colorScheme[step1 * 3 + 2], colorScheme[step2 * 3 + 2], (minTemp + step1 * step), step, temperature);
		val = rgb2color(red, green, blue);
	}
	return val;
}

uint32_t IRSensor::temperatureToABGR(const float temperature, const float minTemp, const float maxTemp)
{
	uint32_t val;
	if (temperature < minTemp) {
		val = abgr2color(colorScheme[0], colorScheme[1], colorScheme[2], 0xFF);
	}
	else if (temperature >= maxTemp) {
		const short colorSchemeSize = sizeof(DEFAULT_COLOR_SCHEME);
		val = abgr2color(colorScheme[(colorSchemeSize - 1) * 3 + 0], colorScheme[(colorSchemeSize - 1) * 3 + 1], colorScheme[(colorSchemeSize - 1) * 3 + 2], 0xFF);
	}
	else {
		const float step = (maxTemp - minTemp) / 10.0;
		const uint8_t step1 = (uint8_t)((temperature - minTemp) / step);
		const uint8_t step2 = step1 + 1;
		const uint8_t red = calculateRGB(colorScheme[step1 * 3 + 0], colorScheme[step2 * 3 + 0], (minTemp + step1 * step), step, temperature);
		const uint8_t green = calculateRGB(colorScheme[step1 * 3 + 1], colorScheme[step2 * 3 + 1], (minTemp + step1 * step), step, temperature);
		const uint8_t blue = calculateRGB(colorScheme[step1 * 3 + 2], colorScheme[step2 * 3 + 2], (minTemp + step1 * step), step, temperature);
		val = abgr2color(red, green, blue, 0xFF);
	}
	return val;
}
