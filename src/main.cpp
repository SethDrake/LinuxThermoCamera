#include "main.h"
#include "common.h"
#include "thermal.h"
#include <iostream>
#include <linux/fb.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <time.h>

IRSensor irSensor;
cv::VideoCapture camera;

struct fb_fix_screeninfo finfo;
struct fb_var_screeninfo vinfo;

uint8_t *framebuffer;

volatile uint16_t thermalImg[THERMAL_RES * THERMAL_RES];

extern void fbcon_cursor(int blank);
extern void clearScreen(const uint16_t color, uint8_t* fbp, struct fb_var_screeninfo* vinfo, struct fb_fix_screeninfo* finfo);
extern void copyImgToFb(const uint16_t imgSizeX, const uint16_t imgSizeY, uint8_t* img, uint8_t* fbp, struct fb_var_screeninfo* vinfo, struct fb_fix_screeninfo* finfo);

extern void initFramebuffer();
extern bool initThermal();
extern bool initCamera();
extern void readThermal();
extern void readCamera();


void fbcon_cursor(const int blank)
{
	const int fd = open("/dev/tty1", O_RDWR);
	if (0 < fd)
	{
		write(fd, "\033[?25", 5);
		write(fd, blank ? "h" : "l", 1);
	}
	close(fd);
}

void clearScreen(const uint16_t color, uint8_t* fbp, struct fb_var_screeninfo* vinfo, struct fb_fix_screeninfo* finfo)
{
	for (int y = 0; y<vinfo->yres; y++)
		for (int x = 0; x<vinfo->xres; x++)
		{
			const long location = (x + vinfo->xoffset) * (vinfo->bits_per_pixel / 8) + (y + vinfo->yoffset) * finfo->line_length;
			*((uint16_t*)(fbp + location)) = color;
		}
}

void copyImgToFb(const uint16_t imgSizeX, const uint16_t imgSizeY, uint8_t* img, uint8_t* fbp, struct fb_var_screeninfo* vinfo, struct fb_fix_screeninfo* finfo)
{
	long t = 0;
	for (int y = 0; y < imgSizeY; y++) {
		for (int x = 0; x < imgSizeX; x++) {
			{
				const long location = (x + vinfo->xoffset) * (vinfo->bits_per_pixel / 8) + (y + vinfo->yoffset) * finfo->line_length;
				*((uint16_t*)(fbp + location)) = *((uint16_t*)img + t);
				t++;
			}
		}
	}
}

void initFramebuffer()
{
	fbcon_cursor(0);

	const int fb_fd = open(FRAMEBUFFER_ADDR, O_RDWR);

	ioctl(fb_fd, FBIOBLANK, VESA_NO_BLANKING);

	//Get variable screen information
	ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo);
	vinfo.grayscale = 0;
	vinfo.bits_per_pixel = 16;
	ioctl(fb_fd, FBIOPUT_VSCREENINFO, &vinfo);
	ioctl(fb_fd, FBIOGET_VSCREENINFO, &vinfo);

	ioctl(fb_fd, FBIOGET_FSCREENINFO, &finfo);

	const long screensize = vinfo.yres_virtual * finfo.line_length;
	framebuffer = (uint8_t*)mmap(nullptr, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, (off_t)0);

	clearScreen(0x0000, framebuffer, &vinfo, &finfo);
}

bool initThermal()
{
	return irSensor.init(THERMAL_I2C, (uint8_t*)&thermalImg, THERMAL_RES, THERMAL_RES, ALTERNATE_COLOR_SCHEME);
}

bool initCamera()
{
	camera.open(0);
	if (!camera.isOpened())
	{
		return false;
	}
	camera.set(cv::CAP_PROP_FRAME_WIDTH, 240);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
	//camera.set(cv::CAP_PROP_FPS, 10);
	return true;
}

void readThermal()
{
	irSensor.readImage();
	irSensor.visualizeImage(THERMAL_RES, THERMAL_RES, 2);
	cv::Mat thermalFrame;
}

void readCamera()
{
	cv::Mat frame;
	cv::Mat convertedFrame;
	camera.read(frame);
	if (!frame.empty())
	{
		cv::Size2f frame_size = frame.size();
		cv::cvtColor(frame, convertedFrame, cv::COLOR_BGR2BGR565);
		copyImgToFb(frame_size.width, frame_size.height, convertedFrame.data, framebuffer, &vinfo, &finfo);
	}
}

int main(int argc, char *argv[])
{
	initFramebuffer();
	if (!initThermal())
	{
		perror("Unable to init AMG8833 sensor!");
		return 1;
	}
	if (!initCamera())
	{
		perror("Unable to init camera sensor!");
		return 2;
	}

	//char str[10];
	while (true) {
		readCamera();
		//const clock_t begin_time = clock();
		//readThermal();
		//copyImgToFb(THERMAL_RES, THERMAL_RES, (uint8_t*)&thermalImg, framebuffer, &vinfo, &finfo);
		/*const clock_t end_time = clock();
		const float timeMs = float(end_time - begin_time) * 1000 / CLOCKS_PER_SEC;
		sprintf(str, "%f", timeMs);
		std::cout << str << std::endl;*/
	}
}
