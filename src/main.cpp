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

volatile uint16_t thermalImg[THERMAL_RES * THERMAL_RES];

extern void fbcon_cursor(int blank);
extern void clearScreen(const uint16_t color, uint8_t* fbp);
extern void copyImgToFb(const uint16_t imgSizeX, const uint16_t imgSizeY, uint8_t* img, uint8_t* fbp);
extern void copyMatToFb(cv::Mat mat, uint8_t* fbp);

extern uint8_t* initFramebuffer();
extern bool initThermal();
extern bool initCamera();

extern cv::Mat readThermal();
extern cv::Mat readCamera();


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

void clearScreen(const uint16_t color, uint8_t* fbp)
{
	for (int y = 0; y<vinfo.yres; y++)
		for (int x = 0; x<vinfo.xres; x++)
		{
			const long location = (x + vinfo.xoffset) * (vinfo.bits_per_pixel / 8) + (y + vinfo.yoffset) * finfo.line_length;
			*((uint16_t*)(fbp + location)) = color;
		}
}

void copyImgToFb(const uint16_t imgSizeX, const uint16_t imgSizeY, uint8_t* img, uint8_t* fbp)
{
	long t = 0;
	for (int y = 0; y < imgSizeY; y++) {
		for (int x = 0; x < imgSizeX; x++) {
			{
				const long location = (x + vinfo.xoffset) * (vinfo.bits_per_pixel / 8) + (y + vinfo.yoffset) * finfo.line_length;
				*((uint16_t*)(fbp + location)) = *((uint16_t*)img + t);
				t++;
			}
		}
	}
}

void copyMatToFb(const cv::Mat mat, uint8_t* fbp)
{
	const cv::Size2d size = mat.size();
	copyImgToFb(size.width, size.height, mat.data, fbp);
}

uint8_t* initFramebuffer()
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
	uint8_t *framebuffer = (uint8_t*)mmap(nullptr, screensize, PROT_READ | PROT_WRITE, MAP_SHARED, fb_fd, (off_t)0);

	clearScreen(0x0000, framebuffer);

	return framebuffer;
}

bool initThermal()
{
	return irSensor.init(THERMAL_I2C, (uint8_t*)&thermalImg, THERMAL_RES, THERMAL_RES, ALTERNATE_COLOR_SCHEME);
}

bool initCamera()
{
	camera.open(0, cv::CAP_V4L);
	if (!camera.isOpened())
	{
		return false;
	}
	camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
	//camera.set(cv::CAP_PROP_FPS, 10);
	return true;
}

cv::Mat readThermal()
{
	cv::Mat thermalFrame;
	irSensor.readImage();
	irSensor.visualizeImage(THERMAL_RES, THERMAL_RES, 2);
	thermalFrame.create(THERMAL_RES, THERMAL_RES, CV_16UC3);
	thermalFrame.data = (uchar*)&thermalImg;
	return thermalFrame;
}

cv::Mat readCamera()
{
	cv::Mat frame, frame2;
	camera.read(frame);
	if (!frame.empty())
	{
		cv::resize(frame, frame2, cv::Size(240, 240), 0, 0, cv::INTER_LINEAR); //resize
		cv::cvtColor(frame2, frame, cv::COLOR_BGR2BGR565); //change color
	}
	return frame;
}

int main(int argc, char *argv[])
{
	
	uint8_t* framebuf = initFramebuffer();
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

	cv::Mat resultFrame;

	int i = 0;

	//char str[10];
	while (true) {
		//const clock_t begin_time = clock();

		const cv::Mat camFrame = readCamera();
		const cv::Mat thermalFrame = readThermal();
		
		//cv::addWeighted(camFrame, 0.5, thermalFrame, 0.5, 0.0, resultFrame);
		
		if (i == 0) {
			copyMatToFb(camFrame, framebuf);
		}
		else {
			copyMatToFb(thermalFrame, framebuf);
		}
		i++;
		if (i > 2)
		{
			i = 0;
		}

		/*const clock_t end_time = clock();
		const float timeMs = float(end_time - begin_time) * 1000 / CLOCKS_PER_SEC;
		sprintf(str, "%f", timeMs);
		std::cout << str << std::endl;*/
	}
}
