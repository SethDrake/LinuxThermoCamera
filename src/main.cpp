#include "main.h"
#include "common.h"
#include "thermal.h"
#include "framebuffer.h"
#include "utils.h"
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
Framebuffer infoPanelFb;

int frameTimeMs = 0;

struct fb_fix_screeninfo finfo;
struct fb_var_screeninfo vinfo;

volatile uint8_t minTemp;
volatile uint8_t maxTemp;
cv::Point hotPos;
cv::Point coldPos;

volatile uint32_t thermalImg[THERMAL_RES * THERMAL_RES];
volatile uint32_t infoPanelImg[INFO_PANEL_WIDTH * INFO_PANEL_HEIGHT];

extern void fbcon_cursor(int blank);
extern void clearScreen(const uint16_t color, uint8_t* fbp);
extern void copyImgToFb(const uint16_t imgSizeX, const uint16_t imgSizeY, uint8_t* img, uint8_t* fbp);
extern void copyMatToFb(cv::Mat mat, uint8_t* fbp);

extern uint8_t* initFramebuffer();
extern bool initThermal();
extern bool initCamera();

extern cv::Mat readThermal();
extern cv::Mat readCamera(int threshold = 0);
extern cv::Mat drawInfoPanel();


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
	return irSensor.init(THERMAL_I2C, THERMAL_RES, THERMAL_RES, ALTERNATE_COLOR_SCHEME);
}

bool initCamera()
{
	camera.open(0, cv::CAP_V4L);
	if (!camera.isOpened())
	{
		return false;
	}
	camera.set(cv::CAP_PROP_FRAME_WIDTH, CAM_FRAME_WIDTH);
	camera.set(cv::CAP_PROP_FRAME_HEIGHT, CAM_FRAME_HEIGHT);
	return true;
}

cv::Mat readThermal()
{
	cv::Mat frame;
	irSensor.readImage();
	irSensor.visualizeImage((uint8_t*)&thermalImg, THERMAL_RES, THERMAL_RES, 2);
	frame.create(THERMAL_RES, THERMAL_RES, CV_8UC4);
	frame.data = (uchar*)&thermalImg;
	cv::cvtColor(frame, frame, cv::COLOR_RGBA2RGB);
	return frame;
}

cv::Mat readCamera(const int threshold)
{
	cv::Mat frame;
	camera.read(frame);
	if (!frame.empty())
	{
		cv::resize(frame, frame, cv::Size(CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT), 0, 0, cv::INTER_LINEAR); //resize
		cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);

		if (threshold > 0)
		{
			cv::Mat edges;
			// ReSharper disable once CppJoinDeclarationAndAssignment
			cv::Mat dst;
			cv::blur(frame, edges, cv::Size(3, 3)); //blur image
			cv::Canny(edges, edges, threshold, threshold * 3, 3); //Canny detector
			// ReSharper disable once CppJoinDeclarationAndAssignment
			dst = cv::Scalar::all(0);
			cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
			frame.copyTo(dst, edges);
			return dst;
		} 

		cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
	}
	return frame;
}

cv::Mat drawInfoPanel()
{
	infoPanelFb.clear(COLOR_BLACK);

	irSensor.drawGradient((uint8_t*)&infoPanelImg, 6, 20, 16, 175, INFO_PANEL_WIDTH);

	const uint8_t hotDot = irSensor.getHotDotIndex();
	const uint8_t hotDotY = hotDot / 8;
	const uint8_t hotDotX = hotDot % 8;
	hotPos = cv::Point(hotDotX * (THERMAL_RES / 8), hotDotY * (THERMAL_RES / 8));
	if (hotPos.x > (THERMAL_RES - 12))
	{
		hotPos.x = THERMAL_RES - 12;
	}
	if (hotPos.y < 12)
	{
		hotPos.y = 12;
	}

	const uint8_t coldDot = irSensor.getColdDotIndex();
	const uint8_t coldDotY = coldDot / 8;
	const uint8_t coldDotX = coldDot % 8;
	coldPos = cv::Point(coldDotX * (THERMAL_RES / 8), coldDotY * (THERMAL_RES / 8));
	if (coldPos.x >(THERMAL_RES - 12))
	{
		coldPos.x = THERMAL_RES - 12;
	}
	if (coldPos.y < 12)
	{
		coldPos.y = 12;
	}

	maxTemp = irSensor.getMaxTemp();
	minTemp = irSensor.getMinTemp();
	const float centerTemp = irSensor.getCenterTemp();

	infoPanelFb.printf(4, 5, COLOR_RED, COLOR_BLACK, "MAX:%u\x81", maxTemp);
	infoPanelFb.printf(4, 180, COLOR_GREEN, COLOR_BLACK, "MIN:%u\x81", minTemp);
	infoPanelFb.printf(4, 206, "VM:%u", 2);
	infoPanelFb.printf(4, 217, "T:%04u", frameTimeMs);
	infoPanelFb.printf(4, 230, "CPU %u%%", (int)GetCPUAverageLoad());

	cv::Mat frame;
	frame.create(INFO_PANEL_HEIGHT, INFO_PANEL_WIDTH , CV_8UC4);
	frame.data = (uchar*)&infoPanelImg;

	char str[6];
	sprintf(str, "%3.2f", centerTemp);
	cv::putText(frame, str, cv::Point(20, 110), cv::FONT_HERSHEY_PLAIN, 1.1, cv::Scalar(0xFF, 0xFF, 0xFF), 2);

	cv::cvtColor(frame, frame, cv::COLOR_RGBA2RGB);
	return frame;
}

void copyMatToFb(const cv::Mat mat, uint8_t* fbp)
{
	cv::Mat resultFrame;
	cv::cvtColor(mat, resultFrame, cv::COLOR_RGB2BGR565);
	const cv::Size2d size = resultFrame.size();
	copyImgToFb(size.width, size.height, resultFrame.data, fbp);
}

int main(int argc, char *argv[])
{
	setlocale(LC_ALL, "");
	
	int cnt = 0;

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

	infoPanelFb.init((uint32_t)&infoPanelImg, INFO_PANEL_WIDTH, INFO_PANEL_HEIGHT, COLOR_WHITE, COLOR_BLACK);
	infoPanelFb.clear(COLOR_BLACK);

	cv::Mat resultFrame;
	resultFrame.create(cv::Size2d(vinfo.xres, vinfo.yres), CV_8UC3);

	cv::Mat infoFrame;
	char str[6];

	while (true) {
		const clock_t begin_time = clock();

		const cv::Mat camFrame = readCamera(0);
		const cv::Mat thermalFrame = readThermal();
		cv::addWeighted(camFrame, 0.6, thermalFrame, 0.4, 0.1, camFrame); //blend images
		//camFrame.convertTo(camFrame, -1, 1.2, 0); //gamma correction

		//center marker
		const cv::Point centerPos = cv::Point(THERMAL_RES / 2, THERMAL_RES / 2);
		
		cv::drawMarker(camFrame, centerPos, cv::Scalar(0xFF, 0xFF, 0xFF), cv::MARKER_CROSS, 20, 1, 8);
		//cv::drawMarker(camFrame, centerPos, cv::Scalar(0x00, 0x00, 0x00), cv::MARKER_SQUARE, 40, 1, 8);
		
		//temperature markers
		sprintf(str, "%u", maxTemp);
		cv::putText(camFrame, str, hotPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0xFF, 0xFF, 0xFF), 1);

		sprintf(str, "%u", minTemp);
		cv::putText(camFrame, str, coldPos, cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0x00, 0xFF, 0x00), 1);

		if (cnt == 0) {
			infoFrame = drawInfoPanel();
		}
		cnt++;
		if (cnt >= 8)
		{
			cnt = 0;
		}

		camFrame.copyTo(resultFrame(cv::Rect(0, 0, camFrame.cols, camFrame.rows))); //copy thermal image to result mat
		infoFrame.copyTo(resultFrame(cv::Rect(CAM_FRAME_WIDTH, 0, infoFrame.cols, infoFrame.rows)));  //copy info panel to result mat

		copyMatToFb(resultFrame, framebuf);

		const clock_t end_time = clock();
		frameTimeMs = float(end_time - begin_time) * 1000 / CLOCKS_PER_SEC;
	}
}
