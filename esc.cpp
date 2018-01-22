/*
  esc.cpp

  gcc esc.cpp BucketESC.cpp MPU6050.cpp I2Cdev.cpp -o esc -l bcm2835 -l m -g
*/

#include <stdio.h>
#include <signal.h>
#include <time.h>
#include <math.h>
#include <bcm2835.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "BucketESC.h"
#include "BucketControl.h"

#define RAD_TO_DEG 57.2957786
#define RADS_TO_DEGS 131.0

BucketControl control;

MPU6050 mpu;
BucketESC esc1(0x29);
BucketESC esc0(0x2A);
//BucketESC esc0(0x29);
//BucketESC esc1(0x2A);

uint16_t mpuPacketSize;
uint16_t mpuFifoCount;
uint8_t mpuFifoBuffer[64];
uint8_t mpuIntStatus;
Quaternion mpuRawData;
VectorFloat mpuG;
float mpuYPR[3];

double gyroXOffset = -312;
double gyroYOffset = -166;
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct timespec gyroNowTime;
double gyroReadTime;
double gyroCompAngleX, gyroCompAngleY;
double gxRate, gyRate;

struct timespec rpmNowTime;
double rpm0;
double rpm1;
double rpmReadTime;

long thrust = 0;

static volatile bool canLoop = true;

void loop();
void getAndCalculateAngles();
void initGyro();
void autoCalibrateGyro();
int calculatePower(int thrust);
int sgn(double x);
void getRPM();
void onControlInput(const char* data);
void sigIntHandler(int sig);

int main(int argc, char **argv) {
	printf("This is program\n");

	signal(SIGINT, sigIntHandler);

	control.start();

	I2Cdev::initialize();

	bcm2835_delay(100);

	// set control input handler after delay so server is already initialized
	control.setOnDataHandlerFunc(onControlInput);

	if (mpu.testConnection())
		printf("Gyro connected\n");
	else
		fprintf(stderr, "Gyro missing\n");

	/*uint8_t cur = gyro.getDLPFMode();
	printf("Current DLPF mode is %d\n", cur);
	gyro.setDLPFMode(MPU6050_DLPF_BW_5);
	cur = gyro.getDLPFMode();
	printf("New DLPF mode is %d\n", cur);*/

	if (esc0.testConnection())
		printf("ESC 0 connected\n");
	else
		fprintf(stderr, "ESC 0 missing\n");

	if (esc1.testConnection())
		printf("ESC 1 connected\n");
	else
		fprintf(stderr, "ESC 1 missing\n");

	mpu.initialize();
	if (mpu.dmpInitialize() == 0) {
		mpu.setDMPEnabled(true);
		mpuPacketSize = mpu.dmpGetFIFOPacketSize();
	} else {
		printf("DMP could not be initialized\n");
	}
	//autoCalibrateGyro();

	printf("Setting both to 0 and waiting one second...\n");

	esc0.setThrottle(0);
	esc1.setThrottle(0);

	bcm2835_delay(1000);

	//initGyro();

	printf("Looping...\n");
	while (canLoop)
		loop();

	printf("\nkthxbye.\n");

	esc0.setThrottle(0);
	esc1.setThrottle(0);

	return 0;
}

/*void initGyro() {
    gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    double roll = atan2(ay, az) * RAD_TO_DEG;
    double pitch = atan2(-ax, az) * RAD_TO_DEG;

    gyroCompAngleX = roll;
    gyroCompAngleY = pitch;

    clock_gettime(CLOCK_MONOTONIC_RAW, &gyroNowTime);
    double secs = (gyroNowTime.tv_sec) + round(gyroNowTime.tv_nsec / 1.0e9);
    gyroReadTime = secs;
}*/

/*long acgCount = 0;
long acgXSum =0;
long acgYSum = 0;
void autoCalibrateGyro() {
	printf("Calibrating gyro (2 seconds)...\n");

	while (acgCount < 2000) {
    	gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		acgCount++;
		acgXSum += gx;
		acgYSum += gy;

		bcm2835_delay(1);
	}

	gyroXOffset = acgXSum / acgCount;
	gyroYOffset = acgYSum / acgCount;

	printf("Calibrated gyro to X offset %.2f and Y offset %.2f\n", gyroXOffset, gyroYOffset);
}*/

void onControlInput(const char* data) {
	float input = 0.0;
	sscanf(data, "%f", &input);
	thrust = (long)(input * 10000);	// limit whatis?
}

const double kt = 3;
const double kv = 400;

void loop() {
	/*getAndCalculateAngles();

	double angleError = 0.0 - gyroCompAngleY;
	double correction = angleError * kt;

	double velocityDemand = correction;
	double velocityError = velocityDemand - gyRate;

	long thrust0 = realThrust - (velocityError * kv);
	long thrust1 = realThrust + (velocityError * kv);
	long power0 = calculatePower(thrust0);
	long power1 = calculatePower(thrust1);
	//esc0.setThrottle(realThrust);
	//esc1.setThrottle(realThrust);*/

	while (mpuFifoCount < mpuPacketSize) {
		mpuIntStatus = mpu.getIntStatus();
		if ((mpuIntStatus & 0x10) || mpuFifoCount == 1024) {
			mpu.resetFIFO();
			printf("MPU FIFO overflowed!\n");
		}

		mpuFifoCount = mpu.getFIFOCount();
	}

	mpu.getFIFOBytes(mpuFifoBuffer, mpuPacketSize);

	mpuFifoCount -= mpuPacketSize;

	mpu.dmpGetQuaternion(&mpuRawData, mpuFifoBuffer);
	mpu.dmpGetGravity(&mpuG, &mpuRawData);
	mpu.dmpGetYawPitchRoll(mpuYPR, &mpuRawData, &mpuG);

	long realThrust = thrust;
	esc0.setThrottle(0); //thrust);
	esc1.setThrottle(thrust);

	getRPM();
	printf("yaw\t%.2f\tpitch\t%.2f\troll\t%.2f\n",
		mpuYPR[0] * 180/M_PI,
		mpuYPR[1] * 180/M_PI,
		mpuYPR[2] * 180/M_PI
	);

	control.send(CONTROL_OUTPUT_ROLL, mpuYPR[1] * 180/M_PI);
	control.send(CONTROL_OUTPUT_PITCH, mpuYPR[2] * 180/M_PI);
	control.send(CONTROL_OUTPUT_RPM_0, rpm0);
	control.send(CONTROL_OUTPUT_RPM_1, rpm1);

		//printf("%d,%d,%d,%d,%.2f,%.2f\n", gx, gy, ax, ay, gyroCompAngleX, gyroCompAngleY);
		//fflush(stdout);
		//printf("gx %.0f\t angleX %.1f\t gy %.0f\t angleY %.1f\n", gx - gyroXOffset, gyroCompAngleX, gy - gyroYOffset, gyroCompAngleY);
	    //printf("%d\t%0.2f\t%.0f\t%d\t%.0f\t%d\t%d\t%.1f\t%d\n", thrust, velocityError, rpm0, power0, rpm1, power1, gy, gyroCompAngleY, ay);

		/*control.send(CONTROL_OUTPUT_ROLL, gyroCompAngleY);
		control.send(CONTROL_OUTPUT_PITCH, gyroCompAngleX);
		control.send(CONTROL_OUTPUT_THROTTLE_0, power0);
		control.send(CONTROL_OUTPUT_THROTTLE_1, power1);*/
	//} else

	//bcm2835_delay(1);
}

/*void getAndCalculateAngles() {
    gyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    clock_gettime(CLOCK_MONOTONIC_RAW, &gyroNowTime);
    double secs = (gyroNowTime.tv_sec) + (gyroNowTime.tv_nsec / 1.0e9);
    double period = secs - gyroReadTime;

    double roll = atan2(ay, az) * RAD_TO_DEG;
    double pitch = atan2(-ax, az) * RAD_TO_DEG;

    gxRate = (gx - gyroXOffset) / RADS_TO_DEGS;
    gyRate = (gy - gyroYOffset) / RADS_TO_DEGS;

    gyroCompAngleX = (0.999 * (gyroCompAngleX + (gxRate * period))) + (0.001 * roll);
    gyroCompAngleY = (0.999 * (gyroCompAngleY + (gyRate * period))) + (0.001 * pitch);

    gyroReadTime = secs;
}*/

const double lal = sqrt(1000);
int calculatePower(int thrust) {
	int power = (int)(sqrt(abs(thrust)) * lal * sgn(thrust));
	power = power > 15000 ? 15000 : power;
	return power;
}

int sgn(double x) {
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

void getRPM() {
	uint16_t esc0PulseCount = esc0.getPulseCount();
	uint16_t esc1PulseCount = esc1.getPulseCount();

	clock_gettime(CLOCK_MONOTONIC_RAW, &rpmNowTime);
	double secs = (rpmNowTime.tv_sec) + (rpmNowTime.tv_nsec / 1.0e9);
	double period = secs - rpmReadTime;

	rpm0 = (esc0PulseCount / period) * (60 / 8.0);
	rpm1 = (esc1PulseCount / period) * (60 / 8.0);

//	printf("%.4f\t%d\t%.3f\t%d\t%.3f\n", period, esc0PulseCount, rpm0, esc1PulseCount, rpm1);

	rpmReadTime = secs;
}

void sigIntHandler(int sig) {
	canLoop = false;
	control.kill();
}
