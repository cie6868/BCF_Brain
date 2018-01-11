#!/bin/bash
g++ esc.cpp BucketControl.cpp BucketESC.cpp MPU6050.cpp I2Cdev.cpp -o esc -l bcm2835 -g -l m -l pthread -l seasocks

