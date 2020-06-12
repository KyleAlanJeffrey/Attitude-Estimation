#include <xc.h>
#include "BOARD.h"
#include "I2C.h"
#include "stdio.h"
#include "MPU9250.h"
#include "ToneGeneration.h"
#include "timers.h"
#include "MatrixMath.h"
//Defines
#define CALIB_TIME 1000

#define ACCEL_SCALE 16834
#define MAG_SCALE 317
#define delay(x) ({int i;for(i=0;i<x;i++);})

void readAccelData(float a[3][1],  float A_tilde[3][3], float B_tilde[3][1]);
void readMagData(float m[3][1], float A_tilde[3][3], float B_tilde[3][1], float misallign[3][3]);
void readData(char mode, float dat[3][1]);
void timeDelay(int millisecondDelay); //Quarter Second Delay
void callibrateGyro(int *offsets, int print);
void callibrateAccel(int *offsets, int print);

int main(void) {
    BOARD_Init();
    MPU9250_Init();
    TIMERS_Init();
    //Determined offsets with least squares tumble 
    float magA_tilde[3][3] = {
        { 1.3571, 0.0447, -0.0205},
        {-0.0275, 1.4244, -0.0002},
        {0.0015, 0.0057, 1.2987}
    };

    float magB_tilde[3][1] = {
        { -0.1421},
        { -1.3081},
        { 0.0573}
    };
    float accA_tilde[3][3] = {
        {0.9972, 0.0064, -0.0058},
        {-0.0112, 0.9684, -0.0277},
        {0.0142, -0.0401, 0.9657}
    };
    float accB_tilde[3][1] = {
        { 0.0271},
        {0.0045},
        {-0.0562}
    };

    float magMisallign[3][3] = {
        {0.5673, 0.7728, 0.2845},
        {-0.1583, 0.4414, -0.8832},
        {-0.8081, 0.4561, 0.3727}
    };

    float a[3][1], m[3][1];
    float misallign[3][3];
    while (1) {
        unsigned int start = TIMERS_GetMilliSeconds();
        unsigned int end = start;
        readMagData(m, magA_tilde, magB_tilde, misallign);
        printf("%.4f, %.4f, %.4f, ", m[0][0], m[1][0], m[2][0]);
        readAccelData(a, accelOffsets);
        printf("%.4f, %.4f, %.4f\n", a[0][0], a[1][0], a[2][0]);
        while (end - start < 20) {
            end = TIMERS_GetMilliSeconds();
        }
    }

    return 0;
}

void readAccelData(float a[3][1], int *offset) {
    short X, Y, Z;
    X = MPU9250_ReadAccelX();
    Y = MPU9250_ReadAccelY();
    Z = MPU9250_ReadAccelZ();
    a[0][0] = (float) X / ACCEL_SCALE;
    a[1][0] = (float) Y / ACCEL_SCALE;
    a[2][0] = (float) Z / ACCEL_SCALE;
}

void readData(char mode, float dat[3][1]) {
    short X, Y, Z;
    if (mode == 'g') {
        X = MPU9250_ReadGyroX();
        Y = MPU9250_ReadGyroY();
        Z = MPU9250_ReadGyroZ();
    } else
        if (mode == 'm') {
        X = MPU9250_ReadMagX();
        Y = MPU9250_ReadMagY();
        Z = MPU9250_ReadMagZ();
    } else
        if (mode == 'a') {
        X = MPU9250_ReadAccelX();
        Y = MPU9250_ReadAccelY();
        Z = MPU9250_ReadAccelZ();
    }
    dat[0][0] = X;
    dat[1][0] = Y;
    dat[2][0] = Z;

    //    printf("%i, %i, %i", X, Y, Z);
}

void timeDelay(int millisecondDelay) { //Quarter Second Delay

    int time0 = TIMERS_GetMilliSeconds();
    int time1 = time0;
    while (time1 - time0 < millisecondDelay) {
        time1 = TIMERS_GetMilliSeconds();
    }
}

void callibrateGyro(int *offsets, int print) {
    //Get Biases
    unsigned int start = TIMERS_GetMilliSeconds();
    unsigned int end = start;
    int i = 0, sumX = 0, sumY = 0, sumZ = 0;
    while (end - start < CALIB_TIME) {
        sumX += MPU9250_ReadGyroX();
        sumY += MPU9250_ReadGyroY();
        sumZ += MPU9250_ReadGyroZ();
        i++;
        end = TIMERS_GetMilliSeconds();
    }
    offsets[0] = sumX / i;
    offsets[1] = sumY / i;
    offsets[2] = sumZ / i;

    if (print) {
        printf("GYRO BIASES:\n"
                "\tx: %i\n"
                "\ty: %i\n"
                "\tz: %i\n", offsets[0], offsets[1], offsets[2]);
    }

}

void readMagData(float m[3][1], float A_tilde[3][3], float B_tilde[3][1], float misallign[3][3]) {
    short X, Y, Z;
    X = MPU9250_ReadMagX();
    Y = MPU9250_ReadMagY();
    Z = MPU9250_ReadMagZ();
    m[0][0] = X / (float) MAG_SCALE;
    m[1][0] = Y / (float) MAG_SCALE;
    m[2][0] = Z / (float) MAG_SCALE;
    //    MatrixMultiply_3_3_1(A_tilde, m, m);
    //    MatrixAdd_31(m, B_tilde, m);
}

void callibrateAccel(int *offsets, int print) {
    //Get Biases
    unsigned int start = TIMERS_GetMilliSeconds();
    unsigned int end = start;
    int i = 0, sumX = 0, sumY = 0, sumZ = 0;
    while (end - start < CALIB_TIME) {
        sumX += MPU9250_ReadAccelX();
        sumY += MPU9250_ReadAccelY();
        sumZ += MPU9250_ReadAccelZ();
        i++;
        end = TIMERS_GetMilliSeconds();
    }
    offsets[0] = sumX / i;
    offsets[1] = sumY / i;
    offsets[2] = (sumZ / i);

    if (print) {
        printf("ACCEL BIASES:\n"
                "\tx: %i\n"
                "\ty: %i\n"
                "\tz: %i\n", offsets[0], offsets[1], offsets[2]);
    }

}
