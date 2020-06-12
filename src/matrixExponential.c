
#include <xc.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "MatrixMath.h"
#include "Oled.h"
#include "MPU9250.h"
#include "timers.h"
#include <BOARD.h>


// SCALE EVERYTHING BY 100
// Defines
#define SCALE 100
#define POLL_RATE_MILLI 20 //50Hz polling = 20 millisecond period
#define GYRO_SCALE 113
#define GYRO_X_OFF -166
#define GYRO_Y_OFF 189
#define GYRO_Z_OFF 18

float w_z = 0;
float w_y = 0;
float w_x = 0;

float dt = .02; //50hz = 1/50 period

//Helper Functions
void readGyroData(void);
float deg2rad(float deg);
float rad2deg(float rad);
float DCMtoTheta(float R_13);
float DCMtoPsi(float R_23, float R_33);
float DCMtoPhi(float R_12, float R_11);
float sinc(float x);

int main(void) {
    BOARD_Init();
    OledInit();
    MPU9250_Init();
    TIMERS_Init();


    float theta;
    float phi;
    float psi;

    float R[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    unsigned int start, end;
    while (1) {
        start = TIMERS_GetMilliSeconds();
        end = start;

        //Read Gyro Data
                readGyroData();

        // build Rexp()
        float ssW2[3][3];
        float scW[3][3];
        float Rexp[3][3];
        float I[3][3] = {
            {1, 0, 0},
            {0, 1, 0},
            {0, 0, 1}
        };


        // Build skew symetric w matrix
        float W[3][3] = {
            {0, -w_z, w_y},
            {w_z, 0, -w_x},
            {-w_y, w_x, 0}
        };

        // Get norm(w) , SCALED BY 50
        float wnorm = sqrt(pow(w_z, 2) + pow(w_y, 2) + pow(w_x, 2));

        // sinc(w/2)
        float s = sinc(wnorm / 100); //

        // cos(w/2)
        float c = cos(wnorm / 100);

        MatrixScalarMultiply(dt, W, W); //W*dt

        // c*s*w
        MatrixScalarMultiply(c * s, W, scW);

        // s*s/2*W*W
        float W_copy[3][3];
        MatrixCopy(W, W_copy);
        MatrixMultiply(W, W_copy, ssW2);
        MatrixScalarMultiply((s * s) / 2, ssW2, ssW2);

        //Rexp= I + csW + ss/2WW
        MatrixAdd(I, scW, I);
        MatrixAdd(I, ssW2, Rexp);

        // R*Rexp(w*dt)
        float Rcopy[3][3];
        MatrixCopy(R, Rcopy);
        MatrixMultiply(Rcopy, Rexp, R);

        //Grab euler angles using DCM parameterse
        theta = rad2deg(DCMtoTheta(R[0][2]));
        phi = rad2deg(DCMtoPhi(R[0][1], R[0][0]));
        psi = rad2deg(DCMtoPsi(R[1][2], R[2][2]));

        MatrixPrint(R, stdout);

        //Draw to Oled
        OledClear(OLED_COLOR_BLACK);
        char buffer[50];
        sprintf(buffer, "Yaw: %.1f\nPitch: %.1f\nRoll: %.1f", psi, theta, phi);
        OledDrawString(buffer);
        OledUpdate();

        //Keep Polling Rate to maintain integration
        while (end - start < POLL_RATE_MILLI) {
            end = TIMERS_GetMilliSeconds();
        }
    }




    return 0;
}

void readGyroData(void) {
    short X, Y, Z;
    X = MPU9250_ReadGyroX();
    Y = MPU9250_ReadGyroY();
    Z = MPU9250_ReadGyroZ();
    w_x = deg2rad((X - GYRO_X_OFF) / GYRO_SCALE);
    w_y = deg2rad((Y - GYRO_Y_OFF) / GYRO_SCALE);
    w_z = deg2rad((Z - GYRO_Z_OFF) / GYRO_SCALE);


}

float deg2rad(float deg) {
    return deg * M_PI / 180;
}

float rad2deg(float rad) {
    return rad / M_PI * 180;
}

float DCMtoTheta(float R_13) {
    return (float) asinf(-R_13);
}

float DCMtoPhi(float R_23, float R_33) {
    return (float) atan2(R_23, R_33);
}

float DCMtoPsi(float R_12, float R_11) {
    return (float) atan2(R_12, R_11);
}

float sinc(float x) {
    if (x < .0001 && x > -.0001) {
        return 1;
    } else {
        return sin(x) / x;
    }
}

float sincTM(float x) {
    if (abs(x) < .01) {
        return 1 - ((x * x) / 6 + ((x * x * x * x) / 120));
    } else {
        return sin(x) / x;
    }
}

