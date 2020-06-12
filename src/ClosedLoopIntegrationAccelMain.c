
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
// IMU IS UPSIDE DOWN, ACCEL CALIBRATION CARES ABOUT THIS AND ATTITUDE MIGHT VISUALISE WEIRD
// Defines
#define POLL_RATE_MILLI 20       //50Hz polling = 20 millisecond period
#define GYRO_SCALE 113
#define ACCEL_SCALE 16834
#define CALIB_TIME 2000 //3 seconds to calibrate

//Helper Functions
void readGyroData(float w[3][1], int *offset);
void readAccelData(float *a, int *offset);
void callibrateGyro(int *offsets, int print);
void callibrateAccel(int *offsets, int print);
void extractEulerAngles(float R[3][3], float *Eul);
void RexpFunction(float Rexp[3][3], float dt, float w[3][1]);
float deg2rad(float deg);
float rad2deg(float rad);
float sinc(float x);

int main(void) {
    //    int gyroOffsets[3] = {-173, 241, 87};
    int gyroOffsets[3] = {0, 0, 0};
    int accelOffsets[3] = {-108, 185, -917};
    float w[3][1]; //gyro data
    float a[3]; //accelerometer data 
    float m[3]; //mag data

    float dt = .02; //50hz = 1/50 period
    float kp_a = 10;
    float ki_a = .5;

    float Eul[3] = {0, 0, 0}; //pitch,roll,yaw

    BOARD_Init();
    OledInit();
    MPU9250_Init();
    TIMERS_Init();

//        callibrateGyro(gyroOffsets, 1); //Get gyro biases
//        callibrateAccel(accelOffsets, 1); //Get accel biases

    float N[3][1] = {
        {1},
        {0},
        {0}
    };
    float E[3][1] = {
        {0},
        {-1},
        {0}
    };
    float D[3][1] = {
        {0},
        {0},
        {-1}
    };

    float R[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };

    float biasEstimate[3][1] = {
        {0},
        {0},
        {0}
    };

    unsigned int start, end;
    while (1) {
        start = TIMERS_GetMilliSeconds();
        end = start;

        //Read Gyro Data
        readGyroData(w, gyroOffsets);
        readAccelData(a, accelOffsets);

//        printf("%.4f,%.4f,%.4f",w[0][0],w[1][0],w[2][0]);
//        printf(", %.4f,%.4f,%.4f\n",a[0],a[1],a[2]);
  
        //Subtract Z-axis error using accelerometer
        float Arx[3][3] = {
            {0, -a[2], a[1]},
            {a[2], 0, -a[0]},
            {-a[1], a[0], 0}
        };
        float Rz[3][1];
        float wmeas_a[3][1];
        float bdot[3][1];
        float R_t[3][3];
        MatrixTranspose(R, R_t);
        MatrixMultiply_31(R_t, D, Rz); //Gives Z vector of DCM
        MatrixMultiply_31(Arx, Rz, wmeas_a); //Measure diff between accel vector and dcm Z vector

        // wmeas_a = kp_a*AxD
        MatrixScalarMultiply_31(kp_a, wmeas_a, wmeas_a); //error term
        MatrixScalarMultiply_31(-ki_a, wmeas_a, bdot); // bias estimate


        //Update biasEstimate of gyro bias
        //biasEstimate = biasEstimate + bdot
        MatrixAdd_31(biasEstimate, bdot, biasEstimate);

        //Correct error with  error calculation
        // i.e. w + wmeas_a - biasEstimate
        MatrixAdd_31(w, wmeas_a, w);
        MatrixSubtract_31(w, biasEstimate, w);


        // build Rexp()
        float Rexp[3][3];
        RexpFunction(Rexp, dt, w);

        // R*Rexp(w*dt)
        float Rcopy[3][3];
        MatrixCopy(R, Rcopy);
        MatrixMultiply(Rcopy, Rexp, R);


        //Grab euler angles using DCM parameterse
        extractEulerAngles(R, Eul);
//        MatrixPrint(R, stdout);
        
        //Draw to Oled
        OledClear(OLED_COLOR_BLACK);
        char buffer[100];
        sprintf(buffer, "Yaw: %.0f\nPitch: %.0f\nRoll: %.0f", rad2deg(Eul[2]), rad2deg(Eul[1]), rad2deg(Eul[0]));
        OledDrawString(buffer);
        OledUpdate();


        //TEST STUFF
        if (BTN1) {
            printf("Bias Estimate:\n"
                    "X:%.4f\n"
                    "Y:%.4f\n", rad2deg(biasEstimate[0][0]), rad2deg(biasEstimate[1][0]));
            while (1);
        }
        //Keep Polling Rate to maintain integration
        while (end - start < POLL_RATE_MILLI) {
            end = TIMERS_GetMilliSeconds();
        }
    }




    return 0;
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
    offsets[2] = (sumZ / i + ACCEL_SCALE) * -1;

    if (print) {
        printf("ACCEL BIASES:\n"
                "\tx: %i\n"
                "\ty: %i\n"
                "\tz: %i\n", offsets[0], offsets[1], offsets[2]);
    }

}

void readGyroData(float w[3][1], int *offset) {
    short X, Y, Z;
    X = MPU9250_ReadGyroX();
    Y = MPU9250_ReadGyroY();
    Z = MPU9250_ReadGyroZ();
    w[0][0] = deg2rad((X - offset[0]) / GYRO_SCALE);
    w[1][0] = deg2rad((Y - offset[1]) / GYRO_SCALE);
    w[2][0] = deg2rad((Z - offset[2]) / GYRO_SCALE);
}

void readAccelData(float *a, int *offset) {
    short X, Y, Z;
    X = MPU9250_ReadAccelX();
    Y = MPU9250_ReadAccelY();
    Z = MPU9250_ReadAccelZ();
    a[0] = (float) (X - offset[0]) / ACCEL_SCALE;
    a[1] = (float) (Y - offset[1]) / ACCEL_SCALE;
    a[2] = (float) (Z - offset[2]) / ACCEL_SCALE;
}

void RexpFunction(float Rexp[3][3], float dt, float w[3][1]) {
    float ssW2[3][3];
    float scW[3][3];
    float I[3][3] = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    };
    // Build skew symetric w matrix
    float W[3][3] = {
        {0, -w[2][0], w[1][0]},
        {w[2][0], 0, -w[0][0]},
        {-w[1][0], w[0][0], 0}
    };

    // Get norm(w) , SCALED BY 50
    float wnorm = sqrt(pow(w[2][0], 2) + pow(w[1][0], 2) + pow(w[0][0], 2));

    // sinc(w/2)
    float s = sinc(wnorm / 100); //scaled by dt after wnorm calc

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

}

float deg2rad(float deg) {
    return deg * M_PI / 180;
}

float rad2deg(float rad) {
    return rad / M_PI * 180;
}

void extractEulerAngles(float R[3][3], float *Eul) {
    Eul[0] = (float) atan2(R[1][2], R[2][2]);
    Eul[1] = (float) asinf(-R[0][2]);
    Eul[2] = (float) atan2(R[0][1], R[0][0]);
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

