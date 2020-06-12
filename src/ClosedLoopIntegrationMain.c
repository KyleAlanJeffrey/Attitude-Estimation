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
#define MAG_SCALE 317
#define ACCEL_SCALE 16834
#define CALIB_TIME 2000 //3 seconds to calibrate

//Helper Functions
float norm(float x[3][1]);
void DCMfromTRIAD(float R[3][3], float accel[3][1], float mag[3][1], float magInertial[3][1], float accelInertial[3][1]);
void rcross(float w[3][1], float wx[3][3]);
void rotation(float Eul[3], float R[3][3]);
void IntegrateClosedLoop(float R[3][3], float B[3][1], float gyros[3][1], float mags[3][1], float accels[3][1], float magInertial[3][1], float accelInertial[3][1], float deltaT);
void readGyroData(float w[3][1], int *offset);
void readAccelData(float a[3][1], float A_tilde[3][3], float B_tilde[3][1]);
void readMagData(float m[3][1], float A_tilde[3][3], float B_tilde[3][1], float misallign[3][3]);
void extractEulerAngles(float R[3][3], float *Eul);
void RexpFunction(float Rexp[3][3], float dt, float w[3][1]);
float deg2rad(float deg);
float rad2deg(float rad);
float sinc(float x);

int main(void) {
    BOARD_Init();
    OledInit();
    MPU9250_Init();
    TIMERS_Init();

    int gyroOffsets[3] = {0, 0, 0};

    //Determined offsets with least squares tumble 
    float magA_tilde[3][3] = {
        { 1.5451, -0.0023, -0.0024},
        { -0.0057, 1.5461, 0.0461},
        { 0.0014, 0.0152, 1.5147}
    };

    float magB_tilde[3][1] = {
        { -0.8298},
        {-0.8805},
        {-0.2021}
    };

    float magMisallign_t[3][3] = {//Data acquired from matlab script using waba set
        { 0.999934117348199, 0.0101467266085907, -0.00536701986298952},
        {-0.0100636370681569, 0.999832481001198, 0.0152883335897083},
        {0.00552124732643123, -0.0152333146137111, 0.999868722360011}
    };

    float accA_tilde[3][3] = {
        { 0.9596, 0.0082, 0.0406},
        {0.0064, 1.0174, 0.0272},
        {0.0883, 0.0046, 0.9614}
    };
    float accB_tilde[3][1] = {
        {-0.0123},
        {-0.0032},
        {-0.0405}
    };

    float magMisallign[3][3];
    MatrixTranspose(magMisallign_t, magMisallign);

    float w[3][1]; //gyro data
    float a[3][1]; //accelerometer data 
    float m[3][1]; //mag data

    float dt = .02; //50hz = 1/50 period

    float Eul[3] = {0, 0, 0}; //pitch,roll,yaw

    //    float magInertial[3][1] = {//Magnetic Field in Santa Cruz 
    //        { 0.477917769071174},
    //        {0.111849968879240},
    //        {0.871254377588856}
    //    };
    float magInertial[3][1] = {//Magnetic Field in Ventura 
        {0.505856079964144},
        {0.108393129717228},
        {0.855780670378464}
    };
    float accelInertial[3][1] = {
        {0},
        {0},
        {1}
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

        //Read Data
        readGyroData(w, gyroOffsets);
        readAccelData(a, accA_tilde, accB_tilde);
        readMagData(m, magA_tilde, magB_tilde, magMisallign);

        //        printf("%.4f, %.4f, %.4f, ", m[0][0], m[1][0], m[2][0]);
        //        printf("%.4f, %.4f, %.4f\n", a[0][0], a[1][0], a[2][0]);
        
        float R_triad[3][3];
        IntegrateClosedLoop(R, biasEstimate, w, m, a, magInertial, accelInertial, dt);
        DCMfromTRIAD(R_triad, a, m, magInertial, accelInertial);


        //Grab euler angles using DCM parameters
        extractEulerAngles(R, Eul);
        MatrixPrint(R, stdout);
        MatrixPrint(R_triad, stdout);

        //Draw to Oled
        OledClear(OLED_COLOR_BLACK);
        char buffer[100];

        //print Mag data
        //        sprintf(buffer, "Mag: %.1f,%.1f, %.1f\n Acc: %.1f,%.1f, %.1f", m[0][0], m[1][0], m[2][0], a[0][0], a[1][0], a[2][0]);
        sprintf(buffer, "Yaw: % .0f\nPitch: % .0f\nRoll: % .0f", rad2deg(Eul[2]), rad2deg(Eul[1]), rad2deg(Eul[0]));
        OledDrawString(buffer);
        OledUpdate();

        //Stop Program
        if (BTN1) {
            while (1);
        }

        //Keep Polling Rate to maintain dt
        while (end - start < POLL_RATE_MILLI) {
            end = TIMERS_GetMilliSeconds();
        }
    }

    return 0;
}

void IntegrateClosedLoop(float R[3][3], float biasEstimate[3][1], float w[3][1], float m[3][1], float a[3][1], float magInertial[3][1], float accelInertial[3][1], float dt) {

    float kp_a = 8;
    float ki_a = .5;
    float kp_m = 6;
    float ki_m = .5;


    //Subtract Z-axis error using accelerometer
    float Arx[3][3];
    rcross(a, Arx);

    float Mrx[3][3];
    rcross(m, Mrx);

    float Rz[3][1];
    float Rx[3][1];
    float wmeas_a[3][1];
    float wmeas_m[3][1];
    float bdot[3][1];
    float R_t[3][3];
    MatrixTranspose(R, R_t);
    MatrixMultiply_31(R_t, accelInertial, Rz); //Gives Z vector of DCM
    MatrixMultiply_31(R_t, magInertial, Rx); //Gives X vector of DCM
    MatrixMultiply_31(Arx, Rz, wmeas_a); //Measure diff between accel vector and dcm Z vector
    MatrixMultiply_31(Mrx, Rx, wmeas_m); //Measure diff between mag vector and dcm x vector

    // bdot = -ki_a*wmeas_a - ki_m*wmeas_m
    float bias_a[3][1], bias_m[3][1];
    MatrixScalarMultiply_31(-ki_a, wmeas_a, bias_a); // bias estimate
    MatrixScalarMultiply_31(-ki_m, wmeas_m, bias_m); // bias estimate
    MatrixAdd_31(bias_m, bias_a, bdot);

    //Correct error with  error calculation
    // i.e. gyroWithFeedBack(w) = w + kp_a*wmeas_a - biasEstimate
    float error_a[3][1], error_m[3][1];
    MatrixScalarMultiply_31(kp_a, wmeas_a, error_a); //error term accel
    MatrixScalarMultiply_31(kp_m, wmeas_m, error_m); //error term mag
    MatrixAdd_31(w, error_a, w);
    MatrixAdd_31(w, error_m, w);

    MatrixSubtract_31(w, biasEstimate, w);

    // build Rexp()
    float Rexp[3][3];
    RexpFunction(Rexp, dt, w);

    // R*Rexp(w*dt)
    float Rcopy[3][3];
    MatrixCopy(R, Rcopy);
    MatrixMultiply(Rcopy, Rexp, R);


    //Update biasEstimate of gyro bias
    //biasEstimate = biasEstimate + bdot
    MatrixScalarMultiply_31(dt, bdot, bdot);
    MatrixAdd_31(biasEstimate, bdot, biasEstimate);

}

void rcross(float w[3][1], float wx[3][3]) {
    wx[0][0] = 0;
    wx[0][1] = -w[2][0];
    wx[0][2] = w[1][0];
    wx[1][0] = w[2][0];
    wx[1][1] = 0;
    wx[1][2] = -w[0][0];
    wx[2][0] = -w[1][0];
    wx[2][1] = w[0][0];
    wx[2][2] = 0;
}

void readGyroData(float w[3][1], int *offset) {
    short X, Y, Z;
    Y = MPU9250_ReadGyroX();
    X = MPU9250_ReadGyroY();
    Z = MPU9250_ReadGyroZ();
    w[0][0] = deg2rad((float) (X - offset[0]) / GYRO_SCALE);
    w[1][0] = deg2rad((float) (Y - offset[1]) / GYRO_SCALE);
    w[2][0] = deg2rad((float) (Z - offset[2]) / GYRO_SCALE);
    w[2][0] *= -1;
}

void readAccelData(float a[3][1], float A_tilde[3][3], float B_tilde[3][1]) {
    short X, Y, Z;
    Y = MPU9250_ReadAccelX();
    X = MPU9250_ReadAccelY();
    Z = MPU9250_ReadAccelZ();
    a[0][0] = (float) X / ACCEL_SCALE;
    a[1][0] = (float) Y / ACCEL_SCALE;
    a[2][0] = (float) Z / ACCEL_SCALE;

    a[0][0] *= -1;
    a[1][0] *= -1;
    //    MatrixMultiply_3_3_1(A_tilde, a, a);
    //    MatrixAdd_31(a, B_tilde, a);
}

void readMagData(float m[3][1], float A_tilde[3][3], float B_tilde[3][1], float misallign[3][3]) {
    short X, Y, Z;
    X = MPU9250_ReadMagX(); //Magnetometer axis are swapped
    Y = MPU9250_ReadMagY();
    Z = MPU9250_ReadMagZ();
    m[0][0] = (float) X / (float) MAG_SCALE;
    m[1][0] = (float) Y / (float) MAG_SCALE;
    m[2][0] = (float) Z / (float) MAG_SCALE;
    MatrixMultiply_3_3_1(A_tilde, m, m);
    MatrixAdd_31(m, B_tilde, m);
    //    MatrixMultiply_3_3_1(misallign, m, m);
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

void rotation(float Eul[3], float R[3][3]) {
    float angleX = Eul[0];
    float angleY = Eul[1];
    float angleZ = Eul[2];

    R[0][0] = cos(angleY) * cos(angleZ);
    R[0][1] = cos(angleY) * sin(angleZ);
    R[0][2] = -sin(angleY);
    R[1][0] = sin(angleX) * sin(angleY) * cos(angleZ) - cos(angleX) * sin(angleZ);
    R[1][1] = sin(angleX) * sin(angleY) * sin(angleZ) - cos(angleX) * cos(angleZ);
    R[1][2] = sin(angleX) * cos(angleY);
    R[2][0] = cos(angleX) * sin(angleY) * cos(angleZ) + sin(angleX) * sin(angleZ);
    R[2][1] = cos(angleX) * sin(angleY) * sin(angleZ) - sin(angleX) * cos(angleZ);
    R[2][2] = cos(angleX) * cos(angleY);

}

float norm(float x[3][1]) {
    return sqrt(pow(x[0][0], 2) + pow(x[1][0], 2) + pow(x[2][0], 2));
}

void DCMfromTRIAD(float R[3][3], float accel[3][1], float mag[3][1], float magInertial[3][1], float accelInertial[3][1]) {
    float m[3][1], magInertialrx[3][3], mrx[3][3], M[3][1], Ax[3][3], Ay[3][3], magMrx[3][1], Mrxmags[3][1], A[3][3];
    float Ay_t[3][3], magrxM[3][1], mrxm[3][1];

    //    accels = accels / norm(accels);
    MatrixScalarDivide_31(norm(accel), accel, accel);
    //    mags = mags / norm(mags);
    MatrixScalarDivide_31(norm(mag), mag, mag);
    rcross(mag, mrx);

    //    m = rcross(mags) * accels;
    MatrixMultiply_3_3_1(mrx, accel, m);
    //    m = m / norm(m);
    MatrixScalarDivide_31(norm(m), m, m);


    //    M = rcross(magInertial) * accelInertial;
    rcross(magInertial, magInertialrx);
    MatrixMultiply_3_3_1(magInertialrx, accelInertial, M);
    //    M = M / norm(M);
    MatrixScalarDivide_31(norm(M), M, M);




    //    A = [magInertial M rcross(magInertial) * M]*[mags m rcross(mags) * m]';
    //            R = A';

    MatrixMultiply_3_3_1(magInertialrx, M, magrxM);
    MatrixMultiply_3_3_1(mrx, m, mrxm);

    Ax[0][0] = magInertial[0][0];
    Ax[1][0] = magInertial[1][0];
    Ax[2][0] = magInertial[2][0];

    Ax[0][1] = M[0][0];
    Ax[1][1] = M[1][0];
    Ax[2][1] = M[2][0];

    Ax[0][2] = magrxM[0][0];
    Ax[1][2] = magrxM[1][0];
    Ax[2][2] = magrxM[2][0];

    Ay[0][0] = mag[0][0];
    Ay[1][0] = mag[1][0];
    Ay[2][0] = mag[2][0];

    Ay[0][1] = m[0][0];
    Ay[1][1] = m[1][0];
    Ay[2][1] = m[2][0];

    Ay[0][2] = mrxm[0][0];
    Ay[1][2] = mrxm[1][0];
    Ay[2][2] = mrxm[2][0];

    MatrixTranspose(Ay, Ay_t);
    MatrixMultiply(Ax, Ay_t, A);
    MatrixTranspose(A, R);

    //
}