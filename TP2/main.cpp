#include "mbed.h"
#include "Robot.h"
#include "math.h"
#include "ActiveCell.h"
#include "HistogramCell.h"
#define M_PI 3.14159265358979323846
//EXERCICIO 1
//Luis Cruz N2011164454
Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalIn button(PC_13);
void poseEst(float p[], float radius, float enc_res, float b);
void SpeedLim(float w[]);
void initializeArrays();
void calcForce();
void sumForces();
void updateActive(float xR, float yR);
//int ReadSensors();
//const int m = 200, n = 200, activeSize = 11;
//histogram size | aSize active region size
const int hSize = 80, aSize = 11;
ActiveCell activeReg[aSize][aSize];
HistogramCell histogram[hSize][hSize];
//Repulsive force sums
float p[3], p_obj[3], p_final[3], fX, fY;
int main(){

    button.mode(PullUp);
    getCountsAndReset();
    setSpeeds(0, 0);
    initializeArrays();
    while(button==1);
    //w[0] = Omega     | w[1] = Left  | w[2] = Right
    //p[0] = X         | p[1] = Y     | p[2] = Theta
    //p_obj[0] = X     | p_obj[1] = Y | p_obj[2] = Theta
    //b = Distance between wheels, enc_res = Encoder Resolution, v = Calculated speed
    //k_v = Speed gain, k_s = Curvature gain, wratio = Angular speed ratio control command
    //Cells dim: 5x5cm |
    float  w[3], v, theta, theta_error, err, integral = 0.0;
    const float radius = 3.5, b = 13.3, enc_res = 1440, k_v = 7, 
    k_s = 60, k_i = 0.02, sample_time = 0.05, d_stalker = 5.0, k_f = 4;
// ===============================================================================
// =================================== COORDS ====================================
// =============================================================================== 
    //Target coordinates
    p_final[0] = 300, p_final[1] = 300, p_final[2] = 0;
    p_obj[0] = 50, p_obj[1] = 50, p_obj[2] = 0;
    //Initial coordinates:
    p[0] = 0, p[1] = 0, p[2] = 0;
// ===============================================================================
// =================================== EXECUTION =================================
// ===============================================================================
    while(1){
        getCountsAndReset();
        pc.printf("Speeds: Left=%lf   Right=%lf\n", w[1], w[2]);
        pc.printf("OBJECTIVE X: %lf  OBJECTIVE Y: %lf\n", p_obj[0], p_obj[1]);
        pc.printf("Position: X=%lf   Y=%lf   Theta=%lf\n\n", p[0], p[1], p[2]);
        pc.printf("Force (X): X=%lf   Force(Y)=%lf\n\n", fX, fY);
        //Path calculation
        poseEst(p, radius, enc_res, b); //Pose estimation
        updateActive(p[0], p[1]);
        p_obj[0] = p[0]+k_f*fX;
        p_obj[1] = p[1]+k_f*fY;
        //Control Law
        err = sqrt(pow((p_obj[0]-p[0]),2)+pow((p_obj[1]-p[1]),2)) - d_stalker; //distance to the point
        theta = atan2(p_obj[1]-p[1],p_obj[0]-p[0]);
        theta = atan2(sin(theta),cos(theta));
        p[2] = atan2(sin(p[2]),cos(p[2]));
        theta_error = theta-p[2];
        w[0] = k_s*(theta_error); //direction gain
        integral += err;
        v = k_v*err+k_i*integral; //Speed calculation
        w[1] = (v-(b/2)*w[0])/radius;
        w[2] = (v+(b/2)*w[0])/radius;
        SpeedLim(w);
        if((fabs(p[0]-p_final[0])+fabs(p[1]-p_final[1])) < 4){
            setSpeeds(0,0);
        }
        else{
            setSpeeds(w[1], w[2]);
            }
        wait(sample_time); 
    }
}
// ===============================================================================
// =================================== FUNCTIONS =================================
// ===============================================================================
//Pose Estimation function
void poseEst(float p[], float radius, float enc_res, float b){
    float deltaDl, deltaDr, deltaD, deltaT;
    deltaDl = ((float)countsLeft)*(2.0*M_PI*radius/enc_res);
    deltaDr = ((float)countsRight)*(2.0*M_PI*radius/enc_res);
    deltaD = (deltaDr + deltaDl)/2;
    deltaT = (deltaDr - deltaDl)/b;
    if(fabs(deltaT) == 0){
        p[0] = p[0] + deltaD*cos(p[2]) + deltaT/2;
        p[1] = p[1] + deltaD*sin(p[2]) + deltaT/2;
        return;
    }
    p[0] = p[0] + deltaD*(sin(deltaT/2.0f)/(deltaT/2.0f))*cos(p[2]+deltaT/2.0f);
    p[1] = p[1] + deltaD*(sin(deltaT/2.0f)/(deltaT/2.0f))*sin(p[2]+deltaT/2.0f);
    p[2] = p[2] + deltaT;
}
//Speed limiter function
void SpeedLim(float w[]){
    float wratio;
    wratio = fabs(w[2]/w[1]);
    if(w[2] > 150 || w[1] > 150){
        if(wratio < 1){
            w[1] = 150;
            w[2] = w[1]*wratio;
        }
        else if(wratio > 1){
            w[2] = 150;
            w[1] = w[2]/wratio;
        }
        else{
            w[2] = 150;
            w[1] = 150;
        }
    }
    if(w[2] < 50 || w[1] < 50){
        if(wratio < 1){
            w[1] = 50;
            w[2] = w[1]*wratio;
        }
        else if(wratio > 1){
            w[2] = 50;
            w[1] = w[2]/wratio;
        }
        else{
            w[2] = 50;
            w[1] = 50;
        }
    }
}

void initializeArrays() {
    for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            histogram[i][j].calculate(i, j);
            if(i>35 && i<45 && j >35 && j<45)
                histogram[i][j].cellVal=3;
        }
    }
    for (int i = 0; i < aSize; i++) {
        for (int j = 0; j < aSize; j++) {
            activeReg[i][j].calDist(i, j);
        }
    }
}
void calcForce(){
    for (int i = 0; i < aSize; i++) {
        for (int j = 0; j < aSize; j++) {
            activeReg[i][j].calForce();
        }
    }
    activeReg[5][5].forceX=0;
    activeReg[5][5].forceY=0;
}
//every time robot changes position we need to call this function to update active region and calculate forces
//xR, yR - robots position in coordinates system
void updateActive(float xR, float yR) {
    int idXr = 0;
    int idYr = 0;
    for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            if (xR > histogram[i][j].x - 2.5f && xR < histogram[i][j].x + 2.5f && yR > histogram[i][j].y - 2.5f &&
                yR < histogram[i][j].y + 2.5f) {
                idXr = i;
                idYr = j;
                break;
            }
        }
    }
    int m = idXr - aSize / 2;
    for (int k = 0; k < aSize; k++) {
        int n = idYr - aSize / 2;
        for (int l = 0; l < aSize; l++) {
            if (m > 0 && n > 0 && m < hSize && n < hSize) {
                activeReg[k][l].cellVal = histogram[m][n].cellVal;
            }
            n++;
        }
        m++;
    }
    calcForce();
    sumForces();
}
void sumForces(){
    //attractive force
    int Fca=10;
    float rFx=0.0;
    float rFy=0.0;
    float distance = sqrt(pow((float)abs(p_final[0] - p[0]), 2) + pow((float)abs(p_final[1] - p[1]), 2));
    float aFx = Fca*(p_final[0]-p[0])/distance;
    float aFy = Fca*(p_final[1]-p[1])/distance;
    //repulsive force
    for(int i=0;i<aSize;i++){
        for(int j=0;j<aSize;j++){
            rFx+=activeReg[i][j].forceX;
            rFy+=activeReg[i][j].forceY;
        }
    }
    //sum
    fX=aFx-rFx;
    fY=aFy-rFy;
}
