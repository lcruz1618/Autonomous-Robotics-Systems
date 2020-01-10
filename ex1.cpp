#include "mbed.h"
#include "Robot.h"
#include "math.h"
#define M_PI 3.14159265358979323846
//EXERCICIO 1
//Luis Cruz N2011164454
Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalIn button(PC_13);
void poseEst(float p[], float radius, float enc_res, float b);
void SpeedLim(float w[]);
int main(){

    button.mode(PullUp);
    getCountsAndReset();
    setSpeeds(0, 0);
    while(button==1);

    //w[0] = Omega     | w[1] = X     | w[2] = Y
    //p[0] = X         | p[1] = Y     | p[2] = Theta
    //p_obj[0] = X     | p_obj[1] = Y | p_obj[2] = Theta
    //b = Distance between wheels, enc_res = Encoder Resolution, v = Calculated speed
    //k_v = Speed gain, k_s = Curvature gain, wratio = Angular speed ratio control command
    float  w[3], v, p[3], p_obj[3], theta, theta_error;
    const float radius = 3.5, b = 13.3, enc_res = 1440, k_v = 7, k_s = 60, sample_time = 0.05;
// ===============================================================================
// =================================== COORDS ====================================
// ===============================================================================
    //Target coordinates
    p_obj[0] = -40, p_obj[1] = 10, p_obj[2] = 0;
    //Initial coordinates:
    p[0] = 0, p[1] = 0, p[2] = 0;
// ===============================================================================
// =================================== EXECUTION =================================
// ===============================================================================
    while(1){
        getCountsAndReset();
        pc.printf("Speeds: Left=%lf   Right=%lf\n", w[1], w[2]);
        pc.printf("Position: X=%lf   Y=%lf   Theta=%lf\n", p[0], p[1], p[2]);

        //Path calculation
        poseEst(p, radius, enc_res, b);
        theta = atan2(p_obj[1]-p[1],p_obj[0]-p[0]);
        theta = atan2(sin(theta),cos(theta));
        p[2] = atan2(sin(p[2]),cos(p[2]));
        theta_error = theta-p[2];
        w[0] = k_s*(theta_error);
        //pc.printf("\nOmega:%lf     a:%lf\n", w[0], theta);
        v = k_v*sqrt(pow((p_obj[0]-p[0]),2)+pow((p_obj[1]-p[1]),2));
        w[1] = (v-(b/2)*w[0])/radius;
        w[2] = (v+(b/2)*w[0])/radius;
        SpeedLim(w);
        if((fabs(p[0]-p_obj[0])+fabs(p[1]-p_obj[1])) < 1){
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
    wratio = w[2]/w[1];
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
