#include "mbed.h"
#include "Robot.h"
#include "math.h"
#define M_PI 3.14159265358979323846
//EXERCICIO 2
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

//y = -ax/b - c/b
//w[0] = Omega     | w[1] = X     | w[2] = Y
//p[0] = X         | p[1] = Y     | p[2] = theta
//p_obj[0] = X     | p_obj[1] = Y | p_obj[2] = theta
//b = Distance between wheels, enc_res = Encoder Resolution, v = Calculated speed
    
    float  w[3], p[3], theta, theta_error, d, a_, b_ , c_;
    const float radius = 3.5, b = 13.3, enc_res = 1440.0, sample_time = 0.05;
    const float k_d = 0.7, k_h = 8.1, v = 230.0;
// ===============================================================================
// =================================== COORDS ====================================
// ===============================================================================
    //Initial coordinates
    p[0] = -25.0, p[1] = 25.0, p[2] = 0;
    //Line to follow
    a_ = 1.0, b_ = -2.0, c_ = +4.0;
// ===============================================================================
// =================================== EXECUTION =================================
// ===============================================================================

    while(1){
        getCountsAndReset();
        pc.printf("Speeds: Left=%lf   Right=%lf\n", w[1], w[2]);
        pc.printf("Position: X=%lf   Y=%lf   theta=%lf\n\n", p[0], p[1], p[2]);
        //Path calculation
        poseEst(p, radius, enc_res, b);
        d = (a_*p[0] + b_*p[1] + c_)/sqrt(pow(a_,2)+pow(b_,2));
        theta = atan2(-a_,b_);
        theta_error = theta - p[2];
        theta_error = atan2(sin(theta_error),cos(theta_error));
        
        //v = 
        w[0] = -k_d*d+k_h*theta_error;
        w[1] = (v-(b/2)*w[0])/radius;
        w[2] = (v+(b/2)*w[0])/radius;
        SpeedLim(w);
        setSpeeds(w[1], w[2]);
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
    //pc.printf("\n robot.cpp: deltaD:%lf, deltaT:%lf, deltaDl:%lf, deltaDr:%lf\n", deltaD, deltaT, deltaDl, deltaDr);
    if(deltaT == 0){
        p[0] = p[0] + deltaD*cos(p[2]); //+deltaT/2
        p[1] = p[1] + deltaD*sin(p[2]); //+deltaT/2
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
    if(w[2] < 30 || w[1] < 30){
        if(wratio < 1){
            w[1] = 30;
            w[2] = w[1]*wratio;
        }
        else if(wratio > 1){
            w[2] = 30;
            w[1] = w[2]/wratio;
        }
        else{
            w[2] = 30;
            w[1] = 30;
        }
    }
}
