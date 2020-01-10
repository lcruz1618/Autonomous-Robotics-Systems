#include "mbed.h"
#include "Robot.h"
#include "math.h"
#define M_PI 3.14159265358979323846
//EXERCICIO 3
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
    //1 pulse/motor rotation = 120 pulse/wheel rotation
    //y = -ax/b - c/b
    //w[0] = Omega     | w[1] = X        | w[2] = Y
    //p[0] = X         | p[1] = Y        | p[2] = Theta (ACTUAL VALUES)
    //p_obj[0] = X     | p_obj[1] = Y    | p_obj[2] = Theta (TARGET VALUES)
    //rab[0] = rho     | rab[1] = alpha  | rab[2] = beta (ACTUAL VALUES)
    ////rab_[0] = rho  | rab_[1] = alpha | rab_[2] = beta (NEW VALUES)
    //b = Distance between wheels, enc_res = Encoder Resolution, v = Calculated speed
    float  w[3], p[3], p_obj[3], rab[3], v;
    const float radius = 3.5, b = 13.3, enc_res = 1440.0, sample_time = 0.05;
    const float k_p = 3.25, k_alpha = 10.0, k_beta = -10.8;
    //(k_alpha - k_p) > 0 , k_p > 0 && k_beta < 0
// ===============================================================================
// =================================== COORDS ====================================
// ===============================================================================
    //Target coordinates
    p_obj[0] = -40, p_obj[1] = 20, p_obj[2] = M_PI;
    p_obj[2] = atan2(sin(p_obj[2]),cos(p_obj[2]));
    //Initial coordinates
    p[0] = 0.0, p[1] = 0.0, p[2] = 0.0;
// ===============================================================================
// =================================== EXECUTION =================================
// ===============================================================================
    while(1){
        getCountsAndReset();
        pc.printf("Speeds: Left=%lf   Right=%lf\n", w[1], w[2]);
        pc.printf("Position: X=%lf   Y=%lf   Theta=%lf\n\n", p[0], p[1], p[2]);
        //Path calculation
        p[2] = atan2(sin(p[2]),cos(p[2]));
        rab[0] = sqrt(pow((p_obj[0]-p[0]),2)+pow((p_obj[1]-p[1]),2));
        rab[1] = -p[2]+atan2((p_obj[1]-p[1]),(p_obj[0]-p[0]));
        rab[2] = -p[2]-rab[1]+p_obj[2];
        rab[1] = atan2(sin(rab[1]),cos(rab[1])); //[-pi,pi]
        rab[2] = atan2(sin(rab[2]),cos(rab[2])); //[-pi,pi]
        
        v = k_p*rab[0];
        w[0]=k_alpha*rab[1]+k_beta*rab[2];
        w[1] = (v-(b/2)*w[0])/radius;
        w[2] = (v+(b/2)*w[0])/radius;
        poseEst(p, radius, enc_res, b);
        SpeedLim(w);
        if((fabs(p[0]-p_obj[0])+fabs(p[1]-p_obj[1])+fabs(p[2]-p_obj[2])) < 1){
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
    //pc.printf("\n robot.cpp: deltaD:%lf, deltaT:%lf, deltaDl:%lf, deltaDr:%lf\n", deltaD, deltaT, deltaDl, deltaDr);
    if(deltaT == 0){
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
    if(w[2] < 80 || w[1] < 80){
        if(wratio < 1){
            w[1] = 80;
            w[2] = w[1]*wratio;
        }
        else if(wratio > 1){
            w[2] = 80;
            w[1] = w[2]/wratio;
        }
        else{
            w[2] = 80;
            w[1] = 80;
        }
    }
}
