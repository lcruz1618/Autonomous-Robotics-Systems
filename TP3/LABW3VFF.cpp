#include "mbed.h"
#include "BufferedSerial.h"
#include "rplidar.h"
#include "Robot.h"
#include "Communication.h"
#include "math.h"
#include "ActiveCell.h"
#include "HistogramCell.h"
#define M_PI 3.14159265358979323846f
//EXERCICIO 1 c/ lidar
//Luis Cruz N2011164454
RPLidar lidar;
BufferedSerial se_lidar(PA_9, PA_10);
PwmOut rplidar_motor(D3);
struct RPLidarMeasurement data;
Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalIn button(PC_13);
void poseEst(float p[], float radius, float enc_res, float b);
void SpeedLim(float w[]);
void initializeArrays();
void calcForce();
void sumForces();
void updateActive(float xR, float yR);
void mapping();
void prob_calc();
int Bresenham(int x1, int y1, int x2, int y2, int cX[], int cY[]);
void log_cell(int cX[], int cY[], int npts);
//int ReadSensors();
//const int m = 200, n = 200, activeSize = 11;
//histogram size | aSize active region size
const int hSize = 80, aSize = 11;
ActiveCell activeReg[aSize][aSize];
HistogramCell histogram[hSize][hSize];
//Repulsive force sums
float p[3], p_obj[3], p_final[3], fX, fY;
const float Fca=6;/*5*/
int aux99 = 0;
int main(){
    pc.baud(115200);
    button.mode(PullUp);
    getCountsAndReset();
    setSpeeds(0, 0);
    initializeArrays();
    while(button==1);
    // Lidar initialization
    rplidar_motor.period(0.001f);
    lidar.begin(se_lidar);
    lidar.setAngle(0,360);
    rplidar_motor.write(0.8f);
    pc.printf("Program started.\n");
    lidar.startThreadScan();
    //w[0] = Omega     | w[1] = Left  | w[2] = Right
    //p[0] = X         | p[1] = Y     | p[2] = Theta
    //p_obj[0] = X     | p_obj[1] = Y | p_obj[2] = Theta
    //b = Distance between wheels, enc_res = Encoder Resolution, v = Calculated speed
    //k_v = Speed gain, k_s = Curvature gain, wratio = Angular speed ratio control command
    //Cells dim: 5x5cm |
    float  w[3], v, theta, theta_error, err, integral = 0.0, k_i = 0.01/*0.02*/;
    const float radius = 3.5, b = 13.3, enc_res = 1440, k_v = 8/*7*/, 
    k_s = 12/*10*/, sample_time = 0.05, d_stalker = 7.5, k_f = 2; /*2.5 VFF*/
// ===============================================================================
// =================================== COORDS ====================================
// =============================================================================== 
    //Target coordinates
    p_final[0] = 20, p_final[1] = 100, p_final[2] = 0;
    //p_obj[0] = 20, p_obj[1] = 20, p_obj[2] = 0;
    //Initial coordinates:
    p[0] = 20, p[1] = 20, p[2] = M_PI/2;
// ===============================================================================
// =================================== EXECUTION =================================
// ===============================================================================
    while(1){
        getCountsAndReset();
        pc.printf("Speeds: Left=%lf   Right=%lf\n", w[1], w[2]);
        pc.printf("OBJECTIVE X: %lf  OBJECTIVE Y: %lf\n", p_obj[0], p_obj[1]);
        pc.printf("Position: X=%lf   Y=%lf   Theta=%lf\n", p[0], p[1], p[2]);
        pc.printf("Force (X): X=%lf   Force(Y)=%lf\n", fX, fY);
        pc.printf("Theta: %lf   Theta_error%lf\n", theta, theta_error);
        mapping();
        //Path calculation
        poseEst(p, radius, enc_res, b); //Pose estimation
        updateActive(p[0], p[1]);
        //pc.printf("FY=%lf\n",  fY);
        p_obj[0] = p[0]+k_f*fX; // add parameter to relate chosen direction (VFH) to the point nearby of the robot
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
        //pc.printf("w0 = %lf | w1 = %lf | w2 = %lf\n",  w[0], w[1], w[2]);
        SpeedLim(w);
        //if((fabs(p[0]-p_final[0])+fabs(p[1]-p_final[1])) < 70) k_i = -0.005;
        if((fabs(p[0]-p_final[0])+fabs(p[1]-p_final[1])) < 4){
            setSpeeds(0,0);
        }
        else if (aux99 > 5){
            setSpeeds(w[1], w[2]);// Motor's output
            }
        wait(sample_time); 
        aux99++;
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
    if(w[2] < 70 || w[1] < 70){
        if(wratio < 1){
            w[1] = 70;
            w[2] = w[1]*wratio;
        }
        else if(wratio > 1){
            w[2] = 70;
            w[1] = w[2]/wratio;
        }
        else{
            w[2] = 70;
            w[1] = 70;
        }
    }
}

void initializeArrays() {
    for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            //if(((i >= 0 && i <= 10) && (j == 12 || j == 16)) || ((i == 0 || i == 10) && (j >= 12 && j <= 16))) histogram[i][j].cellVal=1;
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
            if (xR > (i*5.0f+2.5f - 2.5f) && xR < (i*5.0f+2.5f + 2.5f) && yR > (j*5.0f+2.5f - 2.5f) &&
                yR < (j*5.0f+2.5f + 2.5f)){
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
            if(m >= 0 && n >= 0 && m < hSize && n < hSize) {
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
    float rFx=0.0;
    float rFy=0.0;
    float distance = sqrt(pow((float)abs(p_final[1] - p[1]), 2) + pow((float)abs(p_final[0] - p[0]), 2));
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
    pc.printf("Repulsive (X): X=%lf  (Y)=%lf\n\n", rFx, rFy);
}
void mapping(){
    float thetaR_deg, readAngle, lDist, lAng;
    int xR, yR, xL, yL, cX[128], cY[128], npts;
    //------------Processing LIDAR readings------------
    lDist = data.distance / 10; //mm -> cm
    if(lDist == 0 || lDist > 400) return; //TO DO (opt): if less than ~10cm! -> make a turn around 
    lAng = data.angle;
    thetaR_deg = (p[2] * 180.0f) / M_PI; //TO DO: Add the robot's angle in world frame to the lidar's reagin angle (readAngle)
    if(thetaR_deg < 0) thetaR_deg = 360 + thetaR_deg;
    readAngle = 270 - lAng + thetaR_deg; //Align LIDAR's frame with robot's one
    if(readAngle > 360) readAngle -= 360; //   "     "      "      "      "
    else if(readAngle < 0) readAngle += 360;// "     "      "      "      "
    pc.printf("ReadAngle: %f | data_deg: %f | Distance: %f | pos: (%f,%f)\n", readAngle, lAng, lDist, p[0], p[1]);
    readAngle = (readAngle * M_PI) /180.0f; // deg -> rads
    xL = lDist * cos(readAngle);
    yL = lDist * sin(readAngle);
    xR = p[0]/5, yR = p[1]/5, xL = xL/5, yL = yL/5; // cm -> cell index units
    int xF = xR + xL; //(xR - xL) -> delta distance THINK IS WRONG should be delta = yL-yR;
    int yF = yR + yL;//(yR - yL) -> delta distance  THINK IS WRONG should be delta = yL-yR;
    if(xF < 0 || yF < 0 || xF > 80 || yF > 80) return;
    pc.printf("xF: %d | yF: %d", xF, yF);
    npts = Bresenham(xR, yR, xF, yF, cX, cY);
    log_cell(cX, cY, npts);
    prob_calc();
}
//Bresenham algo -> trace the Lidar's reading line into a bitmap a.k.a. cell's index
int Bresenham(int x1, int y1, int x2, int y2, int cX[], int cY[]){
    int aux = 0;
    int delta_x(x2 - x1);
    // if xR == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;
    int delta_y(y2 - y1);
    // if yR == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;
    cX[aux] = x1, cY[aux] = y1, aux++;
    pc.printf("x = %d | y = %d | cx = %d | cy = %d | aux = %d \n", x1, y1, cX[aux-1], cY[aux-1], aux-1);
    if (delta_x >= delta_y){
        // error may go below zero
        int error(delta_y - (delta_x >> 1));
        while (x1 != x2)
        {
            // reduce error, while taking into account the corner case of error == 0
            if ((error > 0) || (!error && (ix > 0))){
                error -= delta_x;
                y1 += iy;
            }
            error += delta_y;
            x1 += ix;
            cX[aux] = x1, cY[aux] = y1, aux++;
            pc.printf("x = %d | y = %d | cx = %d | cy = %d | aux = %d \n", x1, y1, cX[aux-1], cY[aux-1], aux-1);
        }
    }
    else{
        // error may go below zero
        int error(delta_x - (delta_y >> 1));
        while (y1 != y2){
            // reduce error, while taking into account the corner case of error == 0
            if ((error > 0) || (!error && (iy > 0))){
                error -= delta_y;
                x1 += ix;
            }
            error += delta_x;
            y1 += iy;
            cX[aux] = x1, cY[aux] = y1, aux++;
            //pc.printf("x = %d | y = %d | cx = %d | cy = %d | aux = %d \n", x1, y1, cX[aux-1], cY[aux-1], aux-1); DEBUG
        }
    }
    return aux;
}
void log_cell(int cX[], int cY[], int npts){
    float l_occ = 0.65;
    float l_free = -0.65;
    for(int i = 0; i < (npts-1); i++){
        histogram[cX[i]][cY[i]].logodds = histogram[cX[i]][cY[i]].logodds + l_free; //l0 já inicializado com o array
    }
    for (int i = -1; i < 2; i++){
        for (int j = -1; j < 2; j++){
            histogram[cX[npts-1]+i][cY[npts-1]+j].logodds = histogram[cX[npts-1]+i][cY[npts-1]+j].logodds + l_occ;
        }
    }
//Força precisa de ser calculada de log odds para probabilidades: <50% de ocupação = livre
//Se o valor da repulsiva óptimo era 3, então vai oscilar entre 0 e 3 - 50 e 100%
}
void prob_calc(){
    float aux;
    for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            aux = (1.0f - (1.0f/(1.0f+exp(histogram[i][j].logodds))));
            if(aux > 0.5f){
                histogram[i][j].cellVal = 2*aux; //6
            }
        }
    }
}
