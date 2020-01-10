#include "mbed.h"
#include "BufferedSerial.h"
#include "rplidar.h"
#include "Robot.h"
#include "Communication.h"
#include "math.h"
#include "ActiveCell.h"
#include "HistogramCell.h"
#define M_PI 3.14159265358979323846f
//Luis Cruz N2011164454
//VFH/VFF (LIDAR)

RPLidar lidar;
BufferedSerial se_lidar(PA_9, PA_10);
PwmOut rplidar_motor(D3);
struct RPLidarMeasurement data;

Serial pc(SERIAL_TX, SERIAL_RX, 115200);
DigitalIn button(PC_13);
void poseEst(float p[], float radius, float enc_res, float b);
void SpeedLim(float w[]);
void initializeArrays();
void calcSectors(float theta);
void sumForces();
void updateActive(float xR, float yR,float theta);
void mapping();
void prob_calc();
int Bresenham(int x1, int y1, int x2, int y2, int cX[], int cY[]);
void log_cell(int cX[], int cY[], int npts);
//Histogram size -> hSize | Active region size -> aSize
const int hSize = 80, aSize = 11;
ActiveCell activeReg[aSize][aSize];
HistogramCell histogram[hSize][hSize];
//Repulsive force sums
float p[3], p_obj[3], p_final[3], fX, fY;
int aux99 = 0;
//const float Fca=6;/*5*/

//VFH
const int L=2;
float secVal[36];
float smooth[36];

int main(){
    pc.baud(115200);
    init_communication(&pc);
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    button.mode(PullUp);
    getCountsAndReset();
    setSpeeds(0, 0);
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
    k_s = 12/*10*/, sample_time = 0.05, d_stalker = 5, k_f = 12.5; /*12.5*/ //VFF
    float theta_final;
// ===============================================================================
// =================================== COORDS ====================================
// =============================================================================== 
    //Target coordinates
    p_final[0] = 250, p_final[1] = 100, p_final[2] = 0;
    //p_obj[0] = 20, p_obj[1] = 20, p_obj[2] = 0;
    //Initial coordinates:
    p[0] = 100, p[1] = 100, p[2] = 0;
// ===============================================================================
// =================================== EXECUTION =================================
// ===============================================================================
    initializeArrays();
    while(1){
        // poll for measurements. Returns -1 if no new measurements are available. returns 0 if found one.
        if(lidar.pollSensorData(&data) == 0) //pc.printf("dist:%f  angle:%f\n", data.distance, data.angle); // Prints one lidar measurement.
        getCountsAndReset();
        //pc.printf("Speeds: Left=%lf   Right=%lf\n", w[1], w[2]);  //    DEBUG
        //pc.printf("OBJECTIVE X: %lf  OBJECTIVE Y: %lf\n", p_obj[0], p_obj[1]);        DEBUG
        pc.printf("Position: X=%lf   Y=%lf   Theta=%lf\n\n", p[0], p[1], p[2]);//       DEBUG
        //pc.printf("Force (X): X=%lf   Force(Y)=%lf\n", fX, fY);       DEBUG
        //Path calculation
        mapping();
        poseEst(p, radius, enc_res, b); 
        theta_final = atan2(p_final[1]-p[1],p_final[0]-p[0]);
        theta_final = atan2(sin(theta_final),cos(theta_final));
        updateActive(p[0], p[1], theta_final);
        p_obj[0] = p[0]+k_f*fX; //add parameter to relate chosen direction (VFH) to the point nearby of the robot
        p_obj[1] = p[1]+k_f*fY;
        //Control Law
        err = sqrt(pow((p_obj[0]-p[0]),2)+pow((p_obj[1]-p[1]),2)) - d_stalker; //distance to the "carrot" point
        theta = atan2(p_obj[1]-p[1],p_obj[0]-p[0]);
        //pc.printf("theta MAIN: = %lf\n\n", theta); DEBUG
        theta = atan2(sin(theta),cos(theta));
        p[2] = atan2(sin(p[2]),cos(p[2]));
        theta_error = theta-p[2];
        theta_error = atan2(sin(theta_error),cos(theta_error));
        //pc.printf("theta_error = %lf | p[2]= %lf\n\n", theta_error, p[2]); DEBUG
        w[0] = k_s*(theta_error); //direction gain
        integral += err;
        v = k_v*err+k_i*integral; //Speed calculation
        w[1] = (v-(b/2)*w[0])/radius;
        w[2] = (v+(b/2)*w[0])/radius;
        SpeedLim(w);
        //if((fabs(p[0]-p_final[0])+fabs(p[1]-p_final[1])) < 70) k_i = -0.005; //Not functional, meant to decrease speed as aproaching final point
        if((fabs(p[0]-p_final[0])+fabs(p[1]-p_final[1])) < 5){
            setSpeeds(0,0);
        }
        else if (aux99 > 99){
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
    deltaDl = ((float)countsLeft)*(2.0f*M_PI*radius/enc_res);
    deltaDr = ((float)countsRight)*(2.0f*M_PI*radius/enc_res);
    deltaD = (deltaDr + deltaDl)/2.0f;
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
    /*for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            histogram[i][j].cellVal = 0;
            //if(((i >= 8 && i <= 12) && (j == 0 || j == 8)) || ((i == 8 || i == 12) && (j >= 0 && j <= 8))) histogram[i][j].cellVal=3;
            //if(((i >= 0 && i <= 3) && (j == 8 || j == 12)) || ((i == 0 || i == 3) && (j >= 8 && j <= 12))) histogram[i][j].cellVal=3;
            //if(((i >= 14 && i <= 20) && (j == 8 || j == 9)) || ((i == 14 || i == 20) && (j >= 8 && j <= 9))) histogram[i][j].cellVal=3;
        }
    }*/
    for (int i = 0; i < aSize; i++) {
        for (int j = 0; j < aSize; j++) {
            activeReg[i][j].calDist(i, j);
        }
    }
}
//xR, yR - robots position in coordinates system - ATM updating histogram (10/5)
void updateActive(float xR, float yR,float theta) {
    int idXr = 0;
    int idYr = 0;
    for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            if (xR >= (i*5.0f) && xR < (i*5.0f+5.0f) && yR >= (j*5.0f) &&
                yR < (j*5.0f+5.0f)){
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
    
    for (int i = 0; i < aSize; i++) {
        for (int j = 0; j < aSize; j++) {
            activeReg[i][j].calForce();
        }
    }
    activeReg[5][5].amplitude=0;
    activeReg[5][5].amplitude=0;
    /*for (int j = 10; j >= 0; j--) {
        for (int i = 0; i < 11; i++) {
            cout << "[" << activeReg[i][j].cellVal << "]";
        }
        cout << endl;
    }*/
    calcSectors(theta);
}
void calcSectors(float theta){
    for (int k = 0; k < 36; ++k) {
        secVal[k]=0;
        for (int i = 0; i < aSize; ++i) {
            for (int j = 0; j < aSize; ++j) {
                if(activeReg[i][j].sectorK==k)
                    secVal[k]+=activeReg[i][j].amplitude;
            }
        }
    }

    smooth[0]=(secVal[34]+2*secVal[35]+2*secVal[0]+2*secVal[1]+secVal[2])/5;
    smooth[1]=(secVal[35]+2*secVal[0]+2*secVal[1]+2*secVal[2]+secVal[3])/5;
    smooth[34]=(secVal[32]+2*secVal[33]+2*secVal[34]+2*secVal[35]+secVal[0])/5;
    smooth[35]=(secVal[33]+2*secVal[34]+2*secVal[35]+2*secVal[0]+secVal[1])/5;
    for (int i = 2; i < 34; ++i) {
        smooth[i]=(secVal[i-L]+2*secVal[i-L+1]+2*secVal[i]+2*secVal[i+L-1]+secVal[i+L])/5;
    }
    
    const int thresh=200;//100
    int temp[36];
    int counter = 0, aux = 0;
    int valley[36];
    int sMax=8;
    for(int i=0;i<36;++i){
        //pc.printf("|%lf", smooth[i]);
        if(smooth[i]<thresh){
            temp[i]=1;
            //valley[aux][aux] = 
            counter++;
        }
        else{
            valley[aux] = counter;
            counter = 0;
            aux++;
            temp[i]=0;
            //pc.printf("#%d", i);
        }
        
    }
    //float best=999;
    float theta_deg;
    theta_deg =(theta*180.0f)/M_PI;
    //pc.printf("theta (degrees): = %lf\n\n", theta_deg);
   int destSec = theta_deg / 10;
   if(destSec<0) destSec=36+destSec;
   //cout<<"destination sector: "<<destSec<<endl;
   
   int L=destSec;
    int R=destSec;
    while(temp[L]==0){
        L--;
        if(L<0) L=35;
    }
    while(temp[R]==0){
        R++;
        if(R>35) R=0;
    }
   
    float dirSet, dirC,dirL,dirR;
    if(temp[destSec]==1){
        int k=destSec-1;
        if(k<0) k=35;
        int size=1;
        while(temp[k]==1){
            size++;
            k--;
            if(k<0) k=35;
            if(k==destSec) break;
            if(size>=sMax) break;
        }
        int right=k+1;
        if(right<0) right=35;
        k=destSec+1;
        if(k>35) k=0;
        while(temp[k]==1){
            size++;
            k++;
            if(k>35) k=0;
            if(k==destSec) break;
            if(size>=sMax) break;
        }
        int left=k-1;
        if(left>35) left=0;
        if(size>=sMax) {
        //wide
            dirC=destSec*10;
            //cout << "wide"<<endl;
            }
    
        else if(size>4 && size<sMax) //narrow
        {
            dirC=0.5*(left*10+right*10);
            //cout<<"narrow"<<endl;
        } else {
            int secL = L;
        while (temp[secL] != 1) {
            secL++;
            if (secL > 35) secL = 0;
        }
        int rightL = secL;
        int size = 1;

        int i = secL + 1;
        if (i > 35) i = 0;
        while (temp[i] == 1) {
            size++;
            i++;
            if (i > 35) i = 0;
            if (i == secL) break;
            // Smax here
            if (size >= sMax) break; //tried 10, same behaviour
        }
        int leftL = i - 1;
        if (leftL < 0) leftL = 35;
        if (size >= sMax) //wide
            dirL = rightL * 10 + 0.5 * 10 * 5;
        else if(size>4 && size<sMax)  //narrow
            dirL = 0.5 * (leftL * 10 + rightL * 10);
        else
            dirL=9999;
        ///////////////////////////////////////////////////////////////////
        int secR = R;
        while (temp[secR] != 1) {
            secR--;
            if (secR < 0) secR = 35;
        }

        int leftR = secR;
        int sizeR = 1;

        int j = secR - 1;
        if (j < 0) j = 35;
        while (temp[j] == 1) {
            sizeR++;
            j--;
            if (j < 0) j = 35;
            if (j == secR) break;
            if (sizeR >= sMax) break;
        }
        int rightR = j + 1;
        if (rightR > 35) rightR = 0;
        if (sizeR >= sMax) //wide
            dirR = leftR * 10 + 0.5 * 10 * 5;
        else if(sizeR>4 && sizeR<sMax)//narrow
            dirR = 0.5 * (rightR * 10 + leftR * 10);
        else
            dirR=9999;

        if(dirL>360) dirL=fabs(dirL-360);
        if(dirR>360) dirR=fabs(dirR-360);
        if(fabs(theta_deg-dirL)>fabs(theta_deg-dirR))
            dirC=dirR;
        else
            dirC=dirL;
        }
        dirSet=dirC;
        //cout<<"dirSet: 1"<<endl;

    ///////////////////////////////////////////////////////////
    } else {
        int secL = destSec;
        while (temp[secL] != 1) {
            secL++;
            if (secL > 35) secL = 0;
        }
        int rightL = secL;
        int size = 1;

        int i = secL + 1;
        if (i > 35) i = 0;
        while (temp[i] == 1) {
            size++;
            i++;
            if (i > 35) i = 0;
            if (i == secL) break;
            // Smax here
            if (size >= sMax) break; //5
        }
        int leftL = i - 1;
        if (leftL < 0) leftL = 35;
        if (size >= sMax) //wide
            dirL = rightL * 10 + 0.5 * 10 * 5;
        else if(size>4 && size<sMax)  //narrow
            dirL = 0.5 * (leftL * 10 + rightL * 10);
        else
            dirL=9999;
        ///////////////////////////////////////////////////////////////////
        int secR = destSec;
        while (temp[secR] != 1) {
            secR--;
            if (secR < 0) secR = 35;
        }

        int leftR = secR;
        int sizeR = 1;

        int j = secR - 1;
        if (j < 0) j = 35;
        while (temp[j] == 1) {
            sizeR++;
            j--;
            if (j < 0) j = 35;
            if (j == secR) break;
            if (sizeR >= sMax) break;
        }
        int rightR = j + 1;
        if (rightR > 35) rightR = 0;
        if (sizeR >= sMax) //wide
            dirR = leftR * 10 + 0.5 * 10 * 5;
        else if(sizeR>4 && sizeR<sMax)//narrow
            dirR = 0.5 * (rightR * 10 + leftR * 10);
        else
            dirR=9999;

        if(dirL>360) dirL=fabs(dirL-360);
        if(dirR>360) dirR=fabs(dirR-360);
        if(fabs(theta_deg-dirL)>fabs(theta_deg-dirR))
            dirSet=dirR;
        else
            dirSet=dirL;
        //cout<<"dirSet:2 dirR: "<<dirR<<" dirL: "<<dirL<<endl;
    }
    //cout<<"dirSet: "<<dirSet<<endl;
    fX=cos(dirSet*M_PI/180.0f);
    fY=sin(dirSet*M_PI/180.0f);
}

void mapping(){
    float thetaR_deg, readAngle, lDist, lAng;
    int xR, yR, xL, yL, cX[400], cY[400], npts, xF, yF;
    //------------Processing LIDAR readings------------
    lDist = data.distance / 10; //mm -> cm
    if(lDist == 0 || lDist > 200) return; //TO DO (opt): if less than ~10cm! -> make a turn around 
    lAng = data.angle;
    thetaR_deg = (p[2] * 180.0f) / M_PI; //TO DO: Add the robot's angle in world frame to the lidar's reagin angle (readAngle)
    if(thetaR_deg < 0) thetaR_deg = 360 + thetaR_deg;
    readAngle = 270 - lAng + thetaR_deg; //Align LIDAR's frame with robot's one
    if(readAngle > 360) readAngle -= 360; //   "     "      "      "      "
    else if(readAngle < 0) readAngle += 360;// "     "      "      "      "
    //pc.printf("ReadAngle: %f | data_deg: %f | Distance: %f | pos: (%f,%f)\n", readAngle, lAng, lDist, p[0], p[1]);
    readAngle = (readAngle * M_PI) /180.0f; // deg -> rads
    xL = lDist * cos(readAngle)/5;
    yL = lDist * sin(readAngle)/5;
    xR = p[0]/5, yR = p[1]/5; // cm -> cell index units
    xF = xR + xL; //(xR - xL) -> delta distance THINK IS WRONG should be delta = yL-yR;
    yF = yR + yL;//(yR - yL) -> delta distance  THINK IS WRONG should be delta = yL-yR;
    pc.printf("xF: %d | yF: %d", xF, yF);
	if(xF < 0 || yF < 0 || xF > 80 || yF > 80) return;
    //pc.printf("xF: %d | yF: %d", xF, yF);
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
    //pc.printf("x = %d | y = %d | cx = %d | cy = %d | aux = %d \n", x1, y1, cX[aux-1], cY[aux-1], aux-1);
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
            //pc.printf("x = %d | y = %d | cx = %d | cy = %d | aux = %d \n", x1, y1, cX[aux-1], cY[aux-1], aux-1);
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
            //pc.printf("x = %d | y = %d | cx = %d | cy = %d | aux = %d \n", x1, y1, cX[aux-1], cY[aux-1], aux-1); 
        }
    }
    return aux;
}
void log_cell(int cX[], int cY[], int npts){
    static const float l_occ = 0.65;
    static const float l_free = -0.65;
    for(int i = 0; i < (npts-2); i++){
        if(cX[i] < 0 || cY[i] < 0 || cX[i] > hSize || cY[i] > hSize) return;
        //pc.printf("I'm running l_free!!!!!!!!!! %d time\n cX[i] = %d | cY[i] = %d \n", i, cX[i], cY[i]); //DEBUg
        histogram[cX[i]][cY[i]].logodds = histogram[cX[i]][cY[i]].logodds + l_free; //l0 já inicializado com o array
    }
    /*for (int i = -1; i < 2; i++){
        for (int j = -1; j < 2; j++){
            //pc.printf("cX[%d] + (%d) = %d | cY[%d] + (%d) = %d\n", (npts-1), i, cX[npts-1]+i, (npts-1), j, cY[npts-1]+j);
            if(i == 0 || j == 0) continue;
            histogram[cX[npts-1]+i][cY[npts-1]+j].logodds = histogram[cX[npts-1]+i][cY[npts-1]+j].logodds + l_occ/2; //ATTENTION **
        }
    }*/
    histogram[cX[npts-1]][cY[npts-1]].logodds = histogram[cX[npts-1]][cY[npts-1]].logodds + l_occ; //ATTENTION **
//Força precisa de ser calculada de log odds para probabilidades: <50% de ocupação = livre
//Se o valor da repulsiva óptimo era 3, então vai oscilar entre 0 e 3 - 50 e 100%
}
void prob_calc(){
    float aux;
    for (int i = 0; i < hSize; i++) {
        for (int j = 0; j < hSize; j++) {
            aux = (1.0f - (1.0f/(1.0f+exp(histogram[i][j].logodds))));
            if(aux > 0.5f){
                histogram[i][j].cellVal = 12*aux; //6
            }
        }
    }
}
