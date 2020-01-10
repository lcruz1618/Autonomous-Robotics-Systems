#ifndef UNTITLED2_ACTIVECELL_H
#define UNTITLED2_ACTIVECELL_H
#include <iostream>
#include <iomanip>
#include <math.h>

class ActiveCell {
    
public:
    
    //cell dimension = 5x5cm
    int cellVal;
    float forceX;
    float forceY;
    float distance;
    int xt, yt;
    static const float repulsiveForce = 20;
    //robots position
    int x0;
    int y0;
    ///////////VFH//////////////
    static const int a=70,b=2;
    float angle;
    float amplitude;
    int sectorK;
    //resolution - if changing change also secVal in main & calcSectors
    static const int res=10;

    ActiveCell()
    {
        angle=0;
        amplitude=0;
        sectorK=0;
        cellVal=0;
        forceX=0;
        forceY=0;
        x0=0;
        y0=0;
    }


    void calForce()
    {
        forceX = (repulsiveForce * cellVal / pow(distance, 2)) * (xt - x0) / distance;
        forceY = (repulsiveForce * cellVal / pow(distance, 2)) * (yt - y0) / distance;
        amplitude = cellVal*cellVal*(a-b*distance);
    }
    
//Calculating distance from the robot 
    void calDist(int idX, int idY)
    {
        
        if (idX < 5)
        {
            xt = x0 - (5 - idX) * 5;
        } 
        else if (idX == 5) 
        {
            xt = x0;
        } 
        else 
        {
            xt = x0 + (idX - 5) * 5;
        }
        
        if (idY > 5) 
        {
            yt = y0 - (5 - idY) * 5;
        } 
        else if (idY == 5) 
        {
            yt = y0;
        } 
        else 
        {
            yt = y0 + (idY - 5) * 5;
        }
        distance = sqrt(pow((float)abs(x0 - xt), 2) + pow((float)abs(y0 - yt), 2));
        
        //////angle/////////////
        angle = atan2((float)yt-y0,(float)xt-x0)*180/3.14159265358979323846f;
        sectorK=angle/res;
        if(sectorK<0)
        {
            sectorK=36+sectorK;
        }
    }
};


#endif //UNTITLED2_ACTIVECELL_H
