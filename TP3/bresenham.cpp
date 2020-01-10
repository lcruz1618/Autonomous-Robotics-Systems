#include <cstdlib>
#include <iostream>
#include <cmath>

 

////////////////////////////////////////////////////////////////////////////////
void Bresenham(float x1, float y1, int const x2, int const y2)
{
	/*float px1 = x1/5;
	float py1 = y1/5
	std::cout << px1 << " e " << py1 << std::endl;
	std::cout << ceil(px1)<< " e " << ceil(py1) << std::endl;*/
    int delta_x(x2 - x1);
    int aux = 0;
    // if x1 == x2, then it does not matter what we set here
    signed char const ix((delta_x > 0) - (delta_x < 0));
    delta_x = std::abs(delta_x) << 1;
 
    int delta_y(y2 - y1);
    // if y1 == y2, then it does not matter what we set here
    signed char const iy((delta_y > 0) - (delta_y < 0));
    delta_y = std::abs(delta_y) << 1;
 
    std::cout << x1 << " e " << y1 << std::endl;
    aux++;
 
    if (delta_x >= delta_y)
    {
        // error may go below zero
        int error(delta_y - (delta_x >> 1));
 
        while (x1 != x2)
        {
            // reduce error, while taking into account the corner case of error == 0
            if ((error > 0) || (!error && (ix > 0)))
            {
                error -= delta_x;
                y1 += iy;
            }
            // else do nothing
 
            error += delta_y;
            x1 += ix;
 
            std::cout << x1 << " e " << y1 << std::endl;
            aux++;
        }
    }
    else
    {
        // error may go below zero
        int error(delta_x - (delta_y >> 1));
 
        while (y1 != y2)
        {
            // reduce error, while taking into account the corner case of error == 0
            if ((error > 0) || (!error && (iy > 0)))
            {
                error -= delta_y;
                x1 += ix;
            }
            // else do nothing
 
            error += delta_x;
            y1 += iy;
 
            std::cout << x1 << " e " << y1 << std::endl;
            aux++;
        }
    }

    std::cout << aux << std::endl;
}

int main(){
	int lDist = 20;
	float lAng = M_PI/2+M_PI/4;
	int xL = lDist * cos(lAng);
    int yL = lDist * sin(lAng);
    int xR = 20/5, yR = 20/5;
	xL = xL/5, yL = yL/5;
    //std::cout << xL << " , " << yL << std::endl;
	xL = xR + xL; //(xR - xL) -> delta distance
    yL = yR + yL;
    //std::cout << xL << " , " << yL << std::endl;
    Bresenham(2, 2, 5, 2);
	//Bresenham(4, 4, 8, 8);
	return 0;
}
