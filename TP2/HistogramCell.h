#ifndef UNTITLED2_HISTOGRAMCELL_H
#define UNTITLED2_HISTOGRAMCELL_H


class HistogramCell {
public:
    int cellVal;
    float x;
    float y;

    void calculate(int idX, int idY)
    {
        //this is cells position in coordinate system
        // we add 2,5cm to get position of middle point of the cell not its starting point
        x=idX*5+2.5;
        y=idY*5+2.5;
    }
};


#endif //UNTITLED2_HISTOGRAMCELL_H
