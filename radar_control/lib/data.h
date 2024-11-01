#ifndef DATA_H
#define DATA_H


struct SmallRadarData {
    unsigned int Id;
    double Priority;
    double Rad;
    double Ang;
    double H;
};

struct BigRadarData : public SmallRadarData {
    double SpeedX;
    double SpeedY;
    double SpeedZ;
};


#endif // DATA_H
