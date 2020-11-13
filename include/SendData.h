//
// Created by wh on 2020/11/6.
//

#ifndef ARUCO_ROAD_SENDDATA_H
#define ARUCO_ROAD_SENDDATA_H

#include <pcl/segmentation/sac_segmentation.h>
class SendData {
private:
    
public:
    SendData(){};
    ~SendData(){};
    bool sendData(double X, double Y, double distance, double Xs, double Ys, double zs);
};


#endif //ARUCO_ROAD_SENDDATA_H