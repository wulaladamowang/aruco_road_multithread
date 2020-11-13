//
// Created by wh on 2020/11/6.
//

#include "SendData.h"
/*
 * 将检测的目标的位姿参数发送或者进行显示
 * coefficients: 拟合的目标物的参数，在拟合结果中，参数为轴线的法向量以及轴线上一点的坐标，由于轴线上的坐标点不确定，所以自己进行计算
 * X,Y,distance: 目标物体的位移参数，目标框中点附近的三维坐标
 */
bool SendData::sendData(double X, double Y, double distance,  double Xs, double Ys, double zs){
    std::cout << "X  : " << X << "米   Y: " << Y << "米   Z: " <<distance << "米" << "\n"
              << "X  : " <<  Xs << "度   Y: " <<  Ys << "度   Z: " <<  zs << "度" << std::endl;
    //增加条件， 判断是否发送成功
    return true;
};