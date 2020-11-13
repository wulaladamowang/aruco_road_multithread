//
// Created by wh on 2020/11/6.
//
#include "GetPosition.h"
/*
 * 用来进行将点云拟合为圆柱
 * p_roi_point_cloud: 提取到的目标区域的点云
 * coefficients_cylinder：拟合获得的圆柱的参数
 * inliers_cylinder: 用于拟合的三维点可以通过此参数提取出来
 */
bool GetPosition::setPointCloudToPosition(pcl::PointCloud<pcl::PointXYZ>::Ptr p_roi_point_cloud, double &x_degree, double& y_degree,  double& z_degree){
    ///计算法向量
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);
    ne.setInputCloud(p_roi_point_cloud);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);//设置分割模型为圆柱
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.1);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold (0.05);
    seg.setRadiusLimits(0.10, 0.20);
    seg.setInputCloud(p_roi_point_cloud);
    seg.setInputNormals(cloud_normals);
    
    seg.segment (*inliers_cylinder, *coefficients_cylinder);
    if(inliers_cylinder->indices.size()==0)
    {
        x_degree = 0;
        y_degree = 0;
        z_degree = 0;
        return false;
    }
        
    //可以增加条件用来判断是否满足条件，例如，两次获得坐标的差距应该满足条件
    if(coefficients_cylinder->values[3] > 0)
        x_degree = acos(coefficients_cylinder->values[3])/pi*180;
    else
        x_degree = -acos(coefficients_cylinder->values[3])/pi*180;
    if(coefficients_cylinder->values[3] > 0)
        y_degree = acos(coefficients_cylinder->values[4])/pi*180;
    else
        y_degree = -acos(coefficients_cylinder->values[4])/pi*180;
    if(coefficients_cylinder->values[3] > 0)
        z_degree = acos(coefficients_cylinder->values[5])/pi*180;
    else
        z_degree = -acos(coefficients_cylinder->values[5])/pi*180;
    x_degree = x_degree>0?x_degree:180+x_degree;
    y_degree = y_degree>0?y_degree:180+y_degree;
    z_degree = z_degree>0?z_degree:180+z_degree;
    
    return true;
}