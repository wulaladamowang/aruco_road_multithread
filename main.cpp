#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/time.h>

#include "GetImg.h"
#include "GetRoi.h"
#include "GetMarker.h"
#include "GetRoiPointCloud.h"
#include "GetPosition.h"
#include "SendData.h"

bool stop = false;//控制整体程序是否停止运行

cv::Mat img_left;
sl::Mat point_cloud;
//锁，获得图像的下一阶段是检测marker
bool get_img = false;
std::mutex img_mutex;
std::condition_variable img_var;

void getImage(){
    //GetImg getImg("/home/wang/文档/ZED/HD1080_1015.svo");
    GetImg getImg;
    while(!stop){
        std::unique_lock<std::mutex> lck(img_mutex);
        img_var.wait(lck, []{return !get_img;});

        getImg.grubImage();
        getImg.getImage("left",img_left);
        getImg.getPointCloud(point_cloud);


        get_img = true;
        img_var.notify_one();
    }
}

cv::Mat img_detect_copy;//将上一阶段的img保存，在提取区域点云过程中再次用到了该图像
sl::Mat point_cloud_copy;//同上， 保存点云图像
std::vector<std::vector<cv::Point2f>> marker_corners;
std::vector<int> marker_ids;
//锁，获得角点的下一个阶段是获得roi
bool have_corner = false;
std::mutex cor_mutex;
std::condition_variable cor_var;

void detectArucoMarker(){
    GetMarker getMarker;
    while(!stop){
        std::unique_lock<std::mutex> lck(img_mutex);
        img_var.wait(lck, []{return get_img;});
        std::unique_lock<std::mutex> lck_cor(cor_mutex);
        cor_var.wait(lck_cor, []{return !have_corner;});

        getMarker.detectMarkers(img_left, marker_corners, marker_ids);
        img_detect_copy = img_left.clone();
        point_cloud.copyTo(point_cloud_copy);

        get_img = false;
        have_corner = true;
        img_var.notify_one();
        cor_var.notify_one();
    }
}

cv::Mat mask;
cv::Rect rect_roi;

float distance = 0;
float X = 0;
float Y = 0;
double z_degree = 0;
double X_degree = 0;
double Y_degree = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr p_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::visualization::CloudViewer viewer("Point Cloud");

bool have_position
 = false;
std::mutex position_mutex;
std::condition_variable position_var;

void getPosition(){
    GetRoi getRoi(img_left.size());
    GetRoiPointCloud getRoiPointCloud;
    GetPosition getPosition;

    while(!stop){
        std::unique_lock<std::mutex> lck_cor(cor_mutex);
        cor_var.wait(lck_cor, []{return have_corner;});
        std::unique_lock<std::mutex> lck_position(position_mutex);
        position_var.wait(lck_position, []{return !have_position;});

        if(marker_corners.size()>0){
            getRoi.setCornerToMask(marker_corners, marker_ids);
            getRoi.getMask(mask);
            getRoi.getMaskRect(rect_roi);

            bool is_get_point_cloud = getRoiPointCloud.setMaskToPointCloud(img_detect_copy, p_point_cloud, rect_roi, mask, point_cloud_copy);
            if(is_get_point_cloud)
            {
                getRoiPointCloud.getPosition(X, Y, distance);
                getPosition.setPointCloudToPosition(p_point_cloud, X_degree, Y_degree, z_degree);
            }else
            {
                X = 0;
                Y = 0;
                distance = 0;
                z_degree = 0;
                X_degree = 0;
                Y_degree = 0;
            }
            viewer.showCloud(p_point_cloud);
        }else
        {
            X = 0;
            Y = 0;
            distance = 0;
            z_degree = 0;
            X_degree = 0;
            Y_degree = 0;
        }
        
        have_corner = false;
        have_position = true;
        cor_var.notify_one();
        position_var.notify_one();
    }

}

void sendData(){
    SendData sendData;
    while(!stop){
        std::unique_lock<std::mutex> lck_position(position_mutex);
        position_var.wait(lck_position, []{return have_position;});

        sendData.sendData(X, Y, distance, X_degree, Y_degree, z_degree);

        have_position = false;
        position_var.notify_one();
    }
}

int main() {
    marker_corners.reserve(30);
    marker_ids.reserve(30);
    std::thread t1(getImage);
    sleep(3);//sleep 保证相机正常启动后一些参数的获得
    std::thread t2(detectArucoMarker);
    std::thread t3(getPosition);
    std::thread t4(sendData);

    char key = ' ';
    while(key!='q'){
        key = cv::waitKey(10);
        cv::imshow("img_left", img_left);
    }
    
    stop = true;
    t1.join();
    t2.join();
    t3.join();
    t4.join();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
