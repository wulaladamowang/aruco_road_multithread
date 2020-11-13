#include <iostream>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/time.h>
#include <signal.h>

#include "GetImg.h"
#include "GetRoi.h"
#include "GetMarker.h"
#include "GetRoiPointCloud.h"
#include "GetPosition.h"
#include "SendData.h"

bool stop = false;//控制整体程序是否停止运行

cv::Mat img_left;
sl::Mat point_cloud;

cv::Mat img_left1;
sl::Mat point_cloud1;
//锁，获得图像的下一阶段是检测marker
bool get_img = false;
std::mutex img_mutex;
std::condition_variable img_var;

void getImage(){
    //GetImg getImg("/home/wang/文档/ZED/HD1080_1015.svo");
    GetImg getImg;
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    while(!stop){
        gettimeofday(&t2, NULL);
        std::cout << " 1运行时间间隔："<< ((t2.tv_sec-t1.tv_sec) * 1000 + (t2.tv_usec-t1.tv_usec)/1000) << "毫秒" << std::endl;
        gettimeofday(&t1, NULL);
        getImg.grubImage();
        getImg.getImage("left",img_left1);
        getImg.getPointCloud(point_cloud1);
        std::unique_lock<std::mutex> lck(img_mutex);
        img_var.wait(lck, []{return !get_img;});

        img_left1.copyTo(img_left);
        point_cloud1.copyTo(point_cloud);

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
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    while(!stop){
        gettimeofday(&t2, NULL);
        std::cout << " 2运行时间间隔："<< ((t2.tv_sec-t1.tv_sec) * 1000 + (t2.tv_usec-t1.tv_usec)/1000) << "毫秒" << std::endl;
        gettimeofday(&t1, NULL);
        std::unique_lock<std::mutex> lck(img_mutex);
        img_var.wait(lck, []{return get_img;});
        std::unique_lock<std::mutex> lck_cor(cor_mutex);
        cor_var.wait(lck_cor, []{return !have_corner;});

        img_left.copyTo(img_detect_copy);
        point_cloud.copyTo(point_cloud_copy);

        get_img = false;
        have_corner = true;
        img_var.notify_one();
        cor_var.notify_one();

        getMarker.detectMarkers(img_left, marker_corners, marker_ids);
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

bool have_position = false;


void getPosition(){
    GetRoi getRoi(img_left.size());
    GetRoiPointCloud getRoiPointCloud;
    GetPosition getPosition;
    struct timeval t1, t2;
    gettimeofday(&t1, NULL);
    while(!stop){
        
        gettimeofday(&t2, NULL);
        std::cout << " 3运行时间间隔："<< ((t2.tv_sec-t1.tv_sec) * 1000 + (t2.tv_usec-t1.tv_usec)/1000) << "毫秒" << std::endl;
        gettimeofday(&t1, NULL);
        std::unique_lock<std::mutex> lck_cor(cor_mutex);
        cor_var.wait(lck_cor, []{return have_corner;});

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
        cor_var.notify_one();
    }

}

void post_data(int signo){
    static SendData sendData;
    static struct timeval t1, t2;
    sendData.sendData(X, Y, distance, X_degree, Y_degree, z_degree);
    //std::cout << "hello: " << ++count << std::endl;
    gettimeofday(&t2, NULL);
    std::cout << " 时间间隔："<< ((t2.tv_sec-t1.tv_sec) * 1000 + (t2.tv_usec-t1.tv_usec)/1000) << "毫秒" << std::endl;
    gettimeofday(&t1, NULL);
}

int main() {

    signal(SIGALRM, post_data);
    struct itimerval tick;
    
    signal(SIGALRM, post_data);
    memset(&tick, 0, sizeof(tick));

        //Timeout to run first time
    tick.it_value.tv_sec = 5;
    tick.it_value.tv_usec = 0;

    //After first, the Interval time for clock
    tick.it_interval.tv_sec = 0;
    tick.it_interval.tv_usec = 20000;

    setitimer(ITIMER_REAL, &tick, nullptr);
    marker_corners.reserve(30);
    marker_ids.reserve(30);
    std::thread t1(getImage);
    sleep(10);//sleep 保证相机正常启动后一些参数的获得
    std::thread t2(detectArucoMarker);
    std::thread t3(getPosition);

    char key = ' ';
    while(key!='q'){
        key = cv::waitKey(10);
        cv::imshow("img_left", img_left);
    }
    stop = true;
    t1.join();
    t2.join();
    t3.join();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}
