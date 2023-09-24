#include <ros/ros.h>      
#include <image_transport/image_transport.h>      
#include <cv_bridge/cv_bridge.h>      
#include <sensor_msgs/image_encodings.h>      
#include <opencv2/opencv.hpp>      
  
image_transport::Publisher pub; // 定义全局变量
int width=200, height=200; // 区域的宽和高
  
void imageCallback(const sensor_msgs::ImageConstPtr& msg, image_transport::ImageTransport& it) {      
    // 将图像消息转换为OpenCV格式      
  
    cv_bridge::CvImagePtr cv_ptr;      
    try {      
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);      
    } catch (cv_bridge::Exception& e) {      
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());      
        return;      
    }      
    cv::Mat img = cv_ptr->image;

    int x1 = 100;
    int y1 = 100;
    
    // 定义ROI区域  
    cv::Rect roi(x1, y1, width, height); // 这里的参数可以根据你的需要进行修改
    
    // 检查ROI区域是否在图像边界内  
    if (roi.x + roi.width > img.cols || roi.y + roi.height > img.rows) {  
        ROS_ERROR("ROI area is out of image bounds.");  
        return;  
    }  
  
    // 在原图上绘制ROI区域  
    cv::rectangle(img, roi, cv::Scalar(0, 255, 0), 2); // 用绿色线条绘制矩形  
    
    // 将结果转换为ROS图像消息并发布到另一个话题中      
    sensor_msgs::ImagePtr msg_edges = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
    pub.publish(msg_edges); // 使用全局变量pub     
}      
      
int main(int argc, char** argv) {      
    // 初始化ROS节点      
    ros::init(argc, argv, "image_processor");      
    ros::NodeHandle nh;      
      
    // 创建一个图像传输对象      
    image_transport::ImageTransport it(nh);  
  
    pub = it.advertise("camera/roi_image", 1); // 初始化全局变量pub        
      
    // 创建一个订阅者，订阅图像话题，并指定回调函数      
    image_transport::Subscriber sub = it.subscribe("camera/image", 1, boost::bind(&imageCallback, _1, boost::ref(it)));      
      
    // 循环等待回调函数处理图像消息      
    ros::spin();      
      
    return 0;      
}
