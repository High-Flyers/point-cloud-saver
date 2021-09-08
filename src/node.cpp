#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <string>
#include <tf/transform_listener.h>
#include <fstream>
#include <tuple>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
#define MAX_DISTANCE 18.0f
#define MAX_TRANSLATION 0.4f
#define MAX_ROTATION 0.17f      // 10 degrees

static tf::TransformListener *listener_ptr;
static std::fstream dataFile;
static bool shouldUpdate = false;

inline std::tuple<double, double, double> quaterionToEuler(double w, double x, double y, double z)
{
    double xAxis1 = 2 * (w*x + y*z);
    double xAxis2 = 1 - 2 * (x*x + y*y);
    double roll = std::atan2(xAxis1, xAxis2);

    double sinp = 2 * (w*y - z*x);
    double pitch;
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);
    
    double siny_cosp = 2 * (w*z + x*y);
    double cosy_cosp = 1 - 2 * (y*y + z*z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return {roll, pitch, yaw};
}

void callback(const PointCloud::ConstPtr &msg)
{
    if(shouldUpdate)
    {
        shouldUpdate = false;
        static uint32_t i = 0;
        std::cout << "write... " << i++ << std::endl;
        std::vector<pcl::PointXYZRGB, Eigen::aligned_allocator<pcl::PointXYZRGB>> cleanPointsVec;
        for(auto &pt : msg->points) {
            if(pt.z < MAX_DISTANCE) {
                pcl::PointXYZRGB tmpPt;
                tmpPt.x = pt.x; tmpPt.y = pt.y; tmpPt.z = pt.z;
                tmpPt.r = pt.r; tmpPt.g = pt.g; tmpPt.b = pt.b;
                cleanPointsVec.push_back(tmpPt);
            }
        }

        PointCloud cleanCloud; cleanCloud.points = cleanPointsVec;
        cleanCloud.header = msg->header; cleanCloud.height = msg->height; cleanCloud.width = msg->width;
        cleanCloud.is_dense = msg->is_dense;
        pcl_ros::transformPointCloud(std::string("map"), cleanCloud, cleanCloud, *listener_ptr);
        uint8_t counter = 0;
        for(auto &pt : cleanCloud.points) {
            if(counter++ % 4 == 0)
                //dataFile << pt.x << ' ' << pt.y << ' ' << pt.z << ' ' << (int)pt.r << ' ' << (int)pt.g << ' ' << (int)pt.b << '\n';
                dataFile << pt.x << ' ' << pt.y << ' ' << pt.z << '\n';
        }
        dataFile.flush();
    }
}

void callbackTf(const tf::tfMessageConstPtr &msg)
{
    static geometry_msgs::TransformStamped prevTransform;
    for(auto &tf : msg->transforms)
    {
        if(tf.header.frame_id == "odom") {
            
            double dist = std::sqrt(
                std::pow(tf.transform.translation.x - prevTransform.transform.translation.x, 2.0f) + 
                std::pow(tf.transform.translation.y - prevTransform.transform.translation.y, 2.0f) + 
                std::pow(tf.transform.translation.z - prevTransform.transform.translation.z, 2.0f));
            auto currAngle = quaterionToEuler(
                tf.transform.rotation.w,
                tf.transform.rotation.x,
                tf.transform.rotation.y,
                tf.transform.rotation.z);
            auto prevAngle = quaterionToEuler(
                prevTransform.transform.rotation.w,
                prevTransform.transform.rotation.x,
                prevTransform.transform.rotation.y,
                prevTransform.transform.rotation.z);
            
            bool distCondition = dist >= MAX_TRANSLATION;
            bool angleCondition = 
                std::fabs(std::get<0>(currAngle) - std::get<0>(prevAngle)) >= MAX_ROTATION ||
                std::fabs(std::get<1>(currAngle) - std::get<1>(prevAngle)) >= MAX_ROTATION ||
                std::fabs(std::get<2>(currAngle) - std::get<2>(prevAngle)) >= MAX_ROTATION;

            if(distCondition || angleCondition)
            {
                shouldUpdate = true;
                prevTransform.transform.translation.x = tf.transform.translation.x;
                prevTransform.transform.translation.y = tf.transform.translation.y;
                prevTransform.transform.translation.z = tf.transform.translation.z;
                prevTransform.transform.rotation.w = tf.transform.rotation.w;
                prevTransform.transform.rotation.x = tf.transform.rotation.x;
                prevTransform.transform.rotation.y = tf.transform.rotation.y;
                prevTransform.transform.rotation.z = tf.transform.rotation.z;
            }
                
            
            break;
        }
    }
}

int main(int argc, char **argv)
{   
    dataFile.open("dataTest.xyz", std::fstream::out);
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Time::init();
    ros::Rate rate(50);
    tf::TransformListener listener;
    listener_ptr = &listener;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/camera/depth/points", 1, callback);
    ros::Subscriber subTf = nh.subscribe<tf::tfMessage>("/tf", 1, callbackTf);
    ros::spin();
}
