#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <chrono>
#include <iomanip>
#include <string>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <istream>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <numeric>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <fstream>
#include "utils/kinetic_math.hpp"
#include "utils/cv.hpp"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

/*uav local parameter*/
static geometry_msgs::PoseStamped Aruco_pose_realsense;
static geometry_msgs::PoseStamped Depth_pose_realsense;
static geometry_msgs::PoseStamped LED_pose_realsense;
static geometry_msgs::PoseStamped Camera_pose_vicon;
/* System */
int coutcounter = 0;
static Vec7 Zero7;
static Vec4 Zero4;

void camera_pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pose){
    Camera_pose_vicon.pose.position.x = pose->pose.position.x;
    Camera_pose_vicon.pose.position.y = pose->pose.position.y;
    Camera_pose_vicon.pose.position.z = pose->pose.position.z;
    Camera_pose_vicon.pose.orientation.x = pose->pose.orientation.x;
    Camera_pose_vicon.pose.orientation.y = pose->pose.orientation.y;
    Camera_pose_vicon.pose.orientation.z = pose->pose.orientation.z;
    Camera_pose_vicon.pose.orientation.w = pose->pose.orientation.w;
}
void Aruco_PosePub(Vec6 rpyxyz){
    Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    // Quaterniond Q = rpy2Q(Vec3(0,0,3.14));
    Aruco_pose_realsense.header.stamp = ros::Time::now();
    Aruco_pose_realsense.header.frame_id = "world";
    Aruco_pose_realsense.pose.position.x = rpyxyz(3);
    Aruco_pose_realsense.pose.position.y = rpyxyz(4);
    Aruco_pose_realsense.pose.position.z = rpyxyz(5);
    Aruco_pose_realsense.pose.orientation.w = Q.w();
    Aruco_pose_realsense.pose.orientation.x = Q.x();
    Aruco_pose_realsense.pose.orientation.y = Q.y();
    Aruco_pose_realsense.pose.orientation.z = Q.z();
}
void Depth_PosePub(Vec6 rpyxyz){
    Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    Depth_pose_realsense.header.stamp = ros::Time::now();
    Depth_pose_realsense.header.frame_id = "world";
    Depth_pose_realsense.pose.position.x = rpyxyz(3);
    Depth_pose_realsense.pose.position.y = rpyxyz(4);
    Depth_pose_realsense.pose.position.z = rpyxyz(5);
    Depth_pose_realsense.pose.orientation.w = Q.w();
    Depth_pose_realsense.pose.orientation.x = Q.x();
    Depth_pose_realsense.pose.orientation.y = Q.y();
    Depth_pose_realsense.pose.orientation.z = Q.z();
}
void LED_PosePub(Vec6 rpyxyz){
    Quaterniond Q = rpy2Q(Vec3(rpyxyz(0),rpyxyz(1),rpyxyz(2)));
    LED_pose_realsense.header.stamp = ros::Time::now();
    LED_pose_realsense.header.frame_id = "world";
    LED_pose_realsense.pose.position.x = rpyxyz(3);
    LED_pose_realsense.pose.position.y = rpyxyz(4);
    LED_pose_realsense.pose.position.z = rpyxyz(5);
    LED_pose_realsense.pose.orientation.w = Q.w();
    LED_pose_realsense.pose.orientation.x = Q.x();
    LED_pose_realsense.pose.orientation.y = Q.y();
    LED_pose_realsense.pose.orientation.z = Q.z();
}
Vec6 Camera2World(const Vec3 rvecs, const Vec3 tvecs){ // camera coordinate to world coordinate
    Eigen::Quaterniond q;
    q.w() = Camera_pose_vicon.pose.orientation.w;
    q.x() = Camera_pose_vicon.pose.orientation.x;
    q.y() = Camera_pose_vicon.pose.orientation.y;
    q.z() = Camera_pose_vicon.pose.orientation.z;
    Eigen::Matrix3d Camera_Rotation_world = Eigen::Matrix3d::Identity();
    Camera_Rotation_world = q.matrix();
    Vec3 Cam_Translation_world(Camera_pose_vicon.pose.position.x, Camera_pose_vicon.pose.position.y, Camera_pose_vicon.pose.position.z);
    Vec3 translation_world = Camera_Rotation_world * tvecs + Cam_Translation_world;
    Vec3 rpy_world = Camera_Rotation_world*rvecs;
    Vec6 output;
    output << rpy_world[0],rpy_world[1],rpy_world[2],
              translation_world[0],translation_world[1],translation_world[2];
    return(output);
}
void Aruco_process(Mat image_rgb, const sensor_msgs::ImageConstPtr &depth){
    cv::Mat ArucoOutput = image_rgb.clone();
    std::vector<int> markerIds;
    std::vector<Vec8I> markerConerABCDs;
    Vec2I markerCenter,last_markerCenter;
    Vec8I markerConerABCD;
    Vec8I last_markerConerABCD;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    std::vector<cv::Point2f> markerCorner;
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3d rvec, tvec;
    rvecs.clear();tvecs.clear();
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    if (markerIds.size() > 0){
        markerConerABCDs.clear();
        Aruco_init = true;
        Aruco_found = true;
        cv::aruco::drawDetectedMarkers(ArucoOutput, markerCorners, markerIds);
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, cameraMatrix, distCoeffs, rvecs, tvecs);
        for(unsigned int i=0; i<markerIds.size(); i++){
            cv::aruco::drawAxis(ArucoOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
            markerCorner = markerCorners[i];
            for (unsigned int j=0; j<markerCorner.size();j++){
                cv::Point2f MC = markerCorner[j];
                markerConerABCD[j*2] = MC.x;
                markerConerABCD[j*2+1] = MC.y;
            }
            markerConerABCDs.push_back(markerConerABCD);
        }
        // if (markerIds.size() > 1 ){cout << "Aruco Warning" << endl;}
    }else{Aruco_found = false; ArucoLostcounter++;}
    if (Aruco_init == false){
        CVE_Corners.clear();
        Vec8I CVE_Corner;
        CVE_Corner << 0,0,0,0,0,0,0,0;
        for(int i=0; i<5; i++){CVE_Corners.push_back(CVE_Corner);}
    }
    if (Aruco_init){
        cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
        cv::Mat image_dep = depth_ptr->image;
        Vec3 Depthrvecs;
        if(Aruco_found){
            rvec = rvecs.front();
            tvec = tvecs.front();
            Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
            Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
            Aruco_PosePub(Camera2World(Aruco_rpy_camera,Aruco_translation_camera));
            Depthrvecs = Aruco_rpy_camera;
            double ArucoDepth = find_depth_avg(image_dep,markerConerABCDs.back());
            last_markerConerABCD = markerConerABCDs.back();
            Depth_PosePub(Camera2World(Depthrvecs,camerapixel2tvec(Constant_velocity_predictor(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
            ArucoLostcounter = 0;
        }else{ //Aruco not found do constant-velocity predict
            double ArucoDepth = find_depth_avg(image_dep,last_markerConerABCD);
            Depth_PosePub(Camera2World(Depthrvecs,camerapixel2tvec(Constant_velocity_predictor(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
        }
    }
}
void callback(const sensor_msgs::CompressedImageConstPtr &rgb, const sensor_msgs::ImageConstPtr &depth){
    // cout<<"hello callback "<<endl;
    cv::Mat image_rgb;
    try{
        image_rgb = cv::imdecode(cv::Mat(rgb->data),1);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    /* Aruco */
    Aruco_process(image_rgb,depth);
    // cv::Mat ArucoOutput = image_rgb.clone();
    // std::vector<int> markerIds;
    // std::vector<Vec8I> markerConerABCDs;
    // Vec2I markerCenter,last_markerCenter;
    // Vec8I markerConerABCD;
    // Vec8I last_markerConerABCD;
    // std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    // std::vector<cv::Point2f> markerCorner;
    // std::vector<cv::Vec3d> rvecs, tvecs;
    // cv::Vec3d rvec, tvec;
    // rvecs.clear();tvecs.clear();
    // cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    // cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    // cv::aruco::detectMarkers(image_rgb, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
    // if (markerIds.size() > 0){
    //     markerConerABCDs.clear();
    //     Aruco_init = true;
    //     Aruco_found = true;
    //     cv::aruco::drawDetectedMarkers(ArucoOutput, markerCorners, markerIds);
    //     cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.06, cameraMatrix, distCoeffs, rvecs, tvecs);
    //     for(unsigned int i=0; i<markerIds.size(); i++){
    //         cv::aruco::drawAxis(ArucoOutput, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
    //         markerCorner = markerCorners[i];
    //         for (unsigned int j=0; j<markerCorner.size();j++){
    //             cv::Point2f MC = markerCorner[j];
    //             markerConerABCD[j*2] = MC.x;
    //             markerConerABCD[j*2+1] = MC.y;
    //         }
    //         markerConerABCDs.push_back(markerConerABCD);
    //     }
    //     // if (markerIds.size() > 1 ){cout << "Aruco Warning" << endl;}
    // }else{Aruco_found = false; ArucoLostcounter++;}
    // if (Aruco_init == false){
    //     CVE_Corners.clear();
    //     Vec8I CVE_Corner;
    //     CVE_Corner << 0,0,0,0,0,0,0,0;
    //     for(int i=0; i<5; i++){CVE_Corners.push_back(CVE_Corner);}
    // }
    /* Pose in World Calc */
    // if (Aruco_init){
    //     cv_bridge::CvImagePtr depth_ptr  = cv_bridge::toCvCopy(depth, depth->encoding);
    //     cv::Mat image_dep = depth_ptr->image;
    //     Vec3 Depthrvecs;
    //     if(Aruco_found){
    //         rvec = rvecs.front();
    //         tvec = tvecs.front();
    //         Vec3 Aruco_translation_camera(tvec(0),tvec(1),tvec(2));
    //         Vec3 Aruco_rpy_camera(rvec(0),rvec(1),rvec(2));
    //         Aruco_PosePub(Camera2World(Aruco_rpy_camera,Aruco_translation_camera));
    //         Depthrvecs = Aruco_rpy_camera;
    //         double ArucoDepth = find_depth_avg(image_dep,markerConerABCDs.back());
    //         last_markerConerABCD = markerConerABCDs.back();
    //         Depth_PosePub(Camera2World(Depthrvecs,camerapixel2tvec(Constant_velocity_predictor(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
    //         ArucoLostcounter = 0;
    //     }else{ //Aruco not found do constant-velocity predict
    //         double ArucoDepth = find_depth_avg(image_dep,last_markerConerABCD);
    //         Depth_PosePub(Camera2World(Depthrvecs,camerapixel2tvec(Constant_velocity_predictor(last_markerConerABCD,ArucoLostcounter),ArucoDepth,CamParameters)));
    //     }
    // }
    /* LED PNP */
    // Vec6 LEDtvecrvec = LEDTvecRvec(image_rgb);  
    // LED_PosePub(Camera2World(Vec3(LEDtvecrvec[3],LEDtvecrvec[4],LEDtvecrvec[5]),Vec3(LEDtvecrvec[0],LEDtvecrvec[1],LEDtvecrvec[2])));
    // cout << "Aruco Tvec: " << tvec*1000 << endl;

    /* image plot */
    // cv::imshow("dep_out", image_dep); 
}
int main(int argc, char **argv){
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe("/camera/aligned_depth_to_color/camera_info",1,camera_info_cb);
    ros::Subscriber camerapose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_d455/pose", 1, camera_pose_cb);
    ros::Publisher ArucoPose_pub = nh.advertise<geometry_msgs::PoseStamped>("ArucoPose",1);
    ros::Publisher DepthPose_pub = nh.advertise<geometry_msgs::PoseStamped>("DepthPose",1);
    ros::Publisher LEDPose_pub = nh.advertise<geometry_msgs::PoseStamped>("LEDPose",1);
    message_filters::Subscriber<CompressedImage> rgb_sub(nh, "/camera/color/image_raw/compressed", 1);
    message_filters::Subscriber<Image> dep_sub(nh, "/camera/aligned_depth_to_color/image_raw", 1);
    typedef sync_policies::ApproximateTime<CompressedImage, Image> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, dep_sub);
    sync.registerCallback(boost::bind(&callback, _1, _2));
    PNP3Dpoints();
    ros::Rate loop_rate(50); /* ROS system Hz */

    while(ros::ok()){
        ArucoPose_pub.publish(Aruco_pose_realsense);
        DepthPose_pub.publish(Depth_pose_realsense);
        LEDPose_pub.publish(LED_pose_realsense);
        /* ROS timer */
        // auto currentT = ros::Time::now().toSec();
        // cout << "System_Hz: " << 1/(currentT-LastT) << endl;
        // LastT = currentT;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
