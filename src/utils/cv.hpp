#ifndef CV_DEF_H
#define CV_DEF_H
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include "common.h"

using namespace cv;

/*camera pramater*/
static double fx, fy, cx, cy; //focal length and principal point
static Vec4 CamParameters;
int ArucoLostcounter;
bool Aruco_found = false;
bool Aruco_init = false;
/* Constant velocity estimator */
std::deque<Vec8I> CVE_Corners;
/*PNP*/
cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat depthcameraMatrix = cv::Mat::eye(3,3, CV_64F);
cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
vector<cv::Point3f> PNPPoints3D;
vector<Point2f> PNPPoints2D; //red orange green blue

void camera_info_cb(const sensor_msgs::CameraInfoPtr& msg){
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];
    cameraMatrix.at<double>(0,0) = fx;
    cameraMatrix.at<double>(1,1) = fy;
    cameraMatrix.at<double>(0,2) = cx;
    cameraMatrix.at<double>(1,2) = cy;
    CamParameters << fx,fy,cx,cy;
}
inline Vec2I FindMarkerCenter(const Vec8I& markerConerABCD){
    Vec2I MarkerCenter;
    MarkerCenter << (markerConerABCD[0]+markerConerABCD[2]+markerConerABCD[4]+markerConerABCD[6])/4,
                    (markerConerABCD[1]+markerConerABCD[3]+markerConerABCD[5]+markerConerABCD[7])/4;
    return(MarkerCenter);
}
inline Vec3 camerapixel2tvec (const Vec2I xy, double depth, Vec4 campaprameters ){
    return Vec3 (
                ( xy ( 0,0 )-campaprameters[2] ) *depth/campaprameters[0],
                ( xy ( 1,0 )-campaprameters[3] ) *depth/campaprameters[1],
                depth
                );
}
double find_depth_avg(cv::Mat image_dep, Vec8I markerConerABCD){
    Vec2I markerCenterXY = FindMarkerCenter(markerConerABCD);
    double CornerLength1 = sqrt((pow(markerConerABCD[0]-markerConerABCD[2],2))
                                +(pow(markerConerABCD[1]-markerConerABCD[3],2))); // in pixel
    double CornerLength2 = sqrt((pow(markerConerABCD[4]-markerConerABCD[6],2))
                                +(pow(markerConerABCD[5]-markerConerABCD[7],2))); // in pixel
    double MidDepthTotal = 0;
    int valid_count = 0;
    for(int i = 0; i < max(CornerLength1,CornerLength2); i++){
        for(int j = 0; j < max(CornerLength1,CornerLength2); j++){
            double MidDepthij = 0.001 * image_dep.at<ushort>(markerCenterXY[1]-max(CornerLength1,CornerLength2)+i,
                                                     markerCenterXY[0]-max(CornerLength1,CornerLength2)+j);
            if (MidDepthij > 0 ){
                MidDepthTotal += MidDepthij;
                valid_count++;
            }
        }
    }
    if (valid_count == 0){
        return(0);
    }else{
        return(MidDepthTotal/valid_count);
    }
}
Vec2I Constant_velocity_predictor(const Vec8I last_markerConer,const int Lostcounter){
    if (Lostcounter == 0){
        CVE_Corners.push_back(last_markerConer);
        if (CVE_Corners.size() > 5){ //calculate last five
            CVE_Corners.pop_front();
        }
        return(FindMarkerCenter(last_markerConer));
    }else{
        Vec2I XY = Vec2I(0,0);
        for(unsigned int i=0; i < (CVE_Corners.size()-1) ;i++){
            Vec2I diff_xy = FindMarkerCenter(CVE_Corners.at(i)) - FindMarkerCenter(CVE_Corners.at(i+1));
            XY += diff_xy;
            XY /= CVE_Corners.size();
            XY *= Lostcounter;
        }
        // return(FindMarkerCenter(CVE_Corners.back())+XY);
        return(FindMarkerCenter(last_markerConer));
    }
}
Vec3I HSVaverage(cv::Mat BGRmat){
    cv::Mat HSVmat;
    // int Haverage,Saverage,Vaverage;
    int Htotal = 0;
    int Stotal = 0;
    int Vtotal = 0;
    cvtColor(BGRmat, HSVmat, COLOR_BGR2HSV);
    for (int i=0; i<1280; i++){
        for (int j=0; j<720; j++){
            Htotal += HSVmat.at<Vec3b>(j,i)[0];
            Stotal += HSVmat.at<Vec3b>(j,i)[1];
            Vtotal += HSVmat.at<Vec3b>(j,i)[2];
        }
    }
    return(Vec3I(Htotal/921600,Stotal/921600,Vtotal/921600));
}
// void cameraconfig(cv::Mat BGRmat){ //Cam Auto config
//     if(HSVaverage(BGRmat)[2] < 50){
//         cout << "GO GO" << endl;
//     }
// }
void PNP3Dpoints(){  //Determine the LED pos in real world
    PNPPoints3D.push_back(cv::Point3f(159, 121, 0)); //green
    PNPPoints3D.push_back(cv::Point3f( 80,  42, 0)); //blue
    PNPPoints3D.push_back(cv::Point3f( 80, -42, 0)); //red  
    PNPPoints3D.push_back(cv::Point3f(159,-121, 0)); //red   gh034_small

    // PNPPoints3D.push_back(cv::Point3f( 0,69, 0)); //red   Square TestRig
    // PNPPoints3D.push_back(cv::Point3f(97, 0, 0)); //orange
    // PNPPoints3D.push_back(cv::Point3f( 0, 0, 0)); //green
    // PNPPoints3D.push_back(cv::Point3f(97,69, 0)); //blue
}
Vec6 LEDTvecRvec(Mat image_rgb){
    // Mat image_jpg = imread("./test.jpg");
    // imshow("image_jpg", image_jpg);
    Mat image_hsv,image_threshold;
    cvtColor(image_rgb, image_hsv, COLOR_BGR2HSV);
    Vec6 output;
    Vec3d PNPrvec, PNPtvec; 
    inRange(image_hsv, Scalar(0, 0, 150), Scalar(255, 255, 255), image_threshold);  
    dilate(image_threshold, image_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    erode(image_threshold, image_threshold, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
    // imshow("image_hsv", image_hsv);
    imshow("image_threshold", image_threshold);
    cv::waitKey(1);
    vector<vector<Point> > contours;
    findContours( image_threshold, contours, RETR_TREE, CHAIN_APPROX_SIMPLE );
    vector<Moments> mu(contours.size()); 
    for( size_t i = 0; i < mu.size(); i++ ){ //Find moments
        mu[i] = moments( contours[i] );
    }
    if (mu.size() < 4){ //Return 0 if see less than 4 LEDs
        output << 0,0,0,0,0,0;
        cout << "LED not enough, count: " << mu.size() << endl;
        return(output);
    }else if (mu.size() >20){
        cout << "Cam setting wrong, count: " << mu.size() << endl;
        output << 0,0,0,0,0,0;
        return(output);
    }
    if (mu.size() > 4){ //Remove small moments till 4
        vector<Moments> mu_temp = mu;
        mu.clear();
        vector<float> mu_size(mu_temp.size());
        for( size_t i = 0; i < mu_temp.size(); i++ ){
            mu_size[i] = mu_temp[i].m00;
        }
        sort(mu_size.rbegin(), mu_size.rend());
        for ( size_t i = 0; i < 4; i++ ){
            for ( size_t j = 0; j < mu_temp.size(); j++ ){
                if (mu_size[i] == mu_temp[j].m00){
                    mu.push_back(mu_temp[j]);
                }
            }
        }
    }
    vector<Point2f> mc(mu.size());
    vector<float> mc_x(mu.size());
    for( size_t i = 0; i < mu.size(); i++ ){ //Find centers
        mc[i] = Point2f( static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5)), 
                         static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5)) ); //add 1e-5 to avoid division by zero
        mc_x[i] = mc[i].x;
        // cout << "mc[" << i << "]=" << mc[i] << endl;
    }
    sort(mc_x.begin(), mc_x.end());
    vector<Point2f> mc_temp = mc;
    mc.clear();
    for( size_t i = 0; i < mu.size(); i++ ){
        for ( size_t j = 0; j < mu.size(); j++ ){
            if (mc_x[i] == mc_temp[j].x){
                mc[i] = mc_temp[j];
            }
        }
        // cout << "mc_sorted[" << i << "]=" << mc[i] << endl;
    }
    vector<int> mc_hue(mu.size()); 
    for (unsigned int i = 0; i < mu.size(); i++){ //Find the hue at each mcs
        mc_hue[i] = image_hsv.at<Vec3b>(mc[i])[0];
        // cout << "mc_hue[" << i << "]=" << mc_hue[i] << endl;
    }
    // Green 70~120
    PNPPoints2D.clear();
    for (unsigned int i = 0; i < mu.size(); i++){
        PNPPoints2D.push_back(mc[i]);
        // cout << "PNPPoints2D[" << i << "]=" << PNPPoints2D.end() << endl;
    }
    solvePnP(PNPPoints3D, PNPPoints2D, cameraMatrix, distCoeffs, PNPrvec, PNPtvec, false, SOLVEPNP_ITERATIVE);
    output << PNPtvec[0]*0.001,PNPtvec[1]*0.001,PNPtvec[2]*0.001,PNPrvec[0],PNPrvec[1],PNPrvec[2];
    // for (unsigned int i = 0; i < output.size(); i++){
    // cout << "output[" << i << "]=" << output[i] << endl;
    // }
    return(output);
}
#endif 