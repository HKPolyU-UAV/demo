#include <ros/ros.h>
#include <string>
#include <numeric>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include "utils/kinetic_math.hpp"
#include "utils/uav_mission.hpp"
#include "utils/trajectories.hpp"

static mavros_msgs::State           current_state;
static mavros_msgs::AttitudeTarget  UAV_AttitudeTarget;
static geometry_msgs::PoseStamped   UAV_pose_sub,UGV_pose_sub,UAV_pose_pub,UGV_pose_pub;
static geometry_msgs::TwistStamped  UGV_twist_sub,UAV_twist_sub;
static geometry_msgs::Twist         UAV_twist_pub;
static Vec7 UAV_lp,UGV_lp;
static Vec6 UAVinUGV;
static Vec3 UGVrpy,Traj_init_UGVrpy;

/* System */
bool System_init = false;
double System_initT;
ros::Time init_time,last_request; /* Use at System start up */
static Vec7 Zero7;
static Vec4 Zero4;
static int coutcounter;
/* PID Position controller */
static Vec4   Pos_setpoint,UGV_twist,UAV_twist;
static double PID_duration;
static double PID_InitTime;
static double Last_time = 0;
static Vec4 last_error,integral;
/* Fail safe (not ready)*/
bool   Failsafe_trigger  = false;
/* FSM */
Vec7 UAV_desP,UAV_takeoffP;
Vec7 TargetPos;
int    FSM_state = 0;
bool   FSM_init = false;
Vec7   FSM_init_pose;
double FSM_init_time;
double FSM_finish_time;
bool   FSM_finished= false;
int    Mission_state = 0;
int    Mission_stage = 0;
int    Current_Mission_stage = 0;
Vec8   Current_stage_mission;
bool   FSMinit       = false;
bool   Mission8init  = false;
double M8start_alt;
bool   pub_trajpose  = false;
bool   pub_pidtwist  = false;
bool   pub_trajtwist = false;
bool   Force_start   = false;
bool   ShutDown      = false;
bool   soft_ShutDown = false;

void failsafe(bool Failsafe_trigger){
    if(Failsafe_trigger){

    }
}
Vec6 uav2ugv(){
    Quaterniond UGVq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
    Matrix3d UGV_Rotation_world_mat  = UGVq.toRotationMatrix();
    Vec3 UGV_Translation_world(UGV_lp[0],UGV_lp[1],UGV_lp[2]);
    Vec3 tvecs(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
    Quaterniond UAVq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
    Vec3 rvecs = Q2rpy(UAVq);
    Vec3 Tvecs_ugvframe = UGV_Rotation_world_mat.inverse()*(tvecs-UGV_Translation_world);
    Vec3 Rvecs_ugvframe = UGV_Rotation_world_mat.inverse()*rvecs;
    Vec6 TvecsRvecs_ugvframe;
    TvecsRvecs_ugvframe << Tvecs_ugvframe[0],Tvecs_ugvframe[1],Tvecs_ugvframe[2],
                           Rvecs_ugvframe[0],Rvecs_ugvframe[1],Rvecs_ugvframe[2];
    return(TvecsRvecs_ugvframe);
}
Vec7 ugv_pred_land_pose(double est_duration){
    Vec7 EstimatedPose;
    Vec4 ugv_XYZyaw = Vec4(UGV_lp[0],UGV_lp[1],UGV_lp[2],UGVrpy[2]);
    Vec4 ugv_pred_XYZyaw;
    for(int i=0; i<4; i++){
        ugv_pred_XYZyaw[i] = ugv_XYZyaw[i]+UGV_twist[i]*est_duration;
    }
    Quaterniond ugvq;
    ugvq = rpy2Q(Vec3(0,0,ugv_pred_XYZyaw[3]));
    EstimatedPose << ugv_pred_XYZyaw[0],ugv_pred_XYZyaw[1],ugv_pred_XYZyaw[2],
                     ugvq.w(),ugvq.x(),ugvq.y(),ugvq.z();
    return(EstimatedPose);
}
void ugv_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UGV_pose_sub.pose.position.x = pose->pose.position.x;
    UGV_pose_sub.pose.position.y = pose->pose.position.y;
    UGV_pose_sub.pose.position.z = pose->pose.position.z;
    UGV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UGV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UGV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UGV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UGV_lp << UGV_pose_sub.pose.position.x,UGV_pose_sub.pose.position.y,UGV_pose_sub.pose.position.z,
              UGV_pose_sub.pose.orientation.w,UGV_pose_sub.pose.orientation.x,UGV_pose_sub.pose.orientation.y,UGV_pose_sub.pose.orientation.z;
    Quaterniond UGVq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
    UGVrpy = Q2rpy(UGVq);
}
void ugv_twist_sub(const geometry_msgs::TwistStamped::ConstPtr& twist){
    UGV_twist_sub.twist.linear.x = twist->twist.linear.x;
    UGV_twist_sub.twist.linear.y = twist->twist.linear.y;
    UGV_twist_sub.twist.linear.z = twist->twist.linear.z;
    UGV_twist_sub.twist.angular.x = twist->twist.angular.x;
    UGV_twist_sub.twist.angular.y = twist->twist.angular.y;
    UGV_twist_sub.twist.angular.z = twist->twist.angular.z;
    UGV_twist << UGV_twist_sub.twist.linear.x,UGV_twist_sub.twist.linear.y,
                 UGV_twist_sub.twist.linear.z,UGV_twist_sub.twist.angular.z;
}
Vec4 uav_poistion_controller_PID(Vec4 pose, Vec4 setpoint){
    Vec4 error,u_p,u_i,u_d,output,derivative;
    double iteration_time = ros::Time::now().toSec() - Last_time;
    // cout << "iteration_time: " << iteration_time << endl;
    Vec4 K_p(2,2,1,1);
    Vec4 K_i(0.05,0.05,0.05,0.05);
    Vec4 K_d(0,0,0,0);
    error = setpoint-pose;
    if (error[3]>=M_PI){error[3]-=2*M_PI;}
    if (error[3]<=-M_PI){error[3]+=2*M_PI;}
    for (int i=0; i<4; i++){ //i = x,y,z
        integral[i] += (error[i]*iteration_time);
        if(integral[i] >  1){ integral[i]=  1;}
        if(integral[i] < -1){ integral[i]=-1;}
        derivative[i] = (error[i] - last_error[i])/(iteration_time + 1e-10);
        u_p[i] = error[i]*K_p[i];        //P controller
        u_i[i] = integral[i]*K_i[i];     //I controller
        u_d[i] = derivative[i]*K_d[i];   //D controller
        output[i] = u_p[i]+u_i[i]+u_d[i];
        // cout << "-----------------------------------------------------------------------" << endl;
        // cout << "error[" << i << "]=" << error[i] << " last_error[" << i << "]=" << last_error[i] << endl;
        // cout << "iteration_time=" << iteration_time << " integral[" << i << "]=" << integral[i] << endl;
        // cout << "output[" << i << "]=" << output[i] << " u_p[" << i << "]=" << u_p[i] << " u_i[" << i << "]=" << u_i[i] << " u_d[" << i << "]=" << u_d[i] << endl;
    }
    for (int i=0; i<3; i++){
        if(output[i] >  1){ output[i]=  1;}
        if(output[i] < -1){ output[i]= -1;}
    }
    last_error = error;
    Last_time = ros::Time::now().toSec();
    return(output);
}
void uav_twist_sub(const geometry_msgs::TwistStamped::ConstPtr& twist){
    UAV_twist_sub.twist.linear.x = twist->twist.linear.x;
    UAV_twist_sub.twist.linear.y = twist->twist.linear.y;
    UAV_twist_sub.twist.linear.z = twist->twist.linear.z;
    UAV_twist_sub.twist.angular.x = twist->twist.angular.x;
    UAV_twist_sub.twist.angular.y = twist->twist.angular.y;
    UAV_twist_sub.twist.angular.z = twist->twist.angular.z;
    UAV_twist << UAV_twist_sub.twist.linear.x,UAV_twist_sub.twist.linear.y,
                 UAV_twist_sub.twist.linear.z,UAV_twist_sub.twist.angular.z;
}
void uav_state_sub(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
void uav_pose_sub(const geometry_msgs::PoseStamped::ConstPtr& pose){
    UAV_pose_sub.pose.position.x = pose->pose.position.x;
    UAV_pose_sub.pose.position.y = pose->pose.position.y;
    UAV_pose_sub.pose.position.z = pose->pose.position.z;
    UAV_pose_sub.pose.orientation.w = pose->pose.orientation.w;
    UAV_pose_sub.pose.orientation.x = pose->pose.orientation.x;
    UAV_pose_sub.pose.orientation.y = pose->pose.orientation.y;
    UAV_pose_sub.pose.orientation.z = pose->pose.orientation.z;
    UAV_lp << UAV_pose_sub.pose.position.x,UAV_pose_sub.pose.position.y,UAV_pose_sub.pose.position.z,
              UAV_pose_sub.pose.orientation.w,UAV_pose_sub.pose.orientation.x,UAV_pose_sub.pose.orientation.y,UAV_pose_sub.pose.orientation.z;
    UAVinUGV = uav2ugv();
}
void uav_pose_pub(Vec7 posepub){
    UAV_pose_pub.header.frame_id = "world";
    UAV_pose_pub.pose.position.x = posepub[0];
    UAV_pose_pub.pose.position.y = posepub[1];
    UAV_pose_pub.pose.position.z = posepub[2];
    UAV_pose_pub.pose.orientation.w = posepub[3];
    UAV_pose_pub.pose.orientation.x = posepub[4];
    UAV_pose_pub.pose.orientation.y = posepub[5];
    UAV_pose_pub.pose.orientation.z = posepub[6];
}
void uav_twist_pub(Vec4 vxyzaz){
    UAV_twist_pub.linear.x = vxyzaz(0);
    UAV_twist_pub.linear.y = vxyzaz(1);
    UAV_twist_pub.linear.z = vxyzaz(2);
    UAV_twist_pub.angular.z= vxyzaz(3);
}
void uav_pub(bool pub_trajpose, bool pub_pidtwist){
    if(pub_trajpose){
        Vec8 traj_pos_deque_front = trajectory_pos.front();
        while (ros::Time::now().toSec() - traj_pos_deque_front[0] > 0){
            trajectory_pos.pop_front();
            traj_pos_deque_front = trajectory_pos.front();
        }
        Vec7 uavposepub;
        uavposepub << traj_pos_deque_front[1],traj_pos_deque_front[2],traj_pos_deque_front[3],
                        traj_pos_deque_front[4],traj_pos_deque_front[5],traj_pos_deque_front[6],traj_pos_deque_front[7];
        uav_pose_pub(uavposepub);
        if (traj_pos_deque_front[0] > traj_pos_information[1] && FSM_state == 0){
            Mission_stage++;
            trajectory_pos.clear();
            uav_pose_pub(Zero7);
        }
    }
    if(pub_pidtwist){  //Use PID position controller
        Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
        Vec3 localrpy = Q2rpy(localq);
        Vec4 xyzyaw;
        xyzyaw << UAV_pose_sub.pose.position.x,UAV_pose_sub.pose.position.y,UAV_pose_sub.pose.position.z,localrpy[2];
        if(Mission_state == 7){  //Follow the UGV
            Vec7 UGV_pred_lp = ugv_pred_land_pose(1);
            Pos_setpoint << UGV_pred_lp[0],UGV_pred_lp[1],Current_stage_mission[3],UGVrpy[2];
        }
        if(Mission_state == 8){  //PID landing
            if(!Mission8init){
                Mission8init = true;
                M8start_alt = xyzyaw[2];
            }
            Vec7 UGV_pred_lp = ugv_pred_land_pose(0.5);
            Pos_setpoint << UGV_pred_lp[0],UGV_pred_lp[1],M8start_alt-=0.001,UGVrpy[2];
            if( sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) < 0.15 && sqrt(pow((UAV_lp[2]-UGV_lp[2]),2)) < 0.1 ){
                ShutDown = false;
                soft_ShutDown = true;
            }
        }
        if (PID_InitTime+PID_duration < ros::Time::now().toSec() && PID_duration != 0){ // EndMission
            Mission_stage++;
            uav_twist_pub(Zero4);
        }
        uav_twist_pub(uav_poistion_controller_PID(xyzyaw,Pos_setpoint));
    }
    if(pub_trajtwist){ //yaw use PID else use traj
        Quaterniond localq(UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6]);
        Vec3 localrpy = Q2rpy(localq);
        Vec4 xyzyaw;
        xyzyaw << UAV_lp[0],UAV_lp[1],UAV_lp[2],localrpy[2];
        Vec4 PID_output = uav_poistion_controller_PID(xyzyaw,Pos_setpoint);

        Vec8 traj_vel_deque_front = trajectory_vel.front();
        while (ros::Time::now().toSec() - traj_vel_deque_front[0] > 0){
            trajectory_vel.pop_front();
            traj_vel_deque_front = trajectory_vel.front();
        }
        Vec4 uavvelpub;
        uavvelpub << traj_vel_deque_front[1],traj_vel_deque_front[2],traj_vel_deque_front[3],PID_output[3];
        uav_twist_pub(uavvelpub);
    }
}
string armstatus(){
    if(current_state.armed){
        return("Armed   ");
    }else{
        return("Disarmed");
    }
}
string statestatus(){
    if (Mission_state == 0){
        return("Not Initialized(0)");
    }else if(Mission_state == 1){
        return("TakeOff(1)");
    }else if(Mission_state == 2){
        return("constantVtraj(2)");
    }else if(Mission_state == 3){
        return("AM_traj_pos(3)");
    }else if(Mission_state == 4){
        return("RTL(4)");
    }else if(Mission_state == 5){
        return("Landing(5)");
    }else if(Mission_state == 6){
        return("PID_step(6)");
    }else if(Mission_state == 7){
        return("PID(7)");
    }else if(Mission_state == 8){
        return("PID_landing(8)");
    }else{
        return("System error");
    }
}
void Finite_stage_mission(){  // Main FSM
    if (Mission_stage != Current_Mission_stage){// Generate trajectory while mission stage change
        Vec8 traj_pos;
        trajectory_pos.clear();
        Current_Mission_stage = Mission_stage;      //Update Current_Mission_stage
        Current_stage_mission = waypoints.at(Mission_stage-1);
        Quaterniond Targetq;
        Targetq = rpy2Q(Vec3(0,0,Current_stage_mission[4]));
        Mission_state = Current_stage_mission[0];
        if (Mission_state == 1){ //state = 1 take off with no heading change
            pub_trajpose = true;  pub_pidtwist = false;
            TargetPos << UAV_lp[0],UAV_lp[1],Current_stage_mission[3],UAV_lp[3],UAV_lp[4],UAV_lp[5],UAV_lp[6];
            constantVtraj(UAV_lp, TargetPos, 0.3, Current_stage_mission[6]);
        }
        if (Mission_state == 2){ //state = 2; constant velocity trajectory with desired heading.
            pub_trajpose = true;  pub_pidtwist = false;
            TargetPos << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 3){ //state = 3 AM_traj_pos;
            pub_trajpose = true;  pub_pidtwist = false;
            vector<Vector3d> WPs;
            WPs.clear();
            Vector3d StartP(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
            WPs.push_back(StartP);
            Vector3d WP(Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3]);
            WPs.push_back(WP);
            while(waypoints.at(Mission_stage)[0] == 3){
                cout << "Mission_stage: " << Mission_stage << endl;
                Mission_stage++;
                Current_Mission_stage = Mission_stage;
                Current_stage_mission = waypoints.at(Mission_stage-1);
                WP = Vector3d(Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3]);
                WPs.push_back(WP);
            }
            AM_traj_pos(WPs,UAV_lp,UAV_twist);
        }
        if (Mission_state == 4){ //state = 4; constant velocity RTL but with altitude
            pub_trajpose = true;  pub_pidtwist = false;
            TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],Current_stage_mission[3],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 5){ //state = 5; land.
            pub_trajpose = true;  pub_pidtwist = false;
            TargetPos << UAV_takeoffP[0],UAV_takeoffP[1],UAV_takeoffP[2],Targetq.w(),Targetq.x(),Targetq.y(),Targetq.z();
            constantVtraj(UAV_lp, TargetPos, Current_stage_mission[5], Current_stage_mission[6]);
        }
        if (Mission_state == 6){ //state = 6; PID constant pose position control
            pub_trajpose = false; pub_pidtwist = true;
            trajectory_pos.clear();
            Pos_setpoint << Current_stage_mission[1],Current_stage_mission[2],Current_stage_mission[3],Current_stage_mission[4];
            PID_duration = Current_stage_mission[7];
            PID_InitTime = ros::Time::now().toSec();
        }
        if (Mission_state == 7){ //state = 7; PID position control
            pub_trajpose = false; pub_pidtwist = true;
            trajectory_pos.clear();
            PID_duration = Current_stage_mission[7];
            PID_InitTime = ros::Time::now().toSec();
        }
        if (Mission_state == 8){ //state = 8; PID position control with landing
            pub_trajpose = false; pub_pidtwist = true;
            trajectory_pos.clear();
            PID_duration = Current_stage_mission[7];
            PID_InitTime = ros::Time::now().toSec();
        }
        if (Mission_state == 9){ //state = 9; Switch into Finite state machine
            pub_trajpose = false; pub_pidtwist = false;
            FSM_state = 2;
        }
        if (Mission_state < 6){
            if (Current_stage_mission[7] != 0){ //Wait after finish stage.
                traj_pos = trajectory_pos.back();
                int wpc = Current_stage_mission[7]/Trajectory_timestep;
                for (double i=0; i<wpc; i++){
                    traj_pos[0] += Trajectory_timestep;
                    trajectory_pos.push_back(traj_pos);
                }
            }
            
            /*For CPP deque safety. Default generate 10 second of hover*/
            int hovertime = 10;
            if(trajectory_pos.size()>0){
                traj_pos = trajectory_pos.back();
                for (int i=0; i<(hovertime/Trajectory_timestep); i++){
                    traj_pos[0] += Trajectory_timestep;
                    trajectory_pos.push_back(traj_pos);
                }
                traj_pos_information = Vec2(ros::Time::now().toSec(), traj_pos[0]-hovertime);
            }
        }
        /*For Debug section plot the whole trajectory*/
        // if(trajectory_pos.size()>0){
        //     for (unsigned int i = 0; i < trajectory_pos.size(); i++){
        //     Vec8 current_traj = trajectory_pos.at(i);
        //     cout << "dt: " << current_traj[0] << " x: " << current_traj[1] << " y: " << current_traj[2] << " z: " << current_traj[3] << endl;
        //     }
        // }
    }
}
void Finite_state_machine(){
    if(FSM_state==1){ //Free Flying (position hold while triggered)
        if(!FSM_init){
            FSM_init_pose = UAV_lp;
            FSM_init = true;
        }
        Vec7 position_hold_pose;
        Quaterniond PHq(FSM_init_pose[3],FSM_init_pose[4],FSM_init_pose[5],FSM_init_pose[6]);
        Vec3 PHrpy = Q2rpy(PHq);
        PHq = rpy2Q(Vec3(0,0,PHrpy[2]));
        position_hold_pose << UAV_lp[0],UAV_lp[1],UAV_lp[2],PHq.w(),PHq.x(),PHq.y(),PHq.z();
        uav_pose_pub(position_hold_pose);
        pub_trajpose = true; pub_pidtwist = false;
        // if(){ //wait until have ugv estimated pose
        //     FSM_init = false;
        //     FSM_state++;
        // }
    }
    if(FSM_state==2){ //Follow 1 (using GPS or vicon) stay at horizontal_dist, vertical_dist
        Vec7 FSM_2_pose;
        Quaterniond FSM2q(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
        Vec3 FSM2rpy = Q2rpy(FSM2q);
        double horizontal_dist = 1;
        double vertical_dist = 0.5;
        Vec2 uavxy = Vec2(UGV_lp[0]-horizontal_dist*cos(FSM2rpy[2]),UGV_lp[1]-horizontal_dist*sin(FSM2rpy[2]));
        FSM_2_pose << uavxy[0],uavxy[1],UGV_lp[2]+vertical_dist,UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6];
        uav_pose_pub(FSM_2_pose);
        pub_trajpose = false; pub_pidtwist = false;
        if(sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) < horizontal_dist+0.3 && !FSM_finished){
            // cout << "start count down" << endl;
            FSM_finish_time = ros::Time::now().toSec();
            FSM_finished = true;
        }
        if(sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) > horizontal_dist+0.3){
            FSM_finished = false;
        }
        if(FSM_finish_time - ros::Time::now().toSec() < -3 && FSM_finished){
            FSM_state++;
            FSM_finished = false;
        }
    }
    if(FSM_state==3){ //Follow 2 (using PID)
        Vec7 FSM_3_pose;
        Quaterniond FSM3q(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
        Vec3 FSM3rpy = Q2rpy(FSM3q);
        double horizontal_dist = 1;
        double vertical_dist = 0.5;
        Vec2 uavxy = Vec2(UGV_lp[0]-horizontal_dist*cos(FSM3rpy[2]),UGV_lp[1]-horizontal_dist*sin(FSM3rpy[2]));
        pub_trajpose = false; pub_pidtwist = true;
        Pos_setpoint << uavxy[0],uavxy[1],UGV_lp[2]+vertical_dist,UGVrpy[2];
        PID_duration = 0;
        if(sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) < horizontal_dist+0.3 && !FSM_finished){
            // cout << "start count down" << endl;
            FSM_finish_time = ros::Time::now().toSec();
            FSM_finished = true;
        }
        if(sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2)) > horizontal_dist+0.3){
            FSM_finished = false;
        }
        if(FSM_finish_time - ros::Time::now().toSec() < -3 && FSM_finished){
            FSM_state++;
            FSM_finished = false;
        }
    }
    // if(FSM_state==4){ //continuous vel_traj
    //     Vec7 FSM_4_pose;
    //     Quaterniond FSM4q(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
    //     Vec3 FSM4rpy = Q2rpy(FSM4q);
    //     double horizontal_dist = 1;
    //     double vertical_dist = 0.5;
    //     Vec2 uavxy = Vec2(UGV_lp[0]-horizontal_dist*cos(FSM4rpy[2]),UGV_lp[1]-horizontal_dist*sin(FSM4rpy[2]));
    //     pub_trajpose = false; pub_pidtwist = false; pub_trajtwist = true;
    //     Quaterniond UGVq(UGV_lp[3],UGV_lp[4],UGV_lp[5],UGV_lp[6]);
    //     Vec3 UGVrpy = Q2rpy(UGVq);
    //     Pos_setpoint << uavxy[0],uavxy[1],UGV_lp[2]+vertical_dist,UGVrpy[2];
    //     PID_duration = 0;
    //     vector<Vector3d> WPs;
    //     WPs.clear();
    //     Vector3d StartP(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
    //     WPs.push_back(StartP);
    //     Vector3d EndP(Pos_setpoint[0],Pos_setpoint[1],Pos_setpoint[2]);
    //     AM_traj_vel(WPs,UAV_lp, UAV_twist);
    // }
    if(FSM_state==4){ //Land trajectory
        Vec2 desxy;
        double Des_dist = 0.05;
        desxy = Vec2(UGV_lp[0]+Des_dist*cos(UGVrpy[2]),UGV_lp[1]+Des_dist*sin(UGVrpy[2]));
        if(!FSM_init){
            FSM_init = true;
            pub_trajpose = true;  pub_pidtwist = false;
            Traj_init_UGVrpy = UGVrpy;
            // double mid_dist = 1.5;
            // Vec2 midxy = Vec2(UGV_lp[0]-mid_dist*cos(UGVrpy[2]),UGV_lp[1]-mid_dist*sin(UGVrpy[2]));
            vector<Vector3d> WPs;
            WPs.clear();
            Vector3d StartP(UAV_lp[0],UAV_lp[1],UAV_lp[2]);
            WPs.push_back(StartP);
            Vector3d MidP(UAV_lp[0],UAV_lp[1],UAV_lp[2]-0.1);
            WPs.push_back(MidP);
            Vector3d EndP(desxy[0],desxy[1],UGV_lp[2]+0.1);
            WPs.push_back(EndP);
            AM_traj_pos(WPs,UAV_lp,UAV_twist);

            Vec7 UGV_pred_lp = ugv_pred_land_pose(AM_traj_pos_duration);
            WPs.clear();
            WPs.push_back(StartP);
            WPs.push_back(MidP);
            desxy = Vec2(UGV_pred_lp[0]+Des_dist*cos(UGVrpy[2]),UGV_pred_lp[1]+Des_dist*sin(UGVrpy[2]));
            EndP = Vector3d(desxy[0],desxy[1],UGV_lp[2]+0.1);
            WPs.push_back(EndP);
            AM_traj_pos(WPs,UAV_lp,UAV_twist);

            // FSM_init_time = ros::Time::now().toSec();

            /*For CPP deque safety. Default generate 10 second of hover*/
            int hovertime = 10;
            if(trajectory_pos.size()>0){
                Vec8 traj_pos = trajectory_pos.back();
                for (int i=0; i<(hovertime/Trajectory_timestep); i++){
                    traj_pos[0] += Trajectory_timestep;
                    trajectory_pos.push_back(traj_pos);
                }
                traj_pos_information = Vec2(ros::Time::now().toSec(), traj_pos[0]-hovertime);
            }
        }
        double Traj_leftT = traj_pos_information[1] - ros::Time::now().toSec();
        double Dist_horizontal = sqrt(pow((UAV_lp[0]-UGV_lp[0]),2)+pow((UAV_lp[1]-UGV_lp[1]),2));
        if(abs(Traj_init_UGVrpy[2]-UGVrpy[2])>0.2){
            cout << "---------------------------------------------------" << endl
                 << "------------------UGV not stable-------------------" << endl
                 << "---------------------------------------------------" << endl;
            FSM_state--;
            FSM_init = false;
        }
        if(Traj_leftT < -2){
            cout << "---------------------------------------------------" << endl
                 << "---------------Landing Traj Timeout----------------" << endl
                 << "---------------------------------------------------" << endl;
            FSM_state--;
            FSM_init = false;
        }
        if(UAVinUGV[2] < 0.05 && Dist_horizontal < 0.5){
            cout << "---------------------------------------------------" << endl
                 << "-----Vertical distance not enough to approach------" << endl
                 << "---------------------------------------------------" << endl;
            FSM_state--;
            FSM_init = false;
        }
        if(UAVinUGV[0] > 0.15 ){
            cout << "---------------------------------------------------" << endl
                 << "--------------UAV overshoots UGV-------------------" << endl
                 << "---------------------------------------------------" << endl;
            FSM_state--;
            FSM_init = false;
        }
        if(Traj_leftT <  1 && UAVinUGV[0] > -0.05 && abs(UAVinUGV[1]) < 0.1 && UAVinUGV[2] < 0.15 ){ 
            ShutDown = false;
            soft_ShutDown = true;
            FSM_state++;
        }
    }
    if(FSM_state==6){ //Back up

    }
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "FSM");
    ros::NodeHandle nh;
    ros::Subscriber ugvpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/gh034_car/pose", 5, ugv_pose_sub);
    ros::Subscriber ugvtwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/vrpn_client_node/gh034_car/twist", 5, ugv_twist_sub);
    ros::Subscriber uavpose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 1, uav_pose_sub);
    ros::Subscriber uavtwist_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity_body", 5, uav_twist_sub);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, uav_state_sub);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::Publisher uav_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 5); 
    ros::Publisher uav_vel_pub = nh.advertise<geometry_msgs::Twist>("/mavros/setpoint_velocity/cmd_vel_unstamped", 5);
    ros::Publisher uav_AttitudeTarget = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude",5);
    mavros_msgs::SetMode offb_set_mode,posctl_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    posctl_set_mode.request.custom_mode = "POSCTL";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    nh.getParam("/FSM_node/Force_start", Force_start);
    Zero4 << 0,0,0,0;
    Zero7 << 0,0,0,0,0,0,0;
    UAV_AttitudeTarget.thrust = 0.3;
    ros::Rate loop_rate(100); /* ROS system Hz */

    while(ros::ok()){
        /* System initailize ***************************************************/
        if (!System_init){
            System_initT = ros::Time::now().toSec();
            init_time = ros::Time::now();
            waypoints = Mission_generator(); //Generate stages
            cout << " System Initialized" << " Force_start: " << Force_start << endl;
            /* Waypoints before starting */ 
            uav_pose_pub(Zero7);
            for(int i = 10; ros::ok() && i > 0; --i){
                uav_pos_pub.publish(UAV_pose_pub);
                ros::spinOnce();
                loop_rate.sleep();
            }
            System_init = true;
        }
        /* offboard and arm ****************************************************/
        if((ros::Time::now() - init_time < ros::Duration(20.0))){
            if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(0.5))){
                //Set Offboard trigger duration here
                uav_pos_pub.publish(UAV_pose_pub);
                if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                    ROS_INFO("Offboard enabled");
                }
                last_request = ros::Time::now();
            }else{
                if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(0.5))){
                    if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                last_request = ros::Time::now();
                }
            }
        }
        /* FSM initailize */
        if (current_state.mode == "OFFBOARD" && current_state.armed && !FSMinit){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "Mission stage = 1 Mission start!" <<endl;
        }
        if (Force_start){
            FSMinit = true;
            Mission_stage = 1;
            UAV_takeoffP = UAV_lp;
            cout << "UAV_takeoff_Position: " << UAV_takeoffP[0] << " " << UAV_takeoffP[1] << " " << UAV_takeoffP[2] << endl;
            cout << "------------------Dangerous!------------------" << endl;
            cout << "Mission stage = 1 Mission start!" << endl;
            Force_start = false;
        }
        /* FSM *****************************************************************/
        Finite_stage_mission();
        failsafe(Failsafe_trigger);
        Finite_state_machine();
        uav_pub(pub_trajpose,pub_pidtwist);
        if(ShutDown){ // UAV shut down
            cout << "Warning Vehicle Shut Down" << endl;
            pub_pidtwist = false;
            pub_trajpose = false;
            UAV_AttitudeTarget.thrust = 0;
            uav_AttitudeTarget.publish(UAV_AttitudeTarget);
        }
        if(soft_ShutDown){
            cout << "Vehicle Soft Shut Down" << endl;
            pub_pidtwist = false;
            pub_trajpose = false;
            UAV_AttitudeTarget.thrust -= 0.005;
            cout << "thrust: " << UAV_AttitudeTarget.thrust << endl;
            uav_AttitudeTarget.publish(UAV_AttitudeTarget);
        }
        if(pub_pidtwist){uav_vel_pub.publish(UAV_twist_pub);}
        if(pub_trajpose){uav_pos_pub.publish(UAV_pose_pub);}
        if(FSM_state == 2){uav_pos_pub.publish(UAV_pose_pub);}
        /*Mission information cout**********************************************/
        if(coutcounter > 30 && FSMinit && !ShutDown && !soft_ShutDown){ //reduce cout rate
            if (FSM_state == 0){
                cout << "Status: "<< armstatus() << "    Mode: " << current_state.mode <<endl;
                cout << "Mission_Stage: " << Mission_stage << "    Mission_total_stage: " << waypoints.size() << endl;
                cout << "Mission_State: " << statestatus() << endl;
            }else{
                cout << "FSM State: "<< FSM_state << endl;
            }

            cout << "vicon__pos_x: " << UAV_lp[0] << " y: " << UAV_lp[1] << " z: "<< UAV_lp[2] << endl;
            
            if(pub_trajpose){
                cout << "desiredpos___x: " << UAV_pose_pub.pose.position.x << " y: " << UAV_pose_pub.pose.position.y << " z: "<< UAV_pose_pub.pose.position.z << endl;
                cout << "Traj countdown: " << traj_pos_information[1] - ros::Time::now().toSec() << endl;
            }
            if(pub_pidtwist){
                cout << "des__twist_x: " << UAV_twist_pub.linear.x << " y: " << UAV_twist_pub.linear.y << " z: "<< UAV_twist_pub.linear.z << " az: " << UAV_twist_pub.angular.z << endl;
                if (FSM_state == 0){
                    cout << "PID  countdown: " << PID_InitTime+PID_duration - ros::Time::now().toSec() << endl;
                }
            }
            cout << "CAr____pos_x: " << UGV_pose_sub.pose.position.x << " y: " << UGV_pose_sub.pose.position.y << " z: " << UGV_pose_sub.pose.position.z << endl;
            cout << "UAV_in_UGVframe_x: " << UAVinUGV[0] << " y: " << UAVinUGV[1] << " z: "<< UAVinUGV[2] << endl;
            cout << "Failsafe_Syste: " << Failsafe_trigger << endl;
            cout << "---------------------------------------------------" << endl;
            coutcounter = 0;
        }else{coutcounter++;}
        ros::spinOnce();
        loop_rate.sleep();
    }
}