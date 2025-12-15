#ifndef MISSION_NODE_H
#define MISSION_NODE_H
#include <ros/ros.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <move_base_msgs/MoveBaseActionResult.h>
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_listener.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>

#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>

#include <math.h>

using namespace std;
namespace mission{
class MissionNode{
public:    
    MissionNode(){}
    ~MissionNode(){}
    void initMissionNode()
    {
        ros::NodeHandle nh("~");
        //读取pid控制参数
        nh.param<float>("PID_Control_P", PID_Control_P, 0.75);
        //读取ABC，END四个点坐标
        nh.param<float>("A_x", A[0], 2.0);
        nh.param<float>("A_y", A[1], -0.5);
        nh.param<float>("B_x", B[0], 2.0);
        nh.param<float>("B_y", B[1], -1.5);
        nh.param<float>("C_x", C[0], 2.0);
        nh.param<float>("C_y", C[1], -2.5);
        nh.param<float>("E_x", E[0], 0.0);
        nh.param<float>("E_y", E[1], -2.8);
        //读取二维码目标对准最小偏移量
        nh.param<float>("Yaw_th", yaw_th, 0.1);
        //读取三个射击目标的ID
        nh.param<int>("Ar_0_id", ar_0_id, 0);
        nh.param<int>("Ar_1_id", ar_1_id, 1);
        nh.param<int>("Ar_2_id", ar_2_id, 2);
        //读取对准二维码目标过程中最大速度偏移
        nh.param<float>("Track_max_vel_x", track_max_vel_x, 0.5);  
        //读取对准二维码目标过程中最小速度偏移
        nh.param<float>("Track_thres_vel_x", track_thres_vel_x, 0.05);    
        //判断是否抵达导航地点的sub
        status_sub = nh.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 10, &MissionNode::status_cb, this);
        ar_sub = nh.subscribe<ar_track_alvar_msgs::AlvarMarkers>("/ar_pose_marker",10,&MissionNode::tag_cb,this);
        pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/abot/pose",1,&MissionNode::pos_cb,this);
        //发布速度控制的pub，对准二维码的时候用
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        //发布目标位置的pub，导航的时候用
        goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
        //发布射击命令的pub
        shoot_pub = nh.advertise<std_msgs::String>("/shoot", 10);
        
        printf_param();
        int check_flag;
        cout << "Please check the parameter and setting，1 for go on， else for quit: "<<endl;
        cin >> check_flag;
        //执行循环
        exec_timer = nh.createTimer(ros::Duration(0.05), &MissionNode::execCallback, this);

        pub_A = pub_B = pub_C = pub_E = false;
        shoot_A = shoot_B = shoot_C = false;
        marker_found = false;
        //reach_sign = false;
    }

private:
    ros::Subscriber status_sub, ar_sub, pos_sub, odom_sub;
    ros::Publisher cmd_pub, goal_pub, shoot_pub;
    ros::Timer exec_timer;
    //判断是否发布四个目标点标识位
    bool pub_A, pub_B, pub_C, pub_E;
    bool rever_A = false;
    //判断是否射击标志位
    bool shoot_A, shoot_B, shoot_C;
    //判断是否抵达标志位
    bool reach_sign;
    //判断是否发现目标标志位
    bool marker_found;
    //PID参数
    float PID_Control_P;
    //四个坐标点容器
    Eigen::Vector2f A , temp_A, temp_AA ,B ,C ,E;
    //目标容忍偏移量
    float yaw_th;
    //三次射击的目标id
    int ar_0_id, ar_1_id, ar_2_id;
    //导航状态量
    move_base_msgs::MoveBaseActionResult movebase_state;
    //当前目标id
    int current_target_id;
    //当前位置偏移与预期纠正偏转速度
    float offset_x, vel_x;
    //速度控制阈值
    float track_max_vel_x;
    float track_thres_vel_x;

    double pos_cur[2];
    double yaw_cur;

    int command_state;
    
    //射击函数逻辑
    void shootToar(int i){
        //射击点1
        if (i == 0)
        {
            current_target_id = ar_0_id;
            //std::cout<<"marker found: "<<marker_found<<std::endl;

            if (marker_found)
            {
                geometry_msgs::Twist Command_now;
                Command_now.linear.x = 0.0;
                Command_now.angular.z = vel_x;
                cmd_pub.publish(Command_now);
                /*********test code************************************************/
                //shoot_A = true;
                if(offset_x < yaw_th && offset_x > -yaw_th)
                {   
		    ros::Duration(0.9).sleep();
                    std::cout<<"shoot!!"<<std::endl;

                    std_msgs::String shoot_msg;
                    shoot_msg.data = string("1");
                    shoot_pub.publish(shoot_msg);
                    ros::Duration(0.5).sleep();
                    shoot_pub.publish(shoot_msg);
                    ros::Duration(0.5).sleep();
                    shoot_A = true;
                }
            }
            else
            {
                std::cout<<"marker not found: "<<i<<std::endl;
                if(abs(yaw_cur)>0.5)
                {
                    geometry_msgs::Twist Command_now;
                    Command_now.linear.x = 0.0;
                    Command_now.angular.z = -yaw_cur*0.5;
                    cmd_pub.publish(Command_now);
                }
            }
        }
        //射击点2
        if (i == 1)
        {
            current_target_id = ar_1_id;
            //std::cout<<"marker found: "<<marker_found<<std::endl;
            if (marker_found)
            {
                geometry_msgs::Twist Command_now;
                Command_now.linear.x = 0.0;
                Command_now.angular.z = vel_x;
                cmd_pub.publish(Command_now);
                if(offset_x < yaw_th && offset_x > -yaw_th)
                {
                    ros::Duration(0.9).sleep();
                    std_msgs::String shoot_msg;
                    shoot_msg.data = string("1");
                    shoot_pub.publish(shoot_msg);
                    ros::Duration(0.5).sleep();
                    shoot_pub.publish(shoot_msg);
                    ros::Duration(0.5).sleep();
                    shoot_B = true;
                }
            }
            else
            {
                std::cout<<"marker not found: "<<i<<std::endl;
                if(abs(yaw_cur)>0.5)
                {
                    geometry_msgs::Twist Command_now;
                    Command_now.linear.x = 0.0;
                    Command_now.angular.z = -yaw_cur*0.5;
                    cmd_pub.publish(Command_now);
                }

            }
        }
        //射击点3
        if (i == 2)
        {
            current_target_id = ar_2_id;
            std::cout<<"marker found: "<<marker_found<<std::endl;
            if (marker_found)
            {
                geometry_msgs::Twist Command_now;
                Command_now.linear.x = 0.0;
                Command_now.angular.z = vel_x;
                cmd_pub.publish(Command_now);
                if(offset_x < yaw_th && offset_x > -yaw_th)
                {
                    ros::Duration(0.8).sleep();
                    std_msgs::String shoot_msg;
                    shoot_msg.data = string("1");
                    shoot_pub.publish(shoot_msg);
                    ros::Duration(0.5).sleep();
                    shoot_pub.publish(shoot_msg);
                    ros::Duration(0.5).sleep();
                    shoot_C = true;
                }
            }
            else
            {
                std::cout<<"marker not found: "<<i<<std::endl;
                if(abs(yaw_cur)>0.5)
                {
                    geometry_msgs::Twist Command_now;
                    Command_now.linear.x = 0.0;
                    Command_now.angular.z = -yaw_cur*0.5;
                    cmd_pub.publish(Command_now);
                }
            }

        }
    }
    void pos_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        pos_cur[0] = msg->pose.position.x;
        pos_cur[1] = msg->pose.position.y;
        double quat[4];
        quat[0] = msg->pose.orientation.w;
        quat[1] = msg->pose.orientation.x;
        quat[2] = msg->pose.orientation.y;
        quat[3] = msg->pose.orientation.z;
        yaw_cur = atan2(2.0 * (quat[3] * quat[0] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
    }

    //判断是否抵达目标点函数
    void status_cb(const move_base_msgs::MoveBaseActionResult::ConstPtr &msg)
    {
        movebase_state = *msg;
        //if(movebase_state.status_list.size() == 0) return;
        if(movebase_state.status.status == 3)
        {
            reach_sign = true;
        }
        else reach_sign = false;
        //reach_sign = true;
        /*
        while(!marker_found){
            geometry_msgs::Twist Command_now;
            Command_now.linear.x = 0.0;
            Command_now.angular.z = 0.1;
            cmd_pub.publish(Command_now);

        }
        */
    }
    
    //二维码检测回调函数，回调当前识别到的二维码id与位置
    void tag_cb(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &msg)
    {
        ar_track_alvar_msgs::AlvarMarker marker;
        /*
        if (msg->markers.size() == 0)
        {
            marker_found = false;
        }*/
        int count = msg->markers.size();
        for(int i = 0; i<count; i++)
        {
            marker = msg->markers[i];
            if(marker.id == current_target_id)
            {
                marker_found = true;
                offset_x = marker.pose.pose.position.x;
                vel_x = -PID_Control_P * offset_x;
                satfunc(vel_x, track_max_vel_x, track_thres_vel_x);
            }

        }

    }
    
    //执行逻辑函数
    void execCallback(const ros::TimerEvent& e)
    {   
        //如果尚未发布点A，则发布点A
        if(!pub_A&&!shoot_A)
        {   
            cout << "Now move to Point A" <<endl;
            cout<<"A point: "<<A[0]<<" "<<A[1]<<endl;
            command_state = 0;
            geometry_msgs::PoseStamped A_goal;
            A_goal.header.frame_id = "map";
            A_goal.header.stamp = ros::Time::now();
            A_goal.pose.position.x = A[0];
            A_goal.pose.position.y = A[1];
            A_goal.pose.position.z = 0;
            A_goal.pose.orientation.x = 0;
            A_goal.pose.orientation.y = 0;
            A_goal.pose.orientation.z = 0;
            A_goal.pose.orientation.w = 0.97;
            goal_pub.publish(A_goal);
            reach_sign=false;
            pub_A = true;
        }
        //如果点A完成射击，未发布点B，则发布点B
        if(shoot_A&&!pub_B)
        {  
            reach_sign = false;
            cout << "Now move to Point B" <<endl;
            cout<<"B point: "<<B[0]<<" "<<B[1]<<endl;
            command_state = 2;

            geometry_msgs::PoseStamped B_goal;
            B_goal.header.frame_id = "map";
            B_goal.header.stamp = ros::Time::now();
            B_goal.pose.position.x = B[0];
            B_goal.pose.position.y = B[1];
            B_goal.pose.position.z = 0;
            B_goal.pose.orientation.x = 0;
            B_goal.pose.orientation.y = 0;
            B_goal.pose.orientation.z = 0;
            B_goal.pose.orientation.w = 0.97;
            goal_pub.publish(B_goal);

            pub_B = true;           
        }
        //如果点B完成射击，未发布点C，则发布点C
        if(shoot_B&&!pub_C)
        {  
            reach_sign = false;
            cout << "Now move to Point C" <<endl; 
            cout<<"C point: "<<C[0]<<" "<<C[1]<<endl;
            command_state = 4;
            geometry_msgs::PoseStamped C_goal;
            C_goal.header.frame_id = "map";
            C_goal.header.stamp = ros::Time::now();
            C_goal.pose.position.x = C[0];
            C_goal.pose.position.y = C[1];
            C_goal.pose.position.z = 0;
            C_goal.pose.orientation.x = 0;
            C_goal.pose.orientation.y = 0;
            C_goal.pose.orientation.z = 0;
            C_goal.pose.orientation.w = 0.97;
            goal_pub.publish(C_goal);
            //reach_sign=false;

            pub_C = true;           
        }
        //如果点C完成射击，未发布终点，则发布终点
        if(shoot_C&&!pub_E)
        {   
            reach_sign = false;
            command_state = 6;
            cout << "Now move to Point E" <<endl;
            cout<<"E point: "<<E[0]<<" "<<E[1]<<endl;
            geometry_msgs::PoseStamped E_goal;
            E_goal.header.frame_id = "map";
            E_goal.header.stamp = ros::Time::now();
            E_goal.pose.position.x = E[0];
            E_goal.pose.position.y = E[1];
            E_goal.pose.orientation.x = 0;
            E_goal.pose.orientation.y = 0;
            E_goal.pose.orientation.z = 0;
            E_goal.pose.orientation.w = 1.0;
            goal_pub.publish(E_goal);
            //reach_sign=false;

            pub_E = true;           
        }
        //如果没有到达目标点，什么都不做
        std::cout<<"reach_sign: "<<reach_sign<<std::endl;
        if (!reach_sign) return;
        //第一点射击
        if(!shoot_A && !pub_B ){
            cout << "Shoot to A" <<endl;
            command_state = 1;
            shootToar(0);
        }
        //第二点射击
        else if(pub_B && !shoot_B && !pub_C){
            cout << "Shoot to B" <<endl;
            command_state = 3;
            shootToar(1);
        }
        //第三点设计
        else if(pub_C && !shoot_C && !pub_E){
            cout << "Shoot to C" <<endl;
            command_state = 5;
            shootToar(2);
        }
    }

    //饱和函数
    float satfunc(float data, float Max, float Thres)
    {
        if (abs(data)<Thres)
        {
            return 0;
        }
        else if(abs(data)>Max)
        {
            return ( data > 0 ) ? Max : -Max;
        }
        else
        {
            return data;
        }
    }
    void printf_param()
    {
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>> Parameter <<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "A_x : "<< A[0] << endl;
        cout << "A_y : "<< A[1] << endl;
        cout << "B_x : "<< B[0] << endl;
        cout << "B_y : "<< B[1] << endl;
        cout << "C_x : "<< C[0] << endl;
        cout << "C_y : "<< C[1] << endl;
        cout << "E_x : "<< E[0] << endl;
        cout << "E_y : "<< E[1] << endl;

        cout << "Yaw_th : "<< yaw_th << endl;

        cout << "Ar_0_id : "<< ar_0_id << endl;
        cout << "Ar_1_id : "<< ar_1_id << endl;
        cout << "Ar_2_id : "<< ar_2_id << endl;

        cout << "track_max_vel_x : "<< track_max_vel_x << endl;
        cout << "track_thres_vel_x : "<< track_thres_vel_x << endl;
    }
    void printf_result()
    {
        cout.setf(ios::fixed);
        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>>>>>Vision State<<<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "curent_target_id: " <<  current_target_id <<endl;

        //cout << "pos_target: [X Z] : " << " " << offset_x  << " [m] " << offset_z <<" [m] "<<endl;

        cout <<">>>>>>>>>>>>>>>>>>>>>>>>>Control State<<<<<<<<<<<<<<<<<<<<<<<<" <<endl;
        cout << "State: " << command_state <<endl;
    }
};
}

#endif
