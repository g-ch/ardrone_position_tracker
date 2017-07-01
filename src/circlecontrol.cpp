#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Empty.h"
#include <iostream>

#include "ardrone_autonomy/Navdata.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include "Eigen/Dense"

using namespace Eigen;
using namespace std;

std_msgs::Empty order;
geometry_msgs::Twist cmd;

//const float vel_kp=0.12,vel_ki=0.003,vel_kd=0.008;
const float pos_kp=0.2,pos_ki=0.03,pos_kd=0.01;
#define LOOP_RATE 10

struct raw_state
{
    Vector3f  pos_b;
    Vector3f pos_f;
    Vector3f vel_b;
    Vector3f vel_f;
};
struct output
{
    Vector3f vel_sp;
};
struct control
{
    Vector3f vel_sp;
    Vector3f pos_sp;
};

raw_state raw_stat;
output out;
control contro;
bool record = true;
float start_x=0.0;
float start_y=0.0;

void odometryCallback(const nav_msgs::Odometry &msg){
    raw_stat.pos_b(0)= msg.pose.pose.position.y;//unit: m ?????? what the hell?????
    raw_stat.pos_b(1)= -msg.pose.pose.position.x;

    if(record)
    {
        start_x = raw_stat.pos_b(0);
        start_y = raw_stat.pos_b(1);
        record = false;
    }

    raw_stat.pos_b(0) = raw_stat.pos_b(0) - start_x;
    raw_stat.pos_b(1) = raw_stat.pos_b(1) - start_y;

    //raw_stat.vel_b(0)= msg.twist.twist.linear.x;//unit:m/s
    //raw_stat.vel_b(1)= msg.twist.twist.linear.y;
}

/*void navCallback(const ardrone_autonomy::Navdata &msg)
{
    static bool start=true;
    static float last_time = 0;
    if(start){
        start = false;
        last_time = msg.tm;
    }

    float dt = (msg.tm - last_time)/1000000.0;
    last_time = msg.tm;
//    raw_stat.vel_b(0) = msg.vx/1000;
//    raw_stat.vel_b(1) = msg.vy/1000;
//    raw_stat.pos_b = raw_stat.vel_b * dt;
}*/

void pid_pos(Vector3f& actual,Vector3f& set,Vector3f& control)
{
    static Vector3f err_last;
    static Vector3f err_int;
    static bool new_start = true;
    Vector3f err_pos;
    Vector3f err_d;
    err_pos = set - actual;
    if (new_start)
    {
        err_last = err_pos;
        err_int = Vector3f::Zero();
        new_start = false;
    }
    err_d = (err_pos - err_last)*LOOP_RATE;
    control = err_pos * pos_kp + err_d * pos_kd + err_int * pos_ki;
    err_last = err_pos;
    err_int += err_pos / LOOP_RATE;
}
/*void pid_vel(Vector3f& actual,Vector3f& set,Vector3f& control)
{
    static Vector3f err_lastt;
    static Vector3f err_intt;
    static bool new_startt = true;
    Vector3f err_vel;
    Vector3f err_d;
    err_vel = set - actual;
    if (new_startt)
    {
        err_lastt = err_vel;
        err_intt = Vector3f::Zero();
        new_startt = false;
    }
    err_d = (err_vel - err_lastt)*LOOP_RATE;
    control = err_vel * vel_kp + err_d * vel_kd + err_intt * vel_ki;
    err_lastt = err_vel;
    err_intt += err_vel / LOOP_RATE;
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circlecontrol");
    ros::NodeHandle n;
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_ref", 1);
    ros::Publisher takeoff_pub = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);
    ros::Publisher land_pub = n.advertise<std_msgs::Empty>("/ardrone/land", 1);
    ros::Subscriber sub = n.subscribe("/ardrone/odometry",1,odometryCallback);
   //ros::Subscriber nav_sub = n.subscribe("/ardrone/navdata", 1, navCallback);
    ros::Rate loop_rate(LOOP_RATE);



    static bool new_start=true;

    raw_stat.pos_b = Vector3f::Zero();
    raw_stat.pos_f = Vector3f::Zero();
    raw_stat.vel_b = Vector3f::Zero();
    raw_stat.vel_f = Vector3f::Zero();
    out.vel_sp = Vector3f::Zero();;
    contro.pos_sp = Vector3f::Zero();
    contro.vel_sp = Vector3f::Zero();

    int index = 0;
    while(ros::ok())
    {
        float r=0.8;
        float w=0.1;

        index = index+1;
        ROS_INFO("index:%d",index);
            /*if (index<=70)
            {
                raw_stat.vel_f(0) = index/50.0*0.2;
                raw_stat.pos_f(0) = index/50.0*0.2 * index /2.0/ LOOP_RATE;
            }*/

            if(index <= 624)
            {
                raw_stat.vel_f(0) = w*r*cos(w*index/LOOP_RATE);
                raw_stat.vel_f(1) = w*r*sin(w*index/LOOP_RATE);
                raw_stat.pos_f(0) = r*sin(w*index/LOOP_RATE);
                raw_stat.pos_f(1) = r-r*cos(w*index/LOOP_RATE);
            }
            else if( index > 624)
            {
                raw_stat.vel_f(0) = 0;
                raw_stat.vel_f(1) = 0;
            }
           /* else if(index > 100 && index < 150){
                raw_stat.vel_f(0) = 0.2 - (index-100)/50.0*0.2;
                    raw_stat.pos_f(0) = 2.0 - (150-index)/50.0*0.2 * (150-index) /2.0/ LOOP_RATE;
            }
            else{
                //raw_stat.vel_f(0) = 0.0;
                //raw_stat.pos_f(0) = 2.0;
                phase =1;
            }
            break;
        case 1:
            index2 ++;
            if (index2<=50)
                {
                    raw_stat.vel_f(1) = index2/50.0*0.2;
                    raw_stat.pos_f(1) = index2/50.0*0.2 * index2 /2.0/ LOOP_RATE;
                }else if((index2>50)&&(index2<=100)){
                raw_stat.vel_f(1) = 0.2;
                    raw_stat.pos_f(1) = 0.5+0.2*(index2/LOOP_RATE-5.0);
            }
            else if(index2 > 100 && index2 < 150){
                raw_stat.vel_f(1) = 0.2 - (index2-100)/50.0*0.2;
                    raw_stat.pos_f(1) = 2.0 - (150-index2)/50.0*0.2 * (150-index2) /2.0/ LOOP_RATE;
            }
            else{
                //raw_stat.vel_f(1) = 0.0;
                 //   raw_stat.pos_f(1) = 2.0;
                phase = 2;
            }
            break;

        case 2:
            index3++;
            if (index3<=50)
                {
                    raw_stat.vel_f(1) = -index3/50.0*0.2;
                    raw_stat.pos_f(1) = 2.0 - index3/50.0*0.2 * index3 /2.0/ LOOP_RATE;
                }else if((index3>50)&&(index3<=100)){
                raw_stat.vel_f(1) = -0.2;
                    raw_stat.pos_f(1) = 2- (0.5+0.2*(index3/LOOP_RATE-5.0));
            }
            else if(index3 > 100 && index3 < 150){
                raw_stat.vel_f(1) = -(0.2 - (index3-100)/50.0*0.2);
                    raw_stat.pos_f(1) = (150-index3)/50.0*0.2 * (150-index2) /2.0/ LOOP_RATE;
            }
            else{
                //raw_stat.vel_f(1) = 0.0;
                  //  raw_stat.pos_f(1) = 0.0;
                phase = 3;

            }
            break;
        case 3:
            index4++;
            if (index4<=50)
                {
                    raw_stat.vel_f(0) = -index4/50.0*0.2;
                    raw_stat.pos_f(0) = 2.0 - index4/50.0*0.2 * index4 /2.0/ LOOP_RATE;
                }else if((index4>50)&&(index4<=100)){
                raw_stat.vel_f(0) = -0.2;
                    raw_stat.pos_f(0) = 2- (0.5+0.2*(index4/LOOP_RATE-5.0));
            }
            else if(index4 > 100 && index4 < 150){
                raw_stat.vel_f(0) = -(0.2 - (index4-100)/50.0*0.2);
                    raw_stat.pos_f(0) = (150-index4)/50.0*0.2 * (150-index2) /2.0/ LOOP_RATE;
            }
            else{
                raw_stat.vel_f(0) = 0.0;
                    raw_stat.pos_f(0) = 0.0;
            }
            break;
        }*/









        
        //raw_stat.vel_f(0) = raw_stat.vel_f(0)  * 10;
    ROS_INFO("vset:x=%f y=%f",raw_stat.vel_f(0),raw_stat.vel_f(1));
    ROS_INFO("pset:x=%f y=%f",raw_stat.pos_f(0),raw_stat.pos_f(1));
        //raw_stat.pos_f(0) = raw_stat.pos_f(0)  * 10;

	
    /*if(new_start)
	{
      if(raw_stat.pos_b(1)!=0)
	{
          raw_stat.pos_f = raw_stat.pos_b;
          new_start = false;
          pid_pos(raw_stat.pos_b,raw_stat.pos_f,contro.pos_sp);
          out.vel_sp = raw_stat.vel_f+contro.pos_sp;
          //pid_vel(raw_stat.vel_b,out.vel_sp,contro.vel_sp);
    }
    }*/
    //else
    {
        pid_pos(raw_stat.pos_b,raw_stat.pos_f,contro.pos_sp);
        out.vel_sp = raw_stat.vel_f+contro.pos_sp;
        ROS_INFO("vel_sp:x=%f y=%f",out.vel_sp(0),out.vel_sp(1));
        ROS_INFO("position:x=%f y=%f",raw_stat.pos_b(0),raw_stat.pos_b(1));
        //pid_vel(raw_stat.vel_b,out.vel_sp,contro.vel_sp);
    }

        
        cmd.linear.x = out.vel_sp(0);
        cmd.linear.y = out.vel_sp(1);
        cmd.linear.z = 0.0;
        cmd.angular.x = 0.0;
        cmd.angular.y = 0.0;
        cmd.angular.z = 0.0;
        //cmd.linear.x = 0.02;
        //cmd.linear.y = 0.005;

        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
