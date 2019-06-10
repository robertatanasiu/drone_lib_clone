#include "headers/gdpdrone.h"
#include "Dstar.h"
#include "iostream"

double CalculatePitch(float x, float y, float z, float w) ///< takes quaternions and returns pitch in degrees
{
   // pitch (y-axis rotation)
   float pitch;
   float sinp = +2.0 * (w * y - z * x);
   if (fabs(sinp) >= 1)
       pitch = copysign(M_PI / 2, sinp);
   else
       pitch = asin(sinp);    return pitch * 180 / M_PI;
}

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "B00lake_node");

    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    Dstar *dstar = new Dstar();
    list<state> mypath;

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 1.0;
    int time_takeoff = 100;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    int goalx=500;
    int goaly=100;
    int resolution=5;
    goalx/=resolution;
    goaly/=resolution;
    float angle=atan((double)goaly/(double)goalx);
    angle*=180/M_PI;
    ROS_INFO("angle is [%f]", ceil(angle));
    dstar->init(0,0,goalx,goaly);
    for (int i=0;i<ceil(angle);i++) {
        for (int j=0;j<20;j++) {
            drone.Commands.move_Position_Local(0, 0, 0, 1, "BODY_OFFSET", j);
            ros::spinOnce();
            rate.sleep();
        }
        if (drone.Data.lidar.ranges[1]!=INFINITY) {
            //ROS_INFO("dr [%f]",drone.Data.lidar.ranges[1]);
            
            int x1,y1;
            y1=(int)floor(drone.Data.lidar.ranges[1]*sin(i*M_PI/180)/0.2);
            x1=(int)floor(drone.Data.lidar.ranges[1]*cos(i*M_PI/180)/0.2);
            dstar->updateCell(x1,y1,-1);
            ROS_INFO("x [%i]",x1);
            ROS_INFO("y [%i]",y1);
        }

    }
    
    for (int j=0;j<20;j++) {
        drone.Commands.move_Position_Local(0, 0, 0, -ceil(angle), "BODY_OFFSET", j);
        ros::spinOnce();
        rate.sleep();
    }

    dstar->replan();               // plan a path
	mypath = dstar->getPath();

    int stx=0,sty=0;
    //while (stx!=goalx || sty!=goaly) {
    for (auto &v :mypath)
    {
        dstar->updateStart(v.x,v.y);      // move start to new path point
		for (int j=0;j<20;j++) {
        drone.Commands.move_Position_Local(-v.y+sty, v.x-stx, 0, 0, "BODY_OFFSET", j);
        ros::spinOnce();
        rate.sleep();
        }//Move the actual dron
		stx=v.x;
        sty=v.y;
	ROS_INFO("vx [%i]",v.x);
    ROS_INFO("vy [%i]",v.y);
    }
    //}


    float pitch= CalculatePitch(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w);

    drone.Commands.request_LandingAuto();
    return 0;
}