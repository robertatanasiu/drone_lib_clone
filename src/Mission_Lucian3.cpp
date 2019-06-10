#include "headers/gdpdrone.h"
#include "Dstar.h"

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
    float angle=atan((double)goaly/(double)goalx);
    
    for (int i=0;i<ceil(angle);i++) {
        for (int j=0;j<20;j++) {
            drone.Commands.move_Position_Local(0, 0, 0, 1, "BODY_OFFSET", j);
            ros::spinOnce();
            rate.sleep();
        }
    }

    float pitch= CalculatePitch(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w);

    drone.Commands.request_LandingAuto();
    return 0;
}
    








