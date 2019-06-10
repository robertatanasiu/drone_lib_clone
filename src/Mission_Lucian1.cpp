
#include "headers/gdpdrone.h"
//#include <math.h>
#include "Dstar.h"
//#include <iostream>

void qtToEuler(float x, float y, float z, float w, float angles[3]);
int approx(float a);

struct finalposition
{
    float x, y, z;
};

finalposition finalPosition;
float Orientation[3];

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "_node");

    Dstar *dstar = new Dstar();
    list<state> mypath;

    // Create drone object, this sets everything up
    GDPdrone drone;

    // Set the rate. Default working frequency is 25 Hz
    float loop_rate = 10.0;
    ros::Rate rate = ros::Rate(loop_rate);

    // Initialise and Arm
    drone.Commands.await_Connection();
    drone.Commands.set_Offboard();
    drone.Commands.set_Armed();

    // MISSION STARTS HERE:
    // Request takeoff at 1m altitude. At 25Hz = 10 seconds
    float altitude = 1.0;
    int time_takeoff = 100;
    drone.Commands.request_Takeoff(altitude, time_takeoff);

    // Set final position
    /*finalPosition.x = 0;
    finalPosition.y = 10;
    finalPosition.z = 0;

    // Turn drone to face the goal at current position
    float rotation = atan2(finalPosition.x, finalPosition.y) * 180 / M_PI;
    int count = 0;
    for (count = 0; count < 50; count++)
    {
        drone.Commands.move_Position_Local(0, 0, 0, rotation, "BODY_OFFSET", count);
        ros::spinOnce();
        rate.sleep();
    }

    for (count = 0; count < 50; count++)
    {
    drone.Commands.move_Position_Local(0, 10, 0, 0, "BODY_OFFSET", count);
    ros::spinOnce();
    rate.sleep();
    }*/

    // Initialize final position
    //int resolution = 5;

    //int distance = 10;  //gps distance in m between goal and start
    //int resolution = 5; //Resolution is 1/5 m;

    /*dstar->init(0, 0, distance * resolution, 0);

    for (int i = -distance * resolution; i <= distance * resolution; i++)
        dstar->updateCell(i, -1, -1); //set the ground to be non-traversable*/

    float pitch;
    float xobst, yobst;

    while (drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY)
    {
        ROS_INFO("Intru");
        drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
        qtToEuler(drone.Data.imu.orientation.x, drone.Data.imu.orientation.y, drone.Data.imu.orientation.z, drone.Data.imu.orientation.w, Orientation);
        pitch = Orientation[1];
        xobst = drone.Data.lidar.ranges[1];
    }
    // Get x-distance to the obstacle
    xobst = xobst * cos(pitch * M_PI / 180);
    int xObst = approx(xobst);

    // Get distance to edge of obstacle
    yobst = xobst * ((45 - pitch) * M_PI / 180);
    int yObst = approx(yobst);

    ROS_INFO("xObst [%i]", xObst);
    ROS_INFO("yObst [%i]", yObst);

    int Count = 0;
    float current_x, current_y;

    while (Count != 50)
    {
        drone.Commands.move_Position_Local(0, 0, 0, 0, "BODY_OFFSET", Count);
        Count++;
        ROS_INFO("Stau");
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Current height [%f]", drone.Data.local_pose.pose.position.z);

    Count = 0;

    int distance = 400; //gps distance in m between goal and where I am
    int resolution = 2; //Resolution is 2/10 m;
    int Altitude = (int)altitude;
    dstar->init(0, Altitude, distance / resolution, 0); // set start to (0,0) and goal to (10,0)
    for (int i = -distance / resolution; i <= distance / resolution; i++)
        dstar->updateCell(i, -1, -1); //set the ground to be non-traversable

    for (int i = 0; i <= yObst / resolution; i++)
        dstar->updateCell(xobst * resolution, i, -1);

    float puta1 = drone.Data.local_pose.pose.position.y;
    float puta2 = drone.Data.local_pose.pose.position.z;
    int PULA1 = approx(puta1);
    int PULA2 = approx(puta2);
    dstar->updateStart(PULA1 / 2, PULA2 / 2);
    int xst = drone.Data.local_pose.pose.position.z, yst = drone.Data.local_pose.pose.position.z;

    std::vector<int> vx_relative = {0};
    std::vector<int> vy_relative = {0};

    //current_x = drone.Data.local_pose.pose.position.y;
    //current_y = drone.Data.local_pose.pose.position.z;
    while (!(xst == distance / resolution && yst == 0))
    {
        ROS_INFO("Primul while.");
        dstar->replan();           // plan a path
        mypath = dstar->getPath(); // retrieve path

        for (auto &v : mypath)
        {
            dstar->updateStart(v.x, v.y); // move start to new path point
            ROS_INFO("v.x [%i]", v.x);
            ROS_INFO("v.y [%i]", v.y);
            Count = 0;
            current_x = drone.Data.local_pose.pose.position.x;
            current_y = drone.Data.local_pose.pose.position.y;
            vx_relative.push_back(v.x);
            vy_relative.push_back(v.y);

            /*while (drone.Data.local_pose.pose.position.x < current_x + v.x * 0.2 && drone.Data.local_pose.pose.position.y < current_y + v.y * 0.2)
            {
                drone.Commands.move_Velocity_Local(0, 0.5, 0.5, 0, "BODY_OFFSET");
                ros::spinOnce();
                rate.sleep();
            }*/
            while (Count != 5)
            {
                //ROS_INFO("2 while.");
                drone.Commands.move_Position_Local(0, (vx_relative[vx_relative.size() - 1] - vx_relative[vx_relative.size() - 2]) * 0.2, (vy_relative[vy_relative.size() - 1] - vy_relative[vy_relative.size() - 2]) * 0.2, 0, "BODY_OFFSET", Count);
                Count++;
                ros::spinOnce();
                rate.sleep();
            }
            xst = v.x;
            yst = v.y;
            //std::cout <<v.x <<" "<<v.y<<'\n';
            if (v.x == xObst / resolution && v.y == yObst / resolution + 1)
            {
                //Check for new obstacles
                xObst += 3;
                yObst += 2; //We update the x and y of the new obstacles relative to our current position
                for (int i = 0; i <= yObst / resolution; i++)
                    dstar->updateCell(xObst / resolution, i, -1); // set cells (xobst,i) to be non traversable
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
        ros::spinOnce();
        rate.sleep();
    }

    // Land and disarm
    drone.Commands.request_LandingAuto();

    return 0;
}

void qtToEuler(float x, float y, float z, float w, float angles[3])
{
    float sinr_cosp = +2.0 * (w * x * y * z);
    float cosr_cosp = +1.0 - 2.0 * (x * x + y + y);
    float roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float pitch;
    float sinp = +2.0 * (w * y - z * x);
    if (fabs(sinp) >= 1)
        pitch = copysign(M_PI / 2, sinp);
    else
        pitch = asin(sinp);

    // yaw (z-axis rotation)
    float siny_cosp = +2.0 * (w * z + x * y);
    float cosy_cosp = +1.0 - 2.0 * (y * y + z * z);
    float yaw = atan2(siny_cosp, cosy_cosp);

    angles[0] = roll * 180 / M_PI;
    angles[1] = pitch * 180 / M_PI;
    angles[2] = yaw * 180 / M_PI;
}

int approx(float a)
{
    int A = (int)floor(a * 10);
    if (A % 2 == 1)
        return A--;
    else
        return A;
}