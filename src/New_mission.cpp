#include "Dstar.h"
#include "headers/gdpdrone.h"

int approx(float a)
{
    int A = (int)floor(a * 10);
    if (A % 2 == 1)
        return A--;
    else
        return A;
}

int main(int argc, char **argv)
{
    // Initialise node
    ros::init(argc, argv, "Bukake_node");

    GDPdrone drone;

// Set the rate. Default working frequency is 25 Hz
    float loop_rate = 25.0;
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

    float pitch=4.35;
    int xobst, yobst;

    while (drone.Data.lidar.ranges[3] == INFINITY && drone.Data.lidar.ranges[4] == INFINITY && drone.Data.lidar.ranges[5] == INFINITY)
    {
        drone.Commands.move_Velocity_Local(0, 0.5, 0, 0, "BODY_OFFSET");
        ros::spinOnce();
        rate.sleep();
        xobst = approx(drone.Data.lidar.ranges[1]*cos(pitch * M_PI / 180)); //this should be out of HERE!
    }

    yobst=approx(drone.Data.lidar.ranges[1]*((45 - pitch) * M_PI / 180);

    ROS_INFO("xObst [%i]", xObst);
    ROS_INFO("yObst [%i]", yObst);

int Count;
int distance = 400; //gps distance in m between goal and where I am
int resolution = 2; //Resolution is 2/10 m;
int Altitude = (int)altitude;

dstar->init(0, Altitude, distance / resolution, 0);
for (int i = -distance / resolution; i <= distance / resolution; i++)
        dstar->updateCell(i, -1, -1); //set the ground to be non-traversable

for (int i = 0; i <= yobst / resolution; i++)
        dstar->updateCell(xobst * resolution, i, -1);

int xst =0, yst =Altitude *5;

while (!(xst == distance / resolution && yst == 0))
    {
        dstar->replan();           // plan a path
        mypath = dstar->getPath(); // retrieve path

        for (auto &v : mypath)
        {
            dstar->updateStart(v.x, v.y); // move start to new path point
            ROS_INFO("v.x [%i]", v.x);
            ROS_INFO("v.y [%i]", v.y);
            Count = 0;
        
            while (Count != 20)
            {
                drone.Commands.move_Position_Local(0, (v.x-xst)* 0.2, (v.y-yst) * 0.2, 0, "BODY_OFFSET", Count);
                Count++;
                ros::spinOnce();
                rate.sleep();
            }
            xst = v.x;
            yst = v.y;
            /*if (v.x == xobst / resolution && v.y == yobst / resolution + 1)
            {
                //Check for new obstacles
                xObst += 3;
                yObst += 2; //We update the x and y of the new obstacles relative to our current position
                for (int i = 0; i <= yObst / resolution; i++)
                    dstar->updateCell(xObst / resolution, i, -1); // set cells (xobst,i) to be non traversable
                break;
            }*/
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
