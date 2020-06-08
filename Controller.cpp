#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <iomanip>
#include <string>
#include <cmath>
#include <math.h>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>

#define PATH_PLAN_FILE_NAME "C:/Users/Lachlan/Documents/MTRN4110/PathPlan.txt"
//path plan 00SFLFFLFRFRFFFLFRFLFFLFRFLFLFFF
#define TIME_STEP 64 // time in [ms] of a simulation step
#define MAX_SPEED 6.28

using namespace webots;

struct Obstacle
{
    char LeftWall;
    char FrontWall;
    char RightWall;
};

struct Location
{
	int Row;
	int Col;
};

Obstacle DetectObstacles(Robot *robot, DistanceSensor *ps[3]);
void GoForward(Robot *robot, Motor *leftMotor, Motor *rightMotor);
void Turn(Robot *robot, Motor *leftMotor, Motor *rightMotor, char direction);
double GetBearing(const double *comValue);
Location GetLocation(Robot *robot, GPS *gps);

int main()
{

	//declare input file stream
    std::ifstream inFile(PATH_PLAN_FILE_NAME);
    
    //copy contents of inFile to path
    std::string path;
    getline(inFile,path);
    int PathLength = path.length();


    int InitialRow = (path[0]) - '0';
    int InitialColumn = (path[1]) - '0';
    char InitialHeading = path[2];
    int InitialStep = 00;
    
    //std::cout << "Step: " << std::setfill('0') << std::setw(2) << InitialStep << ", Row: " << InitialRow << ", Column: " << InitialColumn << ", Heading: " << InitialHeading << std::endl;
    
    //Obstacle starting = DetectObstacles(robot, ps);
    //std::cout << starting.LeftWall << starting.FrontWall << starting.RightWall << std::endl;


	// create the Robot instance.
    Robot *robot = new Robot();

    // initialize devices
    DistanceSensor *ps[3];
    char psNames[3][4] = 
    {
    "LDS", "FDS", "RDS"
    };

    for (int i = 0; i < 3; i++) {
        ps[i] = robot->getDistanceSensor(psNames[i]);
        ps[i]->enable(TIME_STEP);
    }

    //initialise motors
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");

    //Turn(robot, leftMotor, rightMotor, 'r');
    //GoForward(robot, leftMotor, rightMotor);

    //initialise compass
    Compass *compass = robot->getCompass("compass");
    compass->enable(TIME_STEP);
    const double *comValue = compass->getValues();
    //std::cout << "comValue0: " << comValue[0] << std::endl;
    //std::cout << "comValue2: " << comValue[2] << std::endl;
    double bearing = GetBearing(comValue);

    //std::cout << bearing << std::endl;

    //initialise GPS
    GPS *gps = robot->getGPS("gps");
    gps->enable(TIME_STEP);
    //const double *gpsValue = gps->getValues();
    //std::cout << "gpsValue0: " << gpsValue[0] << std::endl;
    //std::cout << "gpsValue2: " << gpsValue[2] << std::endl;

    // feedback loop: step simulation until an exit event is received
    while (robot->step(TIME_STEP) != -1) {
    // read sensors outputs
      /*  double psValues[3];
        for (int i = 0; i < 3 ; i++)
        {
            psValues[i] = ps[i]->getValue();
        }

        //std::cout << psValues[0] << "  " << psValues[1] << "  " << psValues[2]  << std::endl;

        //detect obstacles
        bool LeftObstacle = psValues[0] < 500.0;
        bool FrontObstacle = psValues[1] < 500.0;
        bool RightObstacle = psValues[2] < 500.0;

        Obstacle obstacle;

        if (LeftObstacle == 1)
        {
            obstacle.LeftWall = 'Y';
        }
        else
        {
            obstacle.LeftWall = 'N';
        }

        if (RightObstacle == 1)
        {
            obstacle.RightWall = 'Y';
        }
        else
        {
            obstacle.RightWall = 'N';
        }

        if (FrontObstacle == 1)
        {
            obstacle.FrontWall = 'Y';
        }
        else
        {
           obstacle.FrontWall = 'N';
        }

        std::cout << obstacle.LeftWall << "  " << obstacle.FrontWall << "  " << obstacle.RightWall  << std::endl;
*/
        //int counter = 3;
        //std::cout << PathLength << std::endl;

        /*for (int counter = 3; counter < PathLength; counter++)
        {
        	std::cout << "counter " << counter << std::endl;

            if (path[counter] == 'F')
            {
                GoForward(robot, leftMotor, rightMotor);
            }
            else if (path[counter] == 'L')
            {
                Turn(robot, leftMotor, rightMotor, 'L');
            }
            else if (path[counter] == 'R')
            {
                Turn(robot, leftMotor, rightMotor, 'R');
            }

            Sleep(5000);
        }*/

        //initialise GPS
        /*GPS *gps = robot->getGPS("gps");
        gps->enable(TIME_STEP);*/
        //const double *gpsValue = gps->getValues();
        //std::cout << "gpsValue0: " << gpsValue[0] << std::endl;
        //std::cout << "gpsValue2: " << gpsValue[2] << std::endl;

        Location Current = GetLocation(robot, gps);
        std::cout << "row: " << Current.Row << std::endl;
        std::cout << "col: " << Current.Col << std::endl;

        //initialise compass
        Compass *compass = robot->getCompass("compass");
        compass->enable(TIME_STEP);
        const double *comValue = compass->getValues();
        double bearing = GetBearing(comValue);
        //std::cout << "bearing: " << bearing << std::endl;

    }

   
    inFile.close();
    delete robot;
    return 0;
};

void GoForward(Robot *robot, Motor *leftMotor, Motor *rightMotor)
{
    // set the target position of the motors
    leftMotor->setPosition(8.25);
    rightMotor->setPosition(8.25);

    //std::this_thread::sleep_for(2s);
    //std::cout << "done" << std::endl;
    
    return;
}

void Turn(Robot *robot, Motor *leftMotor, Motor *rightMotor, char direction)
{
	int r = 1;
	int l = 1;

	if (direction == 'r')
	{
        r = -1;
	}
	else if (direction == 'l')
	{
        l = -1;
	}
	else
	{
		return;
	}

    // set the target position of the motors
    leftMotor->setPosition(r*-10);
    rightMotor->setPosition(l*-10);
    
}

double GetBearing(const double *comValue)
{
  //const double *north = wb_compass_get_values(tag);
  double rad = atan2(comValue[0], comValue[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;

  if (bearing < 0.0)
    bearing = bearing + 360.0;
  return bearing;
}

Location GetLocation(Robot *robot, GPS *gps)
{
	Location CurrentLocation;
    const double *gpsValue = gps->getValues();
    CurrentLocation.Row = (int)6.0606*gpsValue[2] + 2.48157;
    CurrentLocation.Col = (int)6.0606*gpsValue[0] + 4.484844;

    return CurrentLocation;
}

Obstacle DetectObstacles(Robot *robot, DistanceSensor *ps)
{
	Obstacle obstacle;
    /*// initialize devices
    DistanceSensor *ps[8];
    char psNames[8][4] = {
    "ps0", "ps1", "ps2", "ps3",
    "ps4", "ps5", "ps6", "ps7"
    };
  
    for (int i = 0; i < 8; i++) {
    ps[i] = robot->getDistanceSensor(psNames[i]);
    ps[i]->enable(TIME_STEP);
    }

	Obstacle obstacle;
    // read sensors outputs
    double psValues[8];
    for (int i = 0; i < 8 ; i++)
    {
      psValues[i] = ps[i]->getValue();
    }*/

    

   /* // detect obstacles
    bool RightObstacle =
      psValues[1] > 80.0 ||
      psValues[2] > 80.0;
    bool LeftObstacle =
      psValues[5] > 80.0 ||
      psValues[6] > 80.0;
    bool FrontObstacle =
      psValues[0] > 80.0 ||
      psValues[7] > 80.0;

    std::cout << psValues[1] << psValues[2] << psValues[5] << psValues[6] << psValues[0] << psValues[7] << std::endl;

    if (LeftObstacle == 1)
    {
        obstacle.LeftWall = 'Y';
    }
    else
    {
       obstacle.LeftWall = 'N';
    }

    if (RightObstacle == 1)
    {
        obstacle.RightWall = 'Y';
    }
    else
    {
       obstacle.RightWall = 'N';
    }

    if (FrontObstacle == 1)
    {
        obstacle.FrontWall = 'Y';
    }
    else
    {
       obstacle.FrontWall = 'N';
    }*/
    
    return obstacle;
};
