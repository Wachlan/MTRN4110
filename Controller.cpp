#include <fstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Windows.h>
#include <iomanip>
#include <string>
#include <cmath>
#include <math.h>
#include <ctgmath>

#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Motor.hpp>
#include <webots/Compass.hpp>
#include <webots/GPS.hpp>
#include <webots/PositionSensor.hpp>

#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
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

Obstacle DetectObstacles(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS);
void GoForward(Robot *robot, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftSensor, int repeat, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS, GPS *gps, Compass *compass, int step);
void Turn(Robot *robot, Motor *leftMotor, Motor *rightMotor, Compass *compass, char direction);
double GetBearing(const double *comValue);
Location GetLocation(Robot *robot, GPS *gps);
void PrintInformation(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS, GPS *gps, Compass *compass, int step);
char GetHeading(double bearing);
void ExecutePath();

int main()
{
    ExecutePath();

    return 0;
};

void GoForward(Robot *robot, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftSensor, int repeat, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS, GPS *gps, Compass *compass, int step)
{
    int PrintFlag1 = 1, PrintFlag2 = 1;

    // initialize motor speeds
    double leftSpeed = 0, rightSpeed = 0;

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    //Since both wheels move equal amounts to go forward, so only need to use one
    double LeftWheelPosition = leftSensor->getValue();

    //the next desired position is 8.25 rad further, multiplied by the number of squares to go forward
    double DesiredLeftWheelPosition = LeftWheelPosition + 8.25*repeat;
    
    while (robot->step(TIME_STEP) != -1)
	{
		double LeftWheelPosition = leftSensor->getValue();
        double error = DesiredLeftWheelPosition - LeftWheelPosition;

        if (repeat > 1)
        {
        	if (error > 16.25 && error < 16.75 && PrintFlag2 == 1 && repeat == 3)
        	{
                PrintInformation(robot, LDS, FDS, RDS, gps, compass, step);
                PrintFlag2 = 0;
                step = step + 1;
        	}
        	if (error > 8 && error < 8.5 && PrintFlag1 == 1)
        	{
                PrintInformation(robot, LDS, FDS, RDS, gps, compass, step);
                PrintFlag1 = 0;
                step = step + 1;
        	}
        	
        }

        if (error >= 0.5)
        {
            leftSpeed = MAX_SPEED;
            rightSpeed = MAX_SPEED;
        }
        else if (error < 0.5) //proportional closed feedback control
        {
            leftSpeed = 10*error;
            rightSpeed = 10*error;
        }

        // write actuators inputs
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        //once the position has been reached, stop motors
        if (LeftWheelPosition == DesiredLeftWheelPosition || error < 0.01)
        {
        	leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            break;
        }
	}
    
    return;
}

void Turn(Robot *robot, Motor *leftMotor, Motor *rightMotor, Compass *compass, char direction)
{

    const double *comValue = compass->getValues();
    double bearing = GetBearing(comValue);
    double DesiredBearing = 0;
    
    // initialize motor speeds
    double leftSpeed = 0, rightSpeed = 0;
    int leftMultiplier = 0, rightMultiplier = 0;

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    
    //if the robot is turning right, set desired bearing to current + 90 degrees
    if (direction == 'r')
	{
        DesiredBearing = bearing + 90;
        DesiredBearing = nearbyint(DesiredBearing); //correct any error from the last turn
        if (DesiredBearing > 360.0)                 //ensure bearing is in the range of 0-360
        {
            DesiredBearing = DesiredBearing - 360;
        }

        leftMultiplier = 1;                         //ensure wheels spin in the correct directions
        rightMultiplier = -1;
	}

	//if the robot is turning left, set desired bearing to current - 90 degrees
	else if (direction == 'l')
	{
        DesiredBearing = bearing - 90;
        DesiredBearing = nearbyint(DesiredBearing);
        if (DesiredBearing < 0.0)                     //ensure bearing is in the range of 0-360
        {
            DesiredBearing = DesiredBearing + 360;
        }

        leftMultiplier = -1;
        rightMultiplier = 1;
	}
    
	while (robot->step(TIME_STEP) != -1)
	{
        const double *comValue = compass->getValues();
        double bearing = GetBearing(comValue);

        //use proportional closed loop feedback control
        double error = abs(DesiredBearing - bearing);

        if (error >= 10)
        {
            leftSpeed = MAX_SPEED*leftMultiplier;
            rightSpeed = MAX_SPEED*rightMultiplier;
        }
        else if (error < 10)
        {
            leftSpeed = 0.1*error*leftMultiplier;
            rightSpeed = 0.1*error*rightMultiplier;
        }
        
		// write actuators inputs
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);
        

        if (bearing == DesiredBearing || error < 0.4)
        {
        	leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            break;
        }
	}

	return;
}

double GetBearing(const double *comValue)
{
  double rad = atan2(comValue[0], comValue[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;

  if (bearing < 0.0)
  {
      bearing = bearing + 360.0;
  }
  return bearing;
}

//function to convert GPS co-ordinates into row and column, returns results in a Location struct
Location GetLocation(Robot *robot, GPS *gps)
{
	Location CurrentLocation;
    const double *gpsValue = gps->getValues();
    CurrentLocation.Row = (int)6.0606*gpsValue[2] + 2.48157;
    CurrentLocation.Col = (int)6.0606*gpsValue[0] + 4.484844;

    return CurrentLocation;
}

//function to return which directions have obstacles
Obstacle DetectObstacles(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS)
{
	Obstacle obstacle;  //initialise obstacle struct
    
    bool LeftObstacle = LDS->getValue() < 600.0;
    bool FrontObstacle = FDS->getValue() < 600.0;
    bool RightObstacle = RDS->getValue() < 600.0;

    
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
    
    return obstacle;
};


void PrintInformation(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS, GPS *gps, Compass *compass, int step)
{
    //get current row and column location
    Location CurrentLocation = GetLocation(robot, gps);

    const double *comValue = compass->getValues();
    double bearing = GetBearing(comValue);         //get current bearing
    char heading = GetHeading(bearing);            //convert bearing into a heading

    Obstacle CurrentObstacles = DetectObstacles(robot, LDS, FDS, RDS); //detect obstacles

    std::cout << "Step: " << std::setfill('0') << std::setw(2) << step << ", Row: " << CurrentLocation.Row << ", Column: " << CurrentLocation.Col << ", Heading: " << heading << ", Left Wall: " << CurrentObstacles.LeftWall << ", Front Wall: " << CurrentObstacles.FrontWall << ", Right Wall: " << CurrentObstacles.RightWall << std::endl;
    
    return;
}

//function to convert quantitative bearing into qualitative heading
char GetHeading(double bearing)
{
    char heading = ' ';

    if (bearing > 350 || bearing < 10)
    {
        heading = 'N';
    }

    else if (bearing > 80 && bearing < 100)
    {
        heading = 'E';
    }

    else if (bearing > 170 && bearing < 190)
    {
        heading = 'S';
    }

    else if (bearing > 260 && bearing < 280)
    {
        heading = 'W';
    }

    return heading;
}

void ExecutePath()
{

	//declare input file stream
    std::ifstream inFile(PATH_PLAN_FILE_NAME);
    
    //copy contents of inFile to path
    std::string path;
    getline(inFile,path);
    int PathLength = path.length();

    //print information
    std::cout << "Start - Read path plan from " << PATH_PLAN_FILE_NAME << ":" << std::endl;
    std::cout << path << std::endl;
    std::cout << "Done - Path plan read!" << std::endl;
    std::cout << "Start - Execute path plan!" << std::endl;

	// create the Robot instance.
    Robot *robot = new Robot();

    // initialize devices
    DistanceSensor *ps[3];
    char psNames[3][4] = 
    {
    "LDS", "FDS", "RDS"
    };

    //initialise distance sensors
    for (int i = 0; i < 3; i++) {
        ps[i] = robot->getDistanceSensor(psNames[i]);
        ps[i]->enable(TIME_STEP);
    }

    //initialise motors
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");


    //initialise compass
    Compass *compass = robot->getCompass("compass");
    compass->enable(TIME_STEP);
    //const double *comValue = compass->getValues();

    //initialise GPS
    GPS *gps = robot->getGPS("gps");
    gps->enable(TIME_STEP);

    //initialise Position Sensors
    PositionSensor *leftSensor = robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightSensor = robot->getPositionSensor("right wheel sensor");
    leftSensor->enable(TIME_STEP);
    rightSensor->enable(TIME_STEP);


    int step = 0;

    // feedback loop: step simulation until an exit event is received
    while (robot->step(TIME_STEP) != -1) {
    
        //print out information for step 0
        if (robot->step(TIME_STEP) != -1)
        {
        	PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
            step = step + 1;
        }

        int counter = 3;

        //loop through instructions and respond accordingly
        while(counter < PathLength)
        {
            if (path[counter] == 'F' && path[counter+1] == 'F' && path[counter+2] == 'F')
            {
                GoForward(robot, leftMotor, rightMotor, leftSensor, 3, ps[0], ps[1], ps[2], gps, compass, step);
                step = step + 2;
                counter = counter + 2;
            }
            else if (path[counter] == 'F' && path[counter+1] == 'F')
            {
                GoForward(robot, leftMotor, rightMotor, leftSensor, 2, ps[0], ps[1], ps[2], gps, compass, step);
                step = step + 1;
                counter++;
            }
            else if (path[counter] == 'F')
            {
                GoForward(robot, leftMotor, rightMotor, leftSensor, 1, ps[0], ps[1], ps[2], gps, compass, step);
            }
            else if (path[counter] == 'L')
            {
                Turn(robot, leftMotor, rightMotor, compass, 'l');
            }
            else if (path[counter] == 'R')
            {
                Turn(robot, leftMotor, rightMotor, compass, 'r');
            }
            //std::cout << "diong it here";
            PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
            step = step + 1;
            counter++;
        }


        std::cout << "Done - Path plan executed!" << std::endl;
        break;
    }

   
    inFile.close();
    delete robot;
}
