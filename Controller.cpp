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

//Obstacle DetectObstacles(Robot *robot, DistanceSensor *ps[3]);
Obstacle DetectObstacles(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS);
void GoForward(Robot *robot, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftSensor);
void Turn(Robot *robot, Motor *leftMotor, Motor *rightMotor, Compass *compass, char direction);
double GetBearing(const double *comValue);
Location GetLocation(Robot *robot, GPS *gps);
void PrintInformation(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS, GPS *gps, Compass *compass, int step);
char GetHeading(double bearing);
//double round(double number, double round);

int main()
{

	//declare input file stream
    std::ifstream inFile(PATH_PLAN_FILE_NAME);
    
    //copy contents of inFile to path
    std::string path;
    getline(inFile,path);
    int PathLength = path.length();


    /*int InitialRow = (path[0]) - '0';
    int InitialColumn = (path[1]) - '0';
    char InitialHeading = path[2];
    int InitialStep = 00;*/
    
    //std::cout << "Step: " << std::setfill('0') << std::setw(2) << InitialStep << ", Row: " << InitialRow << ", Column: " << InitialColumn << ", Heading: " << InitialHeading << std::endl;
    
    //Obstacle starting = DetectObstacles(robot, ps);
    //std::cout << starting.LeftWall << starting.FrontWall << starting.RightWall << std::endl;


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
    const double *comValue = compass->getValues();
    //std::cout << "comValue0: " << comValue[0] << std::endl;
    //std::cout << "comValue2: " << comValue[2] << std::endl;
    //double bearing = GetBearing(comValue);

    //std::cout << bearing << std::endl;

    //initialise GPS
    GPS *gps = robot->getGPS("gps");
    gps->enable(TIME_STEP);
    //const double *gpsValue = gps->getValues();
    //std::cout << "gpsValue0: " << gpsValue[0] << std::endl;
    //std::cout << "gpsValue2: " << gpsValue[2] << std::endl;

    //Turn(robot, leftMotor, rightMotor, compass, 'r');
    //GoForward(robot, leftMotor, rightMotor);

    //initialise Position Sensors
    PositionSensor *leftSensor = robot->getPositionSensor("left wheel sensor");
    PositionSensor *rightSensor = robot->getPositionSensor("right wheel sensor");
    leftSensor->enable(TIME_STEP);
    rightSensor->enable(TIME_STEP);

    /*double LeftWheelPosition = leftSensor->getValue();
    double RightWheelPosition = rightSensor->getValue();
    std::cout << "LeftWheelPosition: " << LeftWheelPosition << "   RightWheelPosition: " << RightWheelPosition << std::endl;
    */

    int step = 0;

    /*if (robot->step(TIME_STEP) != -1)
    {
    	PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
        step = step + 1;
    }*/

    // feedback loop: step simulation until an exit event is received
    while (robot->step(TIME_STEP) != -1) {
    
        //Obstacle current = DetectObstacles(robot, ps[0], ps[1], ps[2]);
        //std::cout << current.LeftWall << "  " << current.FrontWall << "  " << current.RightWall  << std::endl;

        //int counter = 3;
        //std::cout << PathLength << std::endl;
        
        //int step = 0;
        //PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
        //Sleep(10000);

        if (robot->step(TIME_STEP) != -1)
        {
        	PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
            step = step + 1;
            //robot->step(TIME_STEP);
        }

        for (int counter = 3; counter < PathLength; counter++)
        {
        	//std::cout << "counter " << counter << std::endl;
        	
            //PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
            //step = step + 1;

            if (path[counter] == 'F')
            {
                GoForward(robot, leftMotor, rightMotor, leftSensor);
            }
            else if (path[counter] == 'L')
            {
                Turn(robot, leftMotor, rightMotor, compass, 'l');
            }
            else if (path[counter] == 'R')
            {
                Turn(robot, leftMotor, rightMotor, compass, 'r');
            }

            PrintInformation(robot, ps[0], ps[1], ps[2], gps, compass, step);
            step = step + 1;
            //Sleep(1000);
            
        }


        //const double *gpsValue = gps->getValues();
        //std::cout << "gpsValue0: " << gpsValue[0] << std::endl;
        //std::cout << "gpsValue2: " << gpsValue[2] << std::endl;

        Location Current = GetLocation(robot, gps);
        //std::cout << "row: " << Current.Row << std::endl;
        //std::cout << "col: " << Current.Col << std::endl;

        //initialise compass
        Compass *compass = robot->getCompass("compass");
        compass->enable(TIME_STEP);
        const double *comValue = compass->getValues();
        double bearing = GetBearing(comValue);
        //std::cout << "bearing: " << bearing << std::endl;

        double LeftWheelPosition = leftSensor->getValue();
        double RightWheelPosition = rightSensor->getValue();
        //std::cout << "LeftWheelPosition: " << LeftWheelPosition << "   RightWheelPosition: " << RightWheelPosition << std::endl;



        /*Turn(robot, leftMotor, rightMotor, compass, 'r');
        Turn(robot, leftMotor, rightMotor, compass, 'r');
        Turn(robot, leftMotor, rightMotor, compass, 'r');
        Turn(robot, leftMotor, rightMotor, compass, 'r');
        Turn(robot, leftMotor, rightMotor, compass, 'l');
        Turn(robot, leftMotor, rightMotor, compass, 'r');
        Turn(robot, leftMotor, rightMotor, compass, 'l');
        Turn(robot, leftMotor, rightMotor, compass, 'l');
        Turn(robot, leftMotor, rightMotor, compass, 'l');
        Turn(robot, leftMotor, rightMotor, compass, 'r');
        Turn(robot, leftMotor, rightMotor, compass, 'r');*/


        /*GoForward(robot, leftMotor, rightMotor, leftSensor);
        GoForward(robot, leftMotor, rightMotor, leftSensor);
        GoForward(robot, leftMotor, rightMotor, leftSensor);*/

        std::cout << "Done - Path plan executed!" << std::endl;
        break;
    }

   
    inFile.close();
    delete robot;
    return 0;
};

void GoForward(Robot *robot, Motor *leftMotor, Motor *rightMotor, PositionSensor *leftSensor)
{
    /*// set the target position of the motors
    leftMotor->setPosition(8.25);
    rightMotor->setPosition(8.25);*/

    // initialize motor speeds
    double leftSpeed  = 0;
    double rightSpeed = 0;

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);


    double LeftWheelPosition = leftSensor->getValue();
    //double RightWheelPosition = rightSensor->getValue();

    //the next desired position is 8.25 rad further, rounded to the nearest multiple of 8.25
    //double DesiredLeftWheelPosition = round(LeftWheelPosition + 8.25, 8.25);
    double DesiredLeftWheelPosition = LeftWheelPosition + 8.25;
    //double DesiredRightWheelPosition = RightWheelPosition + 8.25;

    //std::cout << "Wheel location: " << LeftWheelPosition << "  Desired Wheel location: " << DesiredLeftWheelPosition << std::endl;

    while (robot->step(TIME_STEP) != -1)
	{
		double LeftWheelPosition = leftSensor->getValue();
        //std::cout << "Left wheel position: " << LeftWheelPosition << std::endl;

        double error = DesiredLeftWheelPosition - LeftWheelPosition;

        leftSpeed = 10*error;
        rightSpeed = 10*error;

        if (leftSpeed > MAX_SPEED)
        {
            leftSpeed = MAX_SPEED;
        }
        else if (leftSpeed < -MAX_SPEED)
        {
            leftSpeed = -MAX_SPEED;
        }

        if (rightSpeed > MAX_SPEED)
        {
         	rightSpeed = MAX_SPEED;
        }
        else if (rightSpeed < -MAX_SPEED)
        {
           rightSpeed = -MAX_SPEED;
        }

        // write actuators inputs
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);

        if (LeftWheelPosition == DesiredLeftWheelPosition || error < 0.01)
        {
        	leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            //std::cout << "finished moving" << std::endl;
            break;
        }


	}
    

    //std::cout << "LeftWheelPosition: " << LeftWheelPosition << "   RightWheelPosition: " << RightWheelPosition << std::endl;
   // std::cout << "done" << std::endl;
    return;
}

void Turn(Robot *robot, Motor *leftMotor, Motor *rightMotor, Compass *compass, char direction)
{
	/*int r = 1;
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
    rightMotor->setPosition(l*-10);*/

    const double *comValue = compass->getValues();
    double bearing = GetBearing(comValue);
    double DesiredBearing = 0;
    
    // initialize motor speeds
    double leftSpeed  = 0;
    double rightSpeed = 0;
    int leftMultiplier = 0;
    int rightMultiplier = 0;

    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);
    rightMotor->setVelocity(0.0);

    
    //if the robot is turning right, set desired bearing to current + 90 degrees
    if (direction == 'r')
	{
        DesiredBearing = bearing + 90;
        DesiredBearing = nearbyint(DesiredBearing);
        if (DesiredBearing > 360.0)
        {
            DesiredBearing = DesiredBearing - 360;
        }

        leftMultiplier = 1;
        rightMultiplier = -1;
	}

	//if the robot is turning left, set desired bearing to current - 90 degrees
	else if (direction == 'l')
	{
        DesiredBearing = bearing - 90;
        DesiredBearing = nearbyint(DesiredBearing);
        if (DesiredBearing < 0.0)
        {
            DesiredBearing = DesiredBearing + 360;
        }

        leftMultiplier = -1;
        rightMultiplier = 1;
	}

    //std::cout << "bearing: " << bearing << "  DesiredBearing: " << DesiredBearing << std::endl;

	while (robot->step(TIME_STEP) != -1)
	{
        const double *comValue = compass->getValues();
        double bearing = GetBearing(comValue);
        //std::cout << bearing << std::endl;

        double error = abs(DesiredBearing - bearing);
        leftSpeed = 0.1*error*leftMultiplier;
        rightSpeed = 0.1*error*rightMultiplier;

        if (leftSpeed > MAX_SPEED)
        {
        	leftSpeed = MAX_SPEED;
        }
        else if (leftSpeed < -MAX_SPEED)
        {
            leftSpeed = -MAX_SPEED;
        }

        if (rightSpeed > MAX_SPEED)
        {
        	rightSpeed = MAX_SPEED;
        }
        else if (rightSpeed < -MAX_SPEED)
        {
            rightSpeed = -MAX_SPEED;
        }

		// write actuators inputs
        leftMotor->setVelocity(leftSpeed);
        rightMotor->setVelocity(rightSpeed);
        

        if (bearing == DesiredBearing || error < 0.1)
        {
        	leftMotor->setVelocity(0.0);
            rightMotor->setVelocity(0.0);
            //std::cout << "finished turning" << std::endl;
            break;
        }
	}

	//std::cout << "done" << std::endl;
	return;
}

double GetBearing(const double *comValue)
{
  //const double *north = wb_compass_get_values(tag);
  double rad = atan2(comValue[0], comValue[2]);
  double bearing = (rad - 1.5708) / M_PI * 180.0;

  if (bearing < 0.0)
  {
      bearing = bearing + 360.0;
  }
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

Obstacle DetectObstacles(Robot *robot, DistanceSensor *LDS, DistanceSensor *FDS, DistanceSensor *RDS)
{
	Obstacle obstacle;
    
    double psValues[3];
    
    psValues[0] = LDS->getValue();
    psValues[1] = FDS->getValue();
    psValues[2] = RDS->getValue();

    //std::cout << psValues[0] << "  " << psValues[1] << "  " << psValues[2]  << std::endl;
    

    //detect obstacles
    bool LeftObstacle = psValues[0] < 500.0;
    bool FrontObstacle = psValues[1] < 500.0;
    bool RightObstacle = psValues[2] < 500.0;

    
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
    //const double *gpsValue = gps->getValues();
    //std::cout << "gpsValue0: " << gpsValue[0] << std::endl;
    //std::cout << "gpsValue2: " << gpsValue[2] << std::endl;

    Location CurrentLocation = GetLocation(robot, gps);
    //std::cout << "row: " << CurrentLocation.Row << std::endl;
    //std::cout << "col: " << CurrentLocation.Col << std::endl;

    const double *comValue = compass->getValues();
    double bearing = GetBearing(comValue);
    char heading = GetHeading(bearing);

    Obstacle CurrentObstacles = DetectObstacles(robot, LDS, FDS, RDS);
    //std::cout << current.LeftWall << "  " << current.FrontWall << "  " << current.RightWall  << std::endl;



    std::cout << "Step: " << std::setfill('0') << std::setw(2) << step << ", Row: " << CurrentLocation.Row << ", Column: " << CurrentLocation.Col << ", Heading: " << heading << ", Left Wall: " << CurrentObstacles.LeftWall << ", Front Wall: " << CurrentObstacles.FrontWall << ", Right Wall: " << CurrentObstacles.RightWall << std::endl;
    
    return;
}

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

/*
double round(double number, double round)
{
    double roundedNumber = 0;

    if ((std::fmod(number,round)) < round/2)
    {
        roundedNumber = number - (std::fmod(number,round));
    }
    else
    {
    	roundedNumber = number - (std::fmod(number,round)) + round;
    }

	return roundedNumber;
}*/


/*
go forward

// All the webots classes are defined in the "webots" namespace
using namespace webots;

int main(int argc, char **argv) {
 Robot *robot = new Robot();

 // get the motor devices
 Motor *leftMotor = robot->getMotor("left wheel motor");
 Motor *rightMotor = robot->getMotor("right wheel motor");
 // set the target position of the motors
 leftMotor->setPosition(10.0);
 rightMotor->setPosition(10.0);

 while (robot->step(TIME_STEP) != -1);

 delete robot;

 return 0;
}

