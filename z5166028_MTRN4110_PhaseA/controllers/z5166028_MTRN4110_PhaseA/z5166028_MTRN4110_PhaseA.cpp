// File:          z5166028_MTRN4110_PhaseA.cpp
// Date:          
// Description:   Epuck Controller Phase A
// Author:        Colin Li
// Modifications: 
// Platform:      Windows

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Compass.hpp>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdlib.h>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

#define PI 3.14159265358979323846
#define MOVESTEP 165/20
#define TURNSTEP PI/2*56.6/20/2
#define TIMESTEP 64
#define MAXSPEED 6.28

char wallDetection(double distanceValue);

int main(int argc, char **argv) {
    // create the Robot instance.
    Robot *robot = new Robot();

    // Set file location strings
    const std::string MOTION_PLAN_FILE_NAME = "../../MotionPlan.txt";
    const std::string MOTION_EXECUTION_FILE_NAME = "../../MotionExecution.csv";

    // Initialise variables
    int step = 0;
    int row;
    int col;
    char head;
    char leftWall;
    char frontWall;
    char rightWall;
    double curLeftPos = 0;
    double curRightPos = 0;
    std::string commandString;
    char commandStep;

    // Initialising motors
    Motor *leftMotor = robot->getMotor("left wheel motor");
    Motor *rightMotor = robot->getMotor("right wheel motor");
    leftMotor->setControlPID(15,0.01,0);
    rightMotor->setControlPID(15,0.01,0);

    // Initialising position sensors/encoders
    PositionSensor *leftPos = leftMotor->getPositionSensor();
    PositionSensor *rightPos = rightMotor->getPositionSensor();
    leftPos->enable(TIMESTEP);
    rightPos->enable(TIMESTEP);

    // Initialising distance sensors
    DistanceSensor *leftDist = robot->getDistanceSensor("left_ds");
    DistanceSensor *forwardDist = robot->getDistanceSensor("forward_ds");
    DistanceSensor *rightDist = robot->getDistanceSensor("right_ds");
    leftDist->enable(TIMESTEP);
    forwardDist->enable(TIMESTEP);
    rightDist->enable(TIMESTEP);
    
    // Initialising compass/magnetometer
    Compass *compass = robot->getCompass("compass");
    compass->enable(TIMESTEP);
    
    // Time buffer
    robot->step(TIMESTEP);

    std::cout << "[z5166028_MTRN4110_PhaseA] ";
    std::cout << "Reading in motion plan from ../../MotionPlan.txt...\n";
    
    // Reading MotionPlan.txt file
    std::ifstream readPlanFile(MOTION_PLAN_FILE_NAME);
    while (!readPlanFile.eof()) {
        std::getline(readPlanFile,commandString);
    }
    readPlanFile.close();
    
    std::cout << "[z5166028_MTRN4110_PhaseA] ";
    std::cout << "Motion Plan: " << commandString << "\n";
    std::cout << "[z5166028_MTRN4110_PhaseA] ";
    std::cout << "Motion Plan read in!" << "\n";
    std::cout << "[z5166028_MTRN4110_PhaseA] ";
    std::cout << "Executing motion plan..." << std::endl;

    // Reading initial conditions
    row = commandString[0] - '0';
    col = commandString[1] - '0';
    head = commandString[2];
    
    // Initial wall detection
    leftWall = wallDetection(leftDist->getValue());
    frontWall = wallDetection(forwardDist->getValue());
    rightWall = wallDetection(rightDist->getValue());

    std::cout << "[z5166028_MTRN4110_PhaseA] ";
    std::cout << "Step: " << std::setfill('0') << std::setw(3) << step;
    std::cout << ", Row: " << row;
    std::cout << ", Column: " << col;
    std::cout << ", Heading: " << head;
    std::cout << ", Left Wall: " << leftWall;
    std::cout << ", Front Wall: " << frontWall;
    std::cout << ", Right Wall: " << rightWall << std::endl;
    
    // Writing into .csv file
    std::ofstream writeExeFile(MOTION_EXECUTION_FILE_NAME);
    writeExeFile << "Step,Row,Column,Heading,Left Wall,Front Wall,Right Wall,\n";
    writeExeFile << step << "," << row << "," << col << "," << head << "," << leftWall << "," << frontWall << "," << rightWall << std::endl;
    
    step++;
    
    // Main loop
    while (robot->step(TIMESTEP) != -1) {
    
        // Iterating through motion plan
        while (commandString[step+2] != '\0') {
            // Determine encoder/position sensor values for current step
            curLeftPos = leftPos->getValue();
            curRightPos = rightPos->getValue();
            std::cout << curLeftPos << ", " << curRightPos << std::endl;
            
            // Parse current step command
            commandStep = commandString[step+2];
            
            // Movement
            if (commandStep == 'F') {
                leftMotor->setPosition(curLeftPos + MOVESTEP);
                leftMotor->setVelocity(MAXSPEED);
                rightMotor->setPosition(curRightPos + MOVESTEP);
                rightMotor->setVelocity(MAXSPEED);
                if (head == 'W') {
                    col--;
                } else if (head == 'N') {
                    row--;
                } else if (head == 'E') {
                    col++;
                } else if (head == 'S') {
                    row++;
                }
            }
            else if (commandStep == 'L') {
                leftMotor->setPosition(curLeftPos - TURNSTEP);
                leftMotor->setVelocity(MAXSPEED*0.4);
                rightMotor->setPosition(curRightPos + TURNSTEP);
                rightMotor->setVelocity(MAXSPEED*0.4);
            }
            else if (commandStep == 'R') {
                leftMotor->setPosition(curLeftPos + TURNSTEP);
                leftMotor->setVelocity(MAXSPEED*0.4);
                rightMotor->setPosition(curRightPos - TURNSTEP);
                rightMotor->setVelocity(MAXSPEED*0.4);
            }
            
            robot->step(30*TIMESTEP);
            
            // Heading
            const double *compReading = compass->getValues();
            double rad = atan2(compReading[0],compReading[2]);
            if (rad >= -3*PI/4 && rad < -PI/4) {
                head = 'W';
            } else if (rad >= -PI/4 && rad < PI/4) {
                head = 'N';
            } else if (rad >= PI/4 && rad < 3*PI/4) {
                head = 'E';
            } else {
                head = 'S';
            }
            
            // Wall Detection
            leftWall = wallDetection(leftDist->getValue());
            frontWall = wallDetection(forwardDist->getValue());
            rightWall = wallDetection(rightDist->getValue());
            
            // Writing console messages
            std::cout << "[z5166028_MTRN4110_PhaseA] ";
            std::cout << "Step: " << std::setfill('0') << std::setw(3) << step;
            std::cout << ", Row: " << row;
            std::cout << ", Column: " << col;
            std::cout << ", Heading: " << head;
            std::cout << ", Left Wall: " << leftWall;
            std::cout << ", Front Wall: " << frontWall;
            std::cout << ", Right Wall: " << rightWall << std::endl;
            
            // Writing information to MotionExecution.csv
            writeExeFile << step << "," << row << "," << col << "," << head << "," << leftWall << "," << frontWall << "," << rightWall << std::endl;
            
            step++;
        }
        std::cout << "[z5166028_MTRN4110_PhaseA] ";
        std::cout << "Motion plan executed!" << std::endl;
        writeExeFile.close();
        exit(0);
    };
  
    delete robot;
    return 0;
}

// Function to convert distance sensor value to Y/N
char wallDetection(double distanceValue) {
    char wall;
    if (distanceValue < 1000) {
        wall = 'Y';
    } else {
        wall = 'N';
    }
    return wall;
}