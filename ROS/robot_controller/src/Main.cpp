#include "UR3RobotController.h"
#include <tf/tf.h>
#include <tf/LinearMath/Quaternion.h>




//System header file
#include <iostream>
#include <string>
#include <stdio.h>

#if defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#elif (__unix__)

#include <termios.h>
#include <unistd.h>

#endif

char key_bindings[6] = {'q', 'w', 'e', 'r', 't', 'y'};

int posIndex = -1;
char selected_key(' ');

int getChar(void);

int getKeyIndex(char c)
{
    for (int i = 0; i < 6; i++)
    {
        if (key_bindings[i] == c)
        {
            return i;
        }
    }
    return -1;
}

#if defined(_WIN32) || defined(_WIN64)
int getChar(void)
{
    char input = _getch();
    std::cout << "Char Pressed = " << int(input) << std::endl;
    if (input == 44)
        return -4;
    else if (input == 46)
        return -3;
    else
        return input;
}
#elif (__unix__)

int getChar(void)
{
    int ch;
    struct termios oldt;
    struct termios newt;

    // Store old settings, and copy to new settings
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;

    // Make required changes and apply the settings
    newt.c_lflag &= ~(ICANON | ECHO);
    newt.c_iflag |= IGNBRK;
    newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
    newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
    newt.c_cc[VMIN] = 1;
    newt.c_cc[VTIME] = 0;
    tcsetattr(fileno(stdin), TCSANOW, &newt);

    // Get the current character
    ch = getchar();
    if (ch == 27)
    {
        ch = getchar();
        ch = getchar();
        if (ch == 65 || ch == 68)
        {
            ch = -3;
        }
        else
        {
            ch = -4;
        }
    }
    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

#endif

void modifyPose(geometry_msgs::Pose &delta_pose, int posIndex, int multiplier)
{
    tf::Quaternion quat;
    quat.setEulerZYX(0.0, 0.0, 0.0);
    double rot_step = 0.0174533;
    switch (posIndex)
    {
    case 0:
        delta_pose.position.x += multiplier * 0.0005;
        break;
    case 1:
        delta_pose.position.y += multiplier * 0.0005;
        break;
    case 2:
        delta_pose.position.z += multiplier * 0.0005;
        break;
    case 3:
        quat.setEulerZYX(0.0, 0.0, multiplier * rot_step);
        break;
    case 4:
        quat.setEulerZYX(0.0, multiplier * rot_step, 0.0);
        break;
    case 5:
        quat.setEulerZYX(multiplier * rot_step, 0.0, 0.0);
        break;
    default:
        std::cout << "Error" << std::endl;
    }
    delta_pose.orientation.x = quat.x();
    delta_pose.orientation.y = quat.y();
    delta_pose.orientation.z = quat.z();
    delta_pose.orientation.w = quat.w();
}

void manualRun(RobotController *robotController)
{
    std::cout << "Manual Run Selected" << std::endl;
    std::cout << "Please input : " << std::endl
              << "q for x translation" << std::endl
              << "w for y translation" << std::endl
              << "e for z translation" << std::endl
              << "r for x rotation" << std::endl
              << "t for y rotation" << std::endl
              << "y for z rotation" << std::endl
              << "after pressing one of the above key : " << std::endl
              << "Press < or > in windows OR -> and <- in unix to manipuate" << std::endl
              << "press any other key to terminate" << std::endl;
    while (true)
    {
        geometry_msgs::Pose delta_pose;
        delta_pose.position.x = delta_pose.position.y = delta_pose.position.z = 0.0;
        delta_pose.orientation.x = delta_pose.orientation.y = delta_pose.orientation.z = delta_pose.orientation.w = 0.0;

        int t = getChar();
        if ((t == int('A')) || (t == int('a')))
        { //Temp
            robotController->reset_cylinder();
            continue;
        }
        if (t < 0)
        {
            if (posIndex != -1)
            {
                if (t == -3)
                    modifyPose(delta_pose, posIndex, 1);
                else
                    modifyPose(delta_pose, posIndex, -1);

                robotController->set_delta_pose(delta_pose);
            }
        }
        else
        {
            selected_key = t;
            posIndex = getKeyIndex(selected_key);
            if (posIndex == -1)
                break;
        }
    }
}

void automatedInput(RobotController *robotController)
{
    double time = 10.0;
    double translation_step = 0.04 ;
    double rotation_step = 0.0174533 * 5; //5 degrees
    int repeat = 6;
    double x[12] =    {1.5, 0, 0, -1.5, 0,  0, 0,  0, 0,  0, 0,  0};
    double y[12] =    {0,  1.5, 0, 0, -1.5,  0, 0,  0, 0,  0, 0,  0};
    double z[12] =    {0,  0, 1.5, 0, 0, -1.5, 0,  0, 0,  0, 0,  0};
    double xRot[12] = {0,  0, 0,  0, 0,  0, 1.5, -1.5, 0,  0, 0,  0};
    double yRot[12] = {0,  0, 0,  0, 0,  0, 0,  0, 1.5, -1.5, 0,  0};
    double zRot[12] = {0,  0, 0,  0, 0,  0, 0,  0, 0,  0, 1.5, -1.5};


    std::this_thread::sleep_for(std::chrono::milliseconds(40000));
    geometry_msgs::Pose delta_pose;
    delta_pose.position.x = delta_pose.position.y = delta_pose.position.z = 0.0;
    delta_pose.orientation.x = delta_pose.orientation.y = delta_pose.orientation.z = delta_pose.orientation.w = 0.0;
    tf::Quaternion quat;
    delta_pose.position.x = 0.04;
    delta_pose.position.y = 0.02;
    delta_pose.position.z = 0.03;

    quat.setEulerZYX(0, 0, 0.0174533 * 10);
    delta_pose.orientation.x = quat.x();
    delta_pose.orientation.y = quat.y();
    delta_pose.orientation.z = quat.z();
    delta_pose.orientation.w = quat.w();
    robotController->set_delta_pose(delta_pose);

    std::cout << "Staring" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10000));
//    for (int i = 0; i < 5; i++)
//    {
//        std::cout << "Counter = " << i << std::endl;
        for (int j = 0; j < repeat; j++)
        {
            geometry_msgs::Pose delta_pose;
            delta_pose.position.x = delta_pose.position.y = delta_pose.position.z = 0.0;
            delta_pose.orientation.x = delta_pose.orientation.y = delta_pose.orientation.z = delta_pose.orientation.w = 0.0;

            tf::Quaternion quat;
            delta_pose.position.x = translation_step * x[j];
            delta_pose.position.y = translation_step * y[j];
            delta_pose.position.z = translation_step * z[j];

            quat.setEulerZYX(rotation_step * zRot[j], rotation_step * yRot[j], rotation_step * xRot[j]);
            delta_pose.orientation.x = quat.x();
            delta_pose.orientation.y = quat.y();
            delta_pose.orientation.z = quat.z();
            delta_pose.orientation.w = quat.w();

            robotController->set_delta_pose(delta_pose);
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        }
//        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
//    }
    std::cout << "Collection Ended." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    //robotController->writeJointStatesToFile();
}


int main(int argc, char **argv)
{
    RobotController *robotController;
    robotController = new UR3RobotController();

    if (false == robotController->initialize(argc, argv))
    {
        std::cout << "Robot Initialization Failed !!! Aborting" << std::endl;
        return 0;
    }

    bool automated_input = true;
    if (automated_input)
    {
        automatedInput(robotController);
    }
    else
    {
            manualRun(robotController);
            // automatedHead(robotController, "/home/hmc/Desktop/team_headtracka_ws/test_data.txt");
    }
    return 0;
}
//Fifty ten hull
