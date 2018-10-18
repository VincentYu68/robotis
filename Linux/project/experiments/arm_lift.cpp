#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"
#include <fstream>
#include <iostream>


using namespace Robot;

bool DEBUG_FIRST = true;
bool DEBUG_SECOND = true;


double angle2rad(double angle) {
    return angle / 180 * 3.141592653589;
}

double rad2angle(double rad) {
    return rad / 3.141592653589 * 180;
}

int main()
{
	printf( "\n===== Test DARwIn Armlifting =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

    ///////////////////// Initialize the Pose of Darwin /////////////////
    double init_pose_angle[] = {-0.84473936, 0.72186818, -0.31178562, 0.30717795, 0.51145128, -0.51605895,
                             0., 0., -0.00614356, 0.00614356, -0.68, 0.68, 0.88, -0.88, 0.5237384,-0.5237384, 0.01382301, -0.01382301,
                             0., 0.};
    int init_pose_value[20];
    for (int i = 0; i < 20; i++) {
        init_pose_value[i] = MX28::Angle2Value(rad2angle(init_pose_angle[i]));
    }

    int starting_value[20];
    for (int id = JointData::ID_R_SHOULDER_PITCH; id < JointData::NUMBER_OF_JOINTS; id++) {
        int value;
        if(cm730.ReadWord(id, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
			starting_value[id - JointData::ID_R_SHOULDER_PITCH] = value;
		}
		else{
		    printf("Cant read joint %d \n", id);
		    return 1;
		}
    }

    for (int i = 0; i < 20; i++)
        printf("%f,%d,%d\n",init_pose_angle[i], init_pose_value[i], starting_value[i]);
    if (DEBUG_FIRST)
        return 0;
	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

    for (int i = 0; i < 200; i++) {
        int n = 0;
        int joint_num = 0;
        for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
        {
            int goal_pos = i * 1.0 / 200 * init_pose_value[joint_num] + (200-i) * 1.0 / 200 * starting_value[joint_num];
            if (goal_pos > 4095 or goal_pos < 0) {
                printf("Goal pose for %d wrong!\n", id);
                return 1;
            }
            param[n++] = id;

            param[n++] = 0;
            param[n++] = 0;
            param[n++] = 32;
            param[n++] = 0;

            param[n++] = CM730::GetLowByte(goal_pos);
            param[n++] = CM730::GetHighByte(goal_pos);
            joint_num++;
        }

        usleep(50000);

        if(joint_num > 0)
            cm730.SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
    }

    if (DEBUG_SECOND)
        return 0;
    ///////////////////////////////////////////////////////////////////////


    std::ofstream fileStream;
    char fname[32] = {0,};
    sprintf(fname, "data.txt");    
    fileStream.open(fname, std::ios::out);

	int value;

	int num_steps = 0;
	int max_steps = 200;
	int interp_steps = 100; // at which step should reach the goal

	int init_shoulder_pitch = 2048;
	int init_shoulder_row = 2048;
	int init_elbow = 2048;

	int goal_shoulder_pitch = 0;
	int goal_shoulder_row = 2048;
	int goal_elbow = 4095;

	while(num_steps < max_steps)
	{
	    double cur_shoulder_pitch_angle, cur_shoulder_row_angle, cur_elbow_angle;

		if(cm730.ReadWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_shoulder_pitch_angle = MX28::Value2Angle(value);
		    int goal = (num_steps * 1.0 / interp_steps) * goal_shoulder_pitch + (interp_steps - num_steps) * 1.0 / interp_steps * init_shoulder_pitch;
			cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else
			printf("Can't read angle from l shoulder pitch");
			return 1;

		if(cm730.ReadWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_shoulder_row_angle = MX28::Value2Angle(value);
		    int goal = (num_steps * 1.0 / interp_steps) * goal_shoulder_row + (interp_steps - num_steps) * 1.0 / interp_steps * init_shoulder_row;
			cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else
			printf("Can't read angle from l shoulder row");
			return 1;

	    if(cm730.ReadWord(JointData::ID_L_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_elbow_angle = MX28::Value2Angle(value);
		    int goal = (num_steps * 1.0 / interp_steps) * goal_elbow + (interp_steps - num_steps) * 1.0 / interp_steps * init_elbow;
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else
			printf("Can't read angle from l elbow");
			return 1;

		fileStream << angle2rad(cur_shoulder_pitch_angle) << " " << angle2rad(cur_shoulder_row_angle) << " " << angle2rad(cur_elbow_angle) << std::endl;

		num_steps ++;

		usleep(20000);
	}

	fileStream.close();

	return 0;
}
