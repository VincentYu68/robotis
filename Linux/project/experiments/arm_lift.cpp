#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"
#include <fstream>
#include <iostream>


using namespace Robot;

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
    double init_pose_angle[] = {0.72186818,  0.30717795, -0.51605895, -0.84473936,
       -0.31178562,  0.51145128,  0.        ,  0.        ,  0.        ,
        0.00614356,  0.68      , -0.88      , -0.5237384 , -0.01382301,
        0.        , -0.00614356, -0.68      ,  0.88      ,  0.5237384 ,
        0.01382301};
    int init_pose_value[20];
    for (int i = 0; i < 20; i++) {
        init_pose_value[i] = MX28::Angle2Value(init_pose_angle[i]);
    }

    fprintf(init_pose_value);

	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];
    int n = 0;
    int joint_num = 0;
    for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
    {
        param[n++] = id;

        param[n++] = 0;
        param[n++] = 0;
        param[n++] = 32;
        param[n++] = 0;

        param[n++] = CM730::GetLowByte(init_pose_value[joint_num]);
        param[n++] = CM730::GetHighByte(init_pose_value[joint_num]);
        joint_num++;
    }

    if(joint_num > 0)
        m_CM730->SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);
    ///////////////////////////////////////////////////////////////////////


    std::ofstream fileStream;
    fileStream.open('data.txt', std::ios::out)

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
		    goal = (num_steps * 1.0 / interp_steps) * goal_shoulder_pitch + (interp_steps - num_steps) * 1.0 / interp_steps * init_shoulder_pitch;
			cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else
			printf("Can't read angle from l shoulder pitch");
			return 1;

		if(cm730.ReadWord(JointData::ID_L_SHOULDER_ROW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_shoulder_row_angle = MX28::Value2Angle(value);
		    goal = (num_steps * 1.0 / interp_steps) * goal_shoulder_row + (interp_steps - num_steps) * 1.0 / interp_steps * init_shoulder_row;
			cm730.WriteWord(JointData::ID_L_SHOULDER_ROW, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else
			printf("Can't read angle from l shoulder row");
			return 1;

	    if(cm730.ReadWord(JointData::ID_L_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_elbow_angle = MX28::Value2Angle(value);
		    goal = (num_steps * 1.0 / interp_steps) * goal_elbow + (interp_steps - num_steps) * 1.0 / interp_steps * init_elbow;
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else
			printf("Can't read angle from l elbow");
			return 1;

		fileStream << cur_shoulder_pitch_angle << " " << cur_shoulder_row_angle << " " << cur_elbow_angle << std::endl;

		num_steps ++;

		usleep(20000);
	}

	fileStream.close()

	return 0;
}
