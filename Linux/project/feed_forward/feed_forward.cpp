#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"
#include <fstream>
#include <iostream>
#include <vector>


using namespace Robot;



double angle2rad(double angle) {
    return angle / 180 * 3.141592653589;
}

double rad2angle(double rad) {
    return rad / 3.141592653589 * 180;
}



int main()
{
	printf( "\n===== Test DARwIn Feedforward =====\n\n");

	//////////////////// Framework Initialize ////////////////////////////
	LinuxCM730 linux_cm730("/dev/ttyUSB0");
	CM730 cm730(&linux_cm730);
	if(cm730.Connect() == false)
	{
		printf("Fail to connect CM-730!\n");
		return 0;
	}
	/////////////////////////////////////////////////////////////////////

    ///////////////////// Load feedforward data /////////////////////////
    std::vector<std::vector<int> > feedforward_data;
    std::ifstream dataStream;
    int data;
    char fname2[32] = {0,};
    sprintf(fname2, '../walk_tuner/Log/Log0.txt');
    dataStream.open(fname2, std::ios::in);
    while (!dataStream.eof()) {
        feedforward_data.push_back(std::vector<int>());
        for (int i = 0; i < 20; i++) {
            dataStream >> data;
            feedforward_data[feedforward_data.size()-1].push_back(data);
        }
    }
    dataStream.close();
    printf("Loaded %d steps\n", feedforward_data.size());
    /////////////////////////////////////////////////////////////////////

    ///////////////////// Initialize the Pose of Darwin /////////////////
    int init_pose_value[20];
    for (int i = 0; i < 20; i++) {
        init_pose_value[i] = feedforward_data[0][i];
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

    int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

    for (int i = 0; i < feedforward_data.size(); i++) {
        int n = 0;
        int joint_num = 0;
        for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
        {
            int goal_pos = i * 1.0 / 100 * init_pose_value[joint_num] + (100-i) * 1.0 / 100 * starting_value[joint_num];
            if (goal_pos > 4095 or goal_pos < 0) {
                printf("Goal pose for %d wrong!\n", id);
                return 1;
            }
            param[n++] = id;

            param[n++] = 16;
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


    ///////////////////////////////////////////////////////////////////////
    int num_steps = 0;
    static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC,&next_time);
    int prev_time_micrsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0; // convert to microsecond
    int current_time_microsec = 0;
	for (int i = 0; i < feedforward_data.size(); i++)
	{
	    int n = 0;
        int joint_num = 0;
        for(int id=JointData::ID_R_SHOULDER_PITCH; id<JointData::NUMBER_OF_JOINTS; id++)
        {
            int goal_pos = feedforward_data[i][id];
            if (goal_pos > 4095 or goal_pos < 0) {
                printf("Goal pose for %d wrong!\n", id);
                return 1;
            }
            param[n++] = id;

            param[n++] = 16;
            param[n++] = 0;
            param[n++] = 32;
            param[n++] = 0;

            param[n++] = CM730::GetLowByte(goal_pos);
            param[n++] = CM730::GetHighByte(goal_pos);
            joint_num++;
        }

        if(joint_num > 0)
            cm730.SyncWrite(MX28::P_D_GAIN, MX28::PARAM_BYTES, joint_num, param);


        clock_gettime(CLOCK_MONOTONIC,&next_time);
        current_time_microsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0; // convert to microsecond

		num_steps ++;
        int wait_time = 30000 - (current_time_microsec - prev_time_micrsec);
        //printf("wait %d ms\n", wait_time);
        if (wait_time >= 0)
		    usleep(wait_time);
		prev_time_micrsec = current_time_microsec;
	}

	dataStream.close();

	return 0;
}
