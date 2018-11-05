#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"
#include <fstream>
#include <iostream>


using namespace Robot;



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
    double init_pose_angle[] = {-1.2, 0.72186818, -0.31178562, 0.30717795, 0.51145128, -0.31605895,
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

	int param[JointData::NUMBER_OF_JOINTS * MX28::PARAM_BYTES];

    for (int i = 0; i < 100; i++) {
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


    ///////////////////////////////////////////////////////////////////////

    std::ifstream configStream;
    char fname2[32] = {0,};
    int signal_cycle; // in ms
    int signal_magnitude;
    sprintf(fname2, "config.txt");
    configStream.open(fname2, std::ios::in);
    configStream >> signal_cycle >> signal_magnitude;
    configStream.close();
    printf("experiment configs: %d, %d\n", signal_cycle, signal_magnitude);

    std::ofstream fileStream;
    char fname[32] = {0,};
    sprintf(fname, "data_%d_%d.txt", signal_cycle, signal_magnitude);
    fileStream.open(fname, std::ios::out);

	int value;

	int num_steps = 0;
	int max_steps = 200;

	static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC,&next_time);
	int initial_time_microsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0; // convert to microsecond
	int prev_time_micrsec = initial_time_microsec;
	int initial_time_ms = initial_time_microsec / 1000;

	while(num_steps < max_steps)
	{
	    double cur_elbow_angle;
	    int elbow_goal = init_pose_value[5];
	    double elbow_goal_rad;

	    static struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC,&next_time);
	    int current_time_microsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0 - initial_time_microsec; // convert to microsecond
        int current_time = current_time_microsec / 1000; // in ms

        if ((current_time / signal_cycle) % 2 == 1) {
            elbow_goal = signal_magnitude + init_pose_value[5];
        }

	    if(cm730.ReadWord(JointData::ID_L_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_elbow_angle = MX28::Value2Angle(value);

		    if (elbow_goal > 3100 or elbow_goal < 1300) {
                printf("Goal pose for elbow wrong! %d \n", elbow_goal);
                return 1;
            }

		    //printf("Goal of elbow %d \n", goal);
		    elbow_goal_rad = MX28::Value2Angle(elbow_goal);
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, elbow_goal, 0);
		}
		else {
			printf("Can't read angle from l elbow");
			return 1;
		}


        clock_gettime(CLOCK_MONOTONIC,&next_time);
        current_time_microsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0; // convert to microsecond
        current_time = current_time_microsec / 1000;

		fileStream << angle2rad(cur_elbow_angle) << " " << angle2rad(elbow_goal_rad) << " " << current_time - initial_time_ms << std::endl;

		num_steps ++;
        int wait_time = 50000 - (current_time_microsec - prev_time_micrsec);
        //printf("wait %d ms\n", wait_time);
        if (wait_time >= 0)
		    usleep(wait_time);
		prev_time_micrsec = current_time_microsec;
	}

	fileStream.close();

	return 0;
}
