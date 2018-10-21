#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include "LinuxDARwIn.h"
#include <fstream>
#include <iostream>


using namespace Robot;

bool DEBUG_FIRST = false;
bool DEBUG_SECOND = false;


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
	int max_steps = 500;
	int interp_steps = 100; // at which step should reach the goal

	int init_shoulder_pitch = init_pose_value[1];
	int init_shoulder_row = init_pose_value[3];
	int init_elbow = init_pose_value[5];

	int goal_shoulder_pitch = 0;
	int goal_shoulder_row = init_pose_value[3];
	int goal_elbow = init_pose_value[5] + 512;

	static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC,&next_time);
	int initial_time_microsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0; // convert to microsecond

	while(num_steps < max_steps)
	{
	    double cur_shoulder_pitch_angle, cur_shoulder_row_angle, cur_elbow_angle;
	    double goals[3];
	    int gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z;

		if(cm730.ReadWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_shoulder_pitch_angle = MX28::Value2Angle(value);
		    int goal = (num_steps * 1.0 / interp_steps) * goal_shoulder_pitch + (interp_steps - num_steps) * 1.0 / interp_steps * init_shoulder_pitch;
		    if (num_steps >= interp_steps) {
		        goal = goal_shoulder_pitch;
		    }
		    if (goal > 4095 or goal < 0) {
                printf("Goal pose for pitch wrong! %d, %d \n", goal_shoulder_pitch, init_shoulder_pitch);
                return 1;
            }

		    //printf("Goal of pitch %d \n", goal);
		    goals[0] = MX28::Value2Angle(goal);
			cm730.WriteWord(JointData::ID_L_SHOULDER_PITCH, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else {
			printf("Can't read angle from l shoulder pitch");
			return 1;
		}

		if(cm730.ReadWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_shoulder_row_angle = MX28::Value2Angle(value);
		    int goal = (num_steps * 1.0 / interp_steps) * goal_shoulder_row + (interp_steps - num_steps) * 1.0 / interp_steps * init_shoulder_row;
		    if (num_steps >= interp_steps) {
		        goal = goal_shoulder_row;
		    }
		    if (goal > 4095 or goal < 0) {
                printf("Goal pose for roll wrong! %d, %d \n", goal_shoulder_row, init_shoulder_row);
                return 1;
            }

		    //printf("Goal of roll %d \n", goal);
		    goals[1] = MX28::Value2Angle(goal);
			cm730.WriteWord(JointData::ID_L_SHOULDER_ROLL, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else {
			printf("Can't read angle from l shoulder row");
			return 1;
		}

	    if(cm730.ReadWord(JointData::ID_L_ELBOW, MX28::P_PRESENT_POSITION_L, &value, 0) == CM730::SUCCESS)
		{
		    cur_elbow_angle = MX28::Value2Angle(value);
		    int goal = (num_steps * 1.0 / interp_steps) * goal_elbow + (interp_steps - num_steps) * 1.0 / interp_steps * init_elbow;
		    if (num_steps >= interp_steps) {
		        goal = goal_shoulder_row;
		    }
		    if (goal > 3100 or goal < 1300) {
                printf("Goal pose for elbow wrong! %d, %d \n", goal_elbow, init_elbow);
                return 1;
            }

		    //printf("Goal of elbow %d \n", goal);
		    goals[2] = MX28::Value2Angle(goal);
			cm730.WriteWord(JointData::ID_L_ELBOW, MX28::P_GOAL_POSITION_L, goal, 0);
		}
		else {
			printf("Can't read angle from l elbow");
			return 1;
		}


        // get gyro and accelerometer infor
        cm730.BulkRead();

        gyro_x = cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_X_L)
        gyro_y = cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Y_L)
        gyro_z = cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_GYRO_Z_L)
        accel_x = cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_X_L)
        accel_y = cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Y_L)
        accel_z = cm730.m_BulkReadData[CM730::ID_CM].ReadWord(CM730::P_ACCEL_Z_L)


		/*if(cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_X_L, &gyro_x, 0) != CM730::SUCCESS) {
		    printf("Can't read gyro x");
			return 1;
		}

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Y_L, &gyro_y, 0) != CM730::SUCCESS) {
		    printf("Can't read gyro y");
			return 1;
		}

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_GYRO_Z_L, &gyro_z, 0) != CM730::SUCCESS) {
		    printf("Can't read gyro z");
			return 1;
		}

        if(cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_X_L, &accel_x, 0) != CM730::SUCCESS) {
		    printf("Can't read accel x");
			return 1;
		}

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Y_L, &accel_y, 0) != CM730::SUCCESS) {
		    printf("Can't read accel y");
			return 1;
		}

		if(cm730.ReadWord(CM730::ID_CM, CM730::P_ACCEL_Z_L, &accel_z, 0) != CM730::SUCCESS) {
		    printf("Can't read accel z");
			return 1;
		}*/


        static struct timespec next_time;
        clock_gettime(CLOCK_MONOTONIC,&next_time);
        int current_time_microsec = next_time.tv_sec * 1000000 + next_time.tv_nsec / 1000.0; // convert to microsecond
        int current_time = current_time_microsec / 1000;

		fileStream << angle2rad(cur_shoulder_pitch_angle) << " " << angle2rad(cur_shoulder_row_angle) << " " << angle2rad(cur_elbow_angle)
		           << " " << angle2rad(goals[0]) << " " << angle2rad(goals[1]) << " " << angle2rad(goals[2]) << " "
		           << gyro_x << " " << gyro_y << " " << gyro_z << " " << accel_x << " " << accel_y << " " << accel_z << " " << current_time << std::endl;

		num_steps ++;

		usleep(30000 - (current_time_microsec - initial_time_microsec));
		initial_time_microsec = current_time_microsec;
	}

	fileStream.close();

	return 0;
}
