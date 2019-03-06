/*
 *   LinuxMotionTimer.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include "MotionModule.h"
#include "LinuxMotionTimer.h"

#include <stdlib.h>
#include <string.h>

using namespace Robot;

LinuxMotionTimer::LinuxMotionTimer(MotionManager* manager)
    : m_Manager(manager)
{
    this->m_FinishTimer = false;
    this->m_TimerRunning = false;
}

void *LinuxMotionTimer::TimerProc(void *param)
{
    LinuxMotionTimer *timer = (LinuxMotionTimer *)param;
    static struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC,&next_time);

    int interv[50];
    int count = 0;
    time_t prev_time = -1;
    while(!timer->m_FinishTimer)
    {
        if (prev_time == -1)
            prev_time = time();
        else
        {
            v = time() - prev_time;
            prev_time = time();
            interv[count] = v;
            count += 1;
            if (count >= 50) {
                count = 0;
                int avg = 0;
                for (int i = 0; i < 50; i++)
                    avg += interv[i];
                std::cout << avg * 1.0 / 50 << std::endl;
            }
        }
        next_time.tv_sec += (next_time.tv_nsec + MotionModule::TIME_UNIT * 1000000) / 1000000000;
        next_time.tv_nsec = (next_time.tv_nsec + MotionModule::TIME_UNIT * 1000000) % 1000000000;

        if(timer->m_Manager != NULL)
            timer->m_Manager->Process();

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    }

    pthread_exit(NULL);
}

void LinuxMotionTimer::Start(void)
{
    int error;
    struct sched_param param;
    pthread_attr_t attr;

    pthread_attr_init(&attr);

    error = pthread_attr_setschedpolicy(&attr, SCHED_RR);
    if(error != 0)
        printf("error = %d\n",error);
    error = pthread_attr_setinheritsched(&attr,PTHREAD_EXPLICIT_SCHED);
    if(error != 0)
        printf("error = %d\n",error);

    memset(&param, 0, sizeof(param));
    param.sched_priority = 31;// RT
    error = pthread_attr_setschedparam(&attr, &param);
    if(error != 0)
        printf("error = %d\n",error);

    // create and start the thread
    if((error = pthread_create(&this->m_Thread, &attr, this->TimerProc, this))!= 0)
        exit(-1);

    this->m_TimerRunning=true;

}

void LinuxMotionTimer::Stop(void)
{
    int error=0;

    // seti the flag to end the thread
    if(this->m_TimerRunning)
    {
        this->m_FinishTimer = true;
        // wait for the thread to end
        if((error = pthread_join(this->m_Thread, NULL))!= 0)
            exit(-1);
        this->m_FinishTimer = false;
        this->m_TimerRunning = false;
    }
}

bool LinuxMotionTimer::IsRunning(void)
{
    return this->m_TimerRunning;
}

LinuxMotionTimer::~LinuxMotionTimer()
{
    this->Stop();
    this->m_Manager = NULL;
}
