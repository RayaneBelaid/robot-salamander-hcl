/**
 ****************************************************************************
 * KM-Robota main to test library
 ****************************************************************************
 * @file        main.cpp
 * @brief       Main program body
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coactivate_motor3y and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ****************************************************************************
 */


#include <iostream>
#include <string>
#include <unistd.h>  // Provides sleep function for linux
#include <ctime>
#include <vector>

#include "robot.hpp"


//#include "control_table_maps.hpp"


#define BAUDRATE    1000000
#define NBR_MOTORS  5
#define PI          3.14

using namespace std;

vector<float> fbck_angles(NBR_MOTORS, 0);
vector<float> fbck_leds(NBR_MOTORS);
vector<float> fbck_enabled(NBR_MOTORS);
vector<float> goal_angles(NBR_MOTORS);
vector<int>   goal_leds(NBR_MOTORS, 0);
std::vector<float> duty_factor = {0.7, 0.7, 0.7, 0.7};
std::vector<float> init_angles = {0, 0, 0, 0, 0};
std::vector<float> phase = {0, 4*PI/3, PI, PI/3};
vector<float> prev_goal_angles(NBR_MOTORS);

// Functions
double get_delta_us(struct timespec t2, struct timespec t1);
timespec time_s();

int main()
{
    // Define some variables
    float goal_angle_a = 0;
    float goal_angle_g = 0;
    int time_per_turn = 2000; // in ms = 1 sec
    int ticks_per_turn = time_per_turn / 10; // in ms
    float offset_g; // step on the ground 70% of time is on the ground
    float offset_a;  // step in the air 30% of time is on air
    int cnt = 0;
    struct timespec loop_start, now, start, stop, prev_loop, prev_reset, curr_reset;
    double elapsed, delta; // in us
    int loop_duration = 10*1000; // in us
    int to_sleep = 0;
    int turnCnt = 1;
    bool stance = 0;
    bool reset = 0;
    bool activate_motor1 = 0;
    bool activate_motor2 = 0;
    bool activate_motor3 = 0;
    bool activate_motor0 = 0;

    // Init start
    KMR::dxlP1::Hal hal;

    char path_to_motor_config[] = "../config/test_motors_config.yaml";
    char path_to_KMR_dxl[] = "../KMR_dxlP1";

    vector<int> all_ids = hal.init(path_to_motor_config, path_to_KMR_dxl);

    cout << endl;
    cout << "List of motor IDs" << endl;
    for (int i=0; i<all_ids.size(); i++)
        cout << all_ids[i] << " ";
    cout << endl;

    // Create robot instance
    Robot robot(all_ids, "/dev/ttyUSB0", BAUDRATE, hal);

    // Start testing
    robot.enableMotors();


    /*
    *****************************************************************************
    *                         Start of testing
    ****************************************************************************/
    cout << "Start of testing" << endl;
    robot.checkMode(all_ids);
    sleep(2);
    robot.enableMotors();

    cout << endl;
    for (int i=0; i<goal_angles.size(); i++){
        goal_angles[i] = init_angles[i];
        prev_goal_angles[i] = goal_angles[i];
        cout << "angles: " << goal_angles[i] << endl;
    }

    robot.writeData(goal_angles, all_ids);

    sleep(5);

     while(turnCnt < 6) {
        loop_start = time_s();
        // Reset necessary motors
        robot.resetMultiturnMotors();

        for (int i=0; i<NBR_MOTORS; i++) {

            if (i == 0) {
                if (goal_angles[i] == phase[0]) {
                activate_motor0 = 1;
                }
                if (goal_angles[i] >= phase[1]) {
                    activate_motor1 = 1;
                }
                if (goal_angles[i] >= phase[2]) {
                    activate_motor2 = 1;
                }
                if (goal_angles[i] >= phase[3]) {
                    activate_motor3 = 1;
                }
                if (prev_goal_angles[i] >= 2*PI/3 && prev_goal_angles[i] <= PI && activate_motor0) {
                        stance = 1;
                    }
                    if (cnt <= ticks_per_turn * duty_factor[i] && stance) {    // STANCE
                        cout << "Stance: "<< endl;
                        // calculate the offset based on the DF
                        offset_a = (PI/3)/(ticks_per_turn*(duty_factor[i]));
                        // update command valuesgoal_angles
                        goal_angle_a = prev_goal_angles[i] + offset_a;
                        goal_angles[i] = goal_angle_a;
                        cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;

                    } else {                                           // SWING
                        cout << "Swing: "<< endl;
                        // calculate the offset basedoffset_a on the DF
                        offset_g = (5*PI/3)/(ticks_per_turn*(1-duty_factor[i]));  // calculate the offset
                        // update command values
                        goal_angle_g = prev_goal_angles[i] + offset_g;
                        goal_angles[i] = goal_angle_g;
                        cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;
                    }
                    stance = 0;

            } else if (i == 1 && activate_motor1) {
                if (prev_goal_angles[i] >= 2*PI/3 && prev_goal_angles[i] <= PI) {
                    stance = 1;
                }
                if (cnt <= ticks_per_turn * duty_factor[i] && stance) {    // STANCE
                    cout << "Stance: "<< endl;
                    // calculate the offset based on the DF
                    offset_a = (PI/3)/(ticks_per_turn*(duty_factor[i]));
                    // update command valuesgoal_angles
                    goal_angle_a = prev_goal_angles[i] + offset_a;
                    goal_angles[i] = goal_angle_a;
                    cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;

                } else {                                           // SWING
                    cout << "Swing: "<< endl;
                    // calculate the offset basedoffset_a on the DF
                    offset_g = (5*PI/3)/(ticks_per_turn*(1-duty_factor[i]));  // calculate the offset
                    // update command values
                    goal_angle_g = prev_goal_angles[i] + offset_g;
                    goal_angles[i] = goal_angle_g;
                    cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;
                }
                stance = 0;

            } else if (i == 2 && activate_motor2) {
                 if (prev_goal_angles[i] <= -2*PI/3 && prev_goal_angles[i] >= -PI) {
                    stance = 1;
                }
                if (cnt <= ticks_per_turn * duty_factor[i] && stance) {    // STANCE
                    cout << "Stance: "<< endl;
                    // calculate the offset based on the DF
                    offset_a = (-1)*((PI/3)/(ticks_per_turn*(duty_factor[i])));
                    // update command valuesgoal_angles
                    goal_angle_a = prev_goal_angles[i] + offset_a;
                    goal_angles[i] = goal_angle_a;
                    cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;

                } else {                                           // SWING
                    cout << "Swing: "<< endl;
                    // calculate the offset basedoffset_a on the DF
                    offset_g = (-1)*((5*PI/3)/(ticks_per_turn*(1-duty_factor[i])));  // calculate the offset
                    // update command values
                    goal_angle_g = prev_goal_angles[i] + offset_g;
                    goal_angles[i] = goal_angle_g;
                    cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;
                }
                stance = 0;

            } else if (i == 3 && activate_motor3) {
                if (prev_goal_angles[i] <= -2*PI/3 && prev_goal_angles[i] >= -PI) {
                    stance = 1;
                }
                if (cnt <= ticks_per_turn * duty_factor[i] && stance) {    // STANCE
                    cout << "Stance: "<< endl;
                    // calculate the offset based on the DF
                    offset_a = (-1)*((PI/3)/(ticks_per_turn*(duty_factor[i])));
                    // update command valuesgoal_angles
                    goal_angle_a = prev_goal_angles[i] + offset_a;
                    goal_angles[i] = goal_angle_a;
                    cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;

                } else {                                           // SWING
                    cout << "Swing: "<< endl;
                    // calculate the offset basedoffset_a on the DF
                    offset_g = (-1)*((5*PI/3)/(ticks_per_turn*(1-duty_factor[i])));  // calculate the offset
                    // update command values
                    goal_angle_g = prev_goal_angles[i] + offset_g;
                    goal_angles[i] = goal_angle_g;
                    cout << " goal angles " << i << " : " << goal_angles[i] << "" << endl;
                }
                stance = 0;
            }
        }

        for (int i=0; i<NBR_MOTORS; i++) {
            if (goal_angles[i] > (2 * PI+0.02)) {
                reset = 1;
            } else if (goal_angles[i] < -(2*PI+0.02)) {
                reset = 1;
            }
        }


         for (int i=0; i<NBR_MOTORS; i++) {
            cout << " before writing - goal_angles " << i << " : " << goal_angles[i] << "" << endl;
        }

        robot.writeData(goal_angles, all_ids);


        for (int i=0; i<NBR_MOTORS; i++) {
            if (goal_angles[i] > (2 * PI+0.02) && reset) {
                goal_angles[i] -= (2 * PI+0.02);
                cout << " reset goal_angles " << i << " : " << goal_angles[i] << "" << endl;
            } else if (goal_angles[i] < -(2 * PI+0.02) && reset) {
                goal_angles[i] += (2 * PI+0.02);
                cout << " reset goal_angles " << i << " : " << goal_angles[i] << "" << endl;
            }
        }


        for (int i=0; i<NBR_MOTORS; i++) {
            prev_goal_angles[i] = goal_angles[i];

        }

        reset = 0;
        /*
        if (cnt > ticks_per_turn+1){
            cout << "resetting count" << endl;
            activate_motor0 = 0;
            activate_motor1 = 0;
            activate_motor2 = 0;
            activate_motor3 = 0;
            cnt = 0;
            turnCnt++;
        }*/

        // Time: we want to 10ms control loop
        now = time_s();
        prev_loop = loop_start;
        elapsed = get_delta_us(now, loop_start);
        to_sleep = loop_duration - elapsed;

        if (to_sleep > 0)
            usleep (to_sleep);
        else {
            cout << "Loop too long" << endl;
        }

    }

    robot.disableMotors();

}

timespec time_s()
{
    struct timespec real_time;

    if (clock_gettime(CLOCK_REALTIME, &real_time) == -1 )
    {
        perror("clock gettime");
        exit( EXIT_FAILURE );
    }

    return real_time;
}

double get_delta_us(struct timespec t2, struct timespec t1)
{
    struct timespec td;
    td.tv_nsec = t2.tv_nsec - t1.tv_nsec;
    td.tv_sec  = t2.tv_sec - t1.tv_sec;
    if (td.tv_sec > 0 && td.tv_nsec < 0)
    {
        td.tv_nsec += 1000000000;
        td.tv_sec--;
    }

    return(td.tv_sec*1000000 + td.tv_nsec/1000);
}

