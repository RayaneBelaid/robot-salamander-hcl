/**
 ****************************************************************************
 * KM-Robota robot.cpp to test library
 ****************************************************************************
 * @file        robot.cpp
 * @brief       Main program body
 ****************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ****************************************************************************
 */

#include "robot.hpp"


using namespace std;


/**
 * @brief       Constructor for LibRobot
 * @param[in]   num_motors Number of motors in the robot
 * @param[in]   all_ids List of IDs of all the motors in the robot
 * @param[in]   baudrate Baudrate of the port handling communication with motors
 * @param[in]   port_name Name of the port handling communication with motors
 */
Robot::Robot(vector<int> all_ids, const char *port_name, int baudrate, KMR::dxlP1::Hal hal)
: BaseRobot(all_ids, port_name, baudrate, hal)
{
    // Create handlers
    m_writer = new KMR::dxlP1::Writer(KMR::dxlP1::GOAL_POS, all_ids, portHandler_, packetHandler_, m_hal);
    m_led_writer = new KMR::dxlP1::Writer(KMR::dxlP1::LED, all_ids, portHandler_, packetHandler_, m_hal);
    m_reader = new KMR::dxlP1::Reader(KMR::dxlP1::PRESENT_POS, all_ids, portHandler_, packetHandler_, m_hal);
    m_enabled_reader = new KMR::dxlP1::Reader(KMR::dxlP1::TRQ_ENABLE, all_ids, portHandler_, packetHandler_, m_hal);
    m_led_reader = new KMR::dxlP1::Reader(KMR::dxlP1::LED, all_ids, portHandler_, packetHandler_, m_hal);

    m_CW_reader = new KMR::dxlP1::Reader(KMR::dxlP1::CW_ANGLE_LIMIT, all_ids, portHandler_, packetHandler_, m_hal);

    cout << "Robot instance created" << endl;
}

void Robot::writeData(vector<float> angles, vector<int> ids)
{
    m_writer->addDataToWrite(angles, ids);
    m_writer->syncWrite(ids);
}

void Robot::readData(vector<int> ids, vector<float>& fbck_angles)
{
    m_reader->syncRead(ids);

    for (int i=0; i<ids.size(); i++) {
        fbck_angles[i] = m_reader->m_dataFromMotor[i];
    }

/*     cout << endl;
    cout << "Reading current positions" << endl;
    for (int i=0; i<ids.size(); i++) {
        fbck_angles[i] = m_reader->m_dataFromMotor[i];
        cout << "id: " << ids[i] << " , pos: " << fbck_angles[i] << endl;
    } */

}

void Robot::writeLEDs(vector<int> goal_leds, vector<int> ids)
{
    m_led_writer->addDataToWrite(goal_leds, ids);
    m_led_writer->syncWrite(ids);
}

void Robot::readEnabled(vector<int> ids, vector<float>& fbck_enabled)
{
    cout << endl;
    cout << "read enabled" << endl;
    m_enabled_reader->syncRead(ids);
    for (int i=0; i<ids.size(); i++) {
        fbck_enabled[i] = m_enabled_reader->m_dataFromMotor[i];
        cout << "id: " << ids[i] << " , enabled: " << fbck_enabled[i] << endl;
    }

}

void Robot::readLEDs(vector<int> ids, vector<float>& fbck_leds)
{
    m_led_reader->syncRead(ids);
    for (int i=0; i<ids.size(); i++) {
        fbck_leds[i] = m_led_reader->m_dataFromMotor[i];
    }

}

void Robot::checkMode(vector<int> ids)
{
    double CW_limit;
    int CW_para = 0;
    m_CW_reader->syncRead(ids);
    for (int i=0; i<ids.size(); i++) {
        CW_limit = m_CW_reader->m_dataFromMotor[i];
		CW_para = CW_limit/0.001536 + 4095/2 + 0.5;
        if (CW_para != 4095) {
            cout << "ID " << ids[i] << "not in multiturn!" << endl;
            cout << "Parameter: " << CW_para << endl;            
        }

        else
            cout << "Multiturn ok" << endl;
    }

}