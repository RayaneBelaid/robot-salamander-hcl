/**
 * KM-Robota library
 ******************************************************************************
 * @file            robot.hpp
 * @brief           
 ******************************************************************************
 * @copyright
 * Copyright 2021-2023 Laura Paez Coy and Kamilo Melo                    \n
 * This code is under MIT licence: https://opensource.org/licenses/MIT
 * @authors  Laura.Paez@KM-RoBota.com, 04-2023
 * @authors  Kamilo.Melo@KM-RoBota.com, 05-2023
 ******************************************************************************
 */

#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "KMR_dxlP1_robot.hpp"


class Robot : public KMR::dxlP1::BaseRobot {
private:
    KMR::dxlP1::Writer *m_writer;
    KMR::dxlP1::Writer *m_led_writer;
    KMR::dxlP1::Reader *m_reader;
    KMR::dxlP1::Reader *m_enabled_reader;
    KMR::dxlP1::Reader *m_led_reader;

    KMR::dxlP1::Reader *m_CW_reader;

public:
    Robot(std::vector<int> all_ids, const char *port_name, int baudrate, KMR::dxlP1::Hal hal);
    void writeData(std::vector<float> angles, std::vector<int> ids);
    void readData(std::vector<int> ids, std::vector<float>& fbck_angles);
    void writeLEDs(std::vector<int> goal_leds, std::vector<int> ids);
    void readEnabled(std::vector<int> ids, std::vector<float>& fbck_enabled);
    void readLEDs(std::vector<int> ids, std::vector<float>& fbck_leds);

    void checkMode(std::vector<int> ids);
};


#endif