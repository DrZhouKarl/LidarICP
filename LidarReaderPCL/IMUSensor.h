#pragma once

#define ASIO_STANDALONE

#include <msp_msg.hpp>
#include "MSP.hpp"

class IMUSensor
{
public:
    IMUSensor();
    ~IMUSensor();

    msp::msg::Attitude pull();

    msp::msg::Attitude getAttitude();
    std::chrono::microseconds getStamp();
private:

    msp::MSP msp;
    std::mutex m_attitudeMutex;
    msp::msg::Attitude attitude;
    std::chrono::microseconds m_stamp;
    const float acc_1g;
    const float gyro_unit;
    const float magn_gain;
    const float si_unit_1g;
};
