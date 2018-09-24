#include "stdafx.h"
#include "IMUSensor.h"
#include "MSP.hpp"


IMUSensor::IMUSensor()
    : msp("COM6", 115200)
      , acc_1g(512.0)
      , gyro_unit(1.0 / 4.096)
      , magn_gain(0.92f / 10.0f)
      , si_unit_1g(9.80665f)
{
    msp.setWait(5);
    
    std::thread t1([&]()
        {
            while (true)
            {
                this->m_attitudeMutex.lock();
                this->msp.request_block(this->attitude);
                //m_stamp = std::chrono::duration_cast<std::chrono::microseconds>(
                //    std::chrono::system_clock::now().time_since_epoch());
                this->m_attitudeMutex.unlock();
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        });
    t1.detach();
}

msp::msg::Attitude IMUSensor::pull()
{
    this->msp.request_block(attitude);
    return attitude;
}

msp::msg::Attitude IMUSensor::getAttitude()
{
    std::unique_lock<std::mutex> lock(m_attitudeMutex);
    return attitude;
}

std::chrono::microseconds IMUSensor::getStamp()
{
    return m_stamp;
}

IMUSensor::~IMUSensor()
{
}
