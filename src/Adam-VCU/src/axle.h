#pragma once

#include <Arduino.h>
#include <deque>
#include <optional>

class Axle
{

public:
    struct SerialCommand 
    {
        uint16_t start;
        int16_t  steer;
        int16_t  speed;
        uint16_t checksum;
    } __attribute__((packed));

    struct SerialFeedback
    {
        uint16_t start;
        int16_t  cmd1;
        int16_t  cmd2;
        int16_t  speedR_meas;
        int16_t  speedL_meas;
        int16_t  batVoltage;
        int16_t  boardTemp;
        uint16_t cmdLed;
        uint16_t checksum;
    } __attribute__((packed));

    struct HistoryFrame
    {
        uint32_t TimeStamp = 0;
        SerialFeedback sample;
    };

    struct MotorCommand {
        int16_t motL;
        int16_t motR;
    };

    static constexpr unsigned long HoverSerialBaud = 115200;
    static constexpr uint16_t StartFrame = 0xABCD;
    static constexpr uint8_t BufferLenRecv = sizeof(SerialFeedback) * 2 - 1;

    

    Axle(uart_port_t hwSerialNum, uint8_t pinRX, uint8_t pinTX);
    bool Send(int16_t motL, int16_t motR);
    bool WaitForFeedback(HistoryFrame& out, TickType_t timeout);

protected:
    uart_port_t conn;
    byte recvBuffer[BufferLenRecv];
    uint8_t recvCap = 0;
    QueueHandle_t feedbackQueue;  // size = 1
    QueueHandle_t commandQueue;   // size = 1

    std::deque<HistoryFrame> historyBuffer;
    void SendEventHandler();
    uint8_t ProcessFeedbackFrame(uint8_t* buffer, size_t len);
    bool PushFeedback(const SerialFeedback& fb);
    void ReadTask();
    void SendInternal(int16_t motL, int16_t motR);
};