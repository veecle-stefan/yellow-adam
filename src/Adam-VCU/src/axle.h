#pragma once

#include <Arduino.h>
#include <deque>
#include <optional>

class Axle
{

public:
    enum RemoteCommand : uint8_t {
      CmdNOP = 0,
      CmdBeep = 1,
      CmdPowerOff = 2
    };

    struct SerialCommand 
    {
        uint16_t start;
        int16_t  steer;
        int16_t  speed;
        uint8_t  cmd;
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
        int16_t  currR_meas;
        int16_t  currL_meas;
        uint16_t checksum;
    } __attribute__((packed));

    struct HistoryFrame
    {
        static constexpr uint32_t StaleTimeout = 200; // older values than 200ms are ignored

        uint32_t TimeStamp = 0;
        bool isStale = false;
        SerialFeedback sample;

        void MarkStale(uint32_t currTime) {
            this->isStale = (currTime - this->TimeStamp) > StaleTimeout;
        }
    };

    typedef std::optional<HistoryFrame> MotorStates;

    struct MotorCommand {
        int16_t motL;
        int16_t motR;
        RemoteCommand func = RemoteCommand::CmdNOP;
    };

    static constexpr unsigned long HoverSerialBaud = 115200;
    static constexpr uint16_t StartFrame = 0xABCD;
    static constexpr uint8_t BufferLenRecv = sizeof(SerialFeedback) * 2 - 1;

    

    Axle(uart_port_t hwSerialNum, uint8_t pinRX, uint8_t pinTX);
    void Shutdown();
    bool Send(int16_t motL, int16_t motR, RemoteCommand func = CmdNOP);
    bool WaitForFeedback(HistoryFrame& out, TickType_t timeout);
    MotorStates GetLatestFeedback();
    MotorStates GetLatestFeedback(uint32_t currTime);

protected:
    uart_port_t conn;
    byte recvBuffer[BufferLenRecv];
    uint8_t recvCap = 0;
    QueueHandle_t feedbackQueue;  // size = 1
    QueueHandle_t commandQueue;   // size = 1
    TaskHandle_t receiverTask = NULL;
    TaskHandle_t senderTask = NULL;

    std::deque<HistoryFrame> historyBuffer;
    void SendEventHandler();
    uint8_t ProcessFeedbackFrame(uint8_t* buffer, size_t len);
    bool PushFeedback(const SerialFeedback& fb);
    void ReadTask();
    void SendInternal(int16_t motL, int16_t motR, RemoteCommand remoteCmd);
};