#pragma once
#include <deque>
#include <optional>

class Axle
{

public:
    enum RemoteCommand : uint8_t
    {
        CmdNOP = 0,
        CmdPowerOff = 1,
        CmdDisableMotors = 2,
        CmdEnableMotors = 3,
        CmdSetCurrentLimit = 10, // flag field = current limit in 1/10th Amps (0-255, e.g., 45 = 4.5A)
        CmdSetSpeedLimit = 11,   // flag field = speed limit in RPM / 4 (0-255, e.g., 250 = 1000 RPM)
        CmdBeep = 12
    };

    static constexpr uint8_t encodeFlags(uint8_t beepFreq, uint8_t flg4bit) {
        return  (beepFreq << 4 ) | (flg4bit & 0x0f);
    };
    static constexpr uint8_t FLG_STANDSTILL_FORCE_L = 0x01;
    static constexpr uint8_t FLG_STANDSTILL_FORCE_R = 0x02;

    struct SerialCommand 
    {
        uint16_t start;
        int16_t  motR;
        int16_t  motL;
        uint8_t  flags;
        uint8_t  cmd;
        uint8_t  payload;
        uint16_t checksum;
    } __attribute__((packed));

    struct SerialFeedback
    {
        uint16_t start;
        int16_t  cmdR;
        int16_t  cmdL;
        int16_t  speedL_meas;
        int16_t  speedR_meas;
        int16_t  batVoltage;
        int16_t  boardTemp;
        int16_t  currL_meas;
        int16_t  currR_meas;
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
    static constexpr unsigned long HoverSerialBaud = 115200;
    static constexpr uint16_t StartFrame = 0xABCD;
    static constexpr uint8_t BufferLenRecv = sizeof(SerialFeedback) * 2 - 1;

    

    Axle(uart_port_t hwSerialNum, uint8_t pinRX, uint8_t pinTX);
    void Shutdown();
    bool Send(int16_t motL, int16_t motR, uint8_t flags, Axle::RemoteCommand cmd = Axle::RemoteCommand::CmdNOP, uint8_t payload = 0);
    bool WaitForFeedback(HistoryFrame& out, TickType_t timeout);
    MotorStates GetLatestFeedback();
    MotorStates GetLatestFeedback(uint32_t currTime);

protected:
    uart_port_t conn;
    byte recvBuffer[BufferLenRecv];
    int recvCap = 0;
    QueueHandle_t feedbackQueue;  // size = 1
    QueueHandle_t commandQueue;   // size = 1
    TaskHandle_t receiverTask = NULL;
    TaskHandle_t senderTask = NULL;

    std::deque<HistoryFrame> historyBuffer;
    void SendEventHandler();
    uint8_t ProcessFeedbackFrame(uint8_t* buffer, size_t len);
    bool PushFeedback(const SerialFeedback& fb);
    void ReadTask();
};