#include "axle.h"
#include "hwconfig.h"
#include "driver/uart.h"
#include "swconfig.h"

Axle::Axle(uart_port_t hwSerialNum, uint8_t pinRX, uint8_t pinTX):
conn(hwSerialNum)
{

     // Create size-1 queues (for overwrite semantics)
    feedbackQueue = xQueueCreate(1, sizeof(HistoryFrame));
    configASSERT(feedbackQueue != nullptr);

    commandQueue = xQueueCreate(1, sizeof(MotorCommand));
    configASSERT(commandQueue != nullptr);


     // 1. Configure the UART
    uart_config_t uart_config = {
        .baud_rate = HoverSerialBaud,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    // 2. Install driver and set pins
    // Using a 1024-byte ring buffer for RX
    uart_driver_install(conn, 1024 * 2, 0, 0, NULL, 0);
    uart_param_config(conn, &uart_config);
    uart_set_pin(conn, pinTX, pinRX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

      xTaskCreatePinnedToCore(
        [](void* pvParameters) {
            static_cast<Axle*>(pvParameters)->SendEventHandler();
            vTaskDelete(NULL);
        },
        "AxleW",
        SWConfig::Tasks::MinStakSize,
        this,
        SWConfig::Tasks::PrioHigh,
        NULL,
        SWConfig::CoreAffinity::CoreComms
    );

     xTaskCreatePinnedToCore(
        [](void* pvParameters) {
            static_cast<Axle*>(pvParameters)->ReadTask();
            vTaskDelete(NULL);
        },
        "AxleR",
        SWConfig::Tasks::MinStakSize,
        this,
        SWConfig::Tasks::PrioHigh,
        NULL,
        SWConfig::CoreAffinity::CoreComms
    );

}


void Axle::SendInternal(int16_t motL, int16_t motR, RemoteCommand remoteCmd)
{
    SerialCommand command;
    // Create command
    command.start    = Axle::StartFrame;
    command.steer    = motL;
    command.speed    = motR;
    command.cmd      = remoteCmd;
    command.checksum = (uint16_t)(command.start ^ command.steer ^ command.speed);

    // Write to Serial
    uart_write_bytes(conn, reinterpret_cast<const char*>(&command), sizeof(SerialCommand));
}


// Push latest command into size-1 queue
bool Axle::Send(int16_t motL, int16_t motR, RemoteCommand func)
{
    MotorCommand cmd{motL, motR, func};
    // Overwrite last command, never blocks
    BaseType_t res = xQueueOverwrite(commandQueue, &cmd);
    return (res == pdPASS);
}

bool Axle::WaitForFeedback(HistoryFrame& out, TickType_t timeout)
{
    return (xQueueReceive(feedbackQueue, &out, timeout) == pdTRUE);
}

Axle::MotorStates Axle::GetLatestFeedback()
{
    HistoryFrame frame;
    if (xQueuePeek(feedbackQueue, &frame, 0) == pdTRUE) {
        return frame;                // copy into optional
    }
    return std::nullopt;
}

Axle::MotorStates Axle::GetLatestFeedback(uint32_t currTime)
{
    auto temp = GetLatestFeedback();
    if (temp.has_value()) {
        temp->MarkStale(currTime);
    }
    return temp;
}

bool Axle::PushFeedback(const SerialFeedback& fb)
{
    HistoryFrame frame;
    frame.sample    = fb;
    frame.TimeStamp = millis();

    // Latest-wins semantics; unblocks any task waiting on receive
    return (xQueueOverwrite(feedbackQueue, &frame) == pdPASS);
}

uint8_t Axle::ProcessFeedbackFrame(uint8_t* buffer, size_t len)
{
    size_t skipped = 0;

    while (len >= sizeof(SerialFeedback)) {
        SerialFeedback packet;
        memcpy(&packet, buffer, sizeof(SerialFeedback));

        uint16_t checksum = static_cast<uint16_t>(
            packet.start ^
            packet.cmd1 ^
            packet.cmd2 ^
            packet.speedR_meas ^
            packet.speedL_meas ^
            packet.batVoltage ^
            packet.boardTemp ^
            packet.currL_meas ^
            packet.currR_meas
        );

        if (packet.start == StartFrame && checksum == packet.checksum) {
            PushFeedback(packet);  // just hand it to the queue
            return static_cast<uint8_t>(skipped + sizeof(SerialFeedback));
        }

        buffer++;
        skipped++;
        len--;
    }

    return 0;
}

void Axle::ReadTask()
{
    for (;;) {

        // If buffer is full and we still didn't find a frame, drop 1 byte (oldest)
        // to make room and keep a sliding window of the last BufferLenRecv bytes.
        if (recvCap == BufferLenRecv) {
            memmove(recvBuffer, recvBuffer + 1, BufferLenRecv - 1);
            recvCap -= 1;
        }

        // Read directly into the free space at the end of recvBuffer
        int len = uart_read_bytes(conn,
                                  recvBuffer + recvCap,
                                  BufferLenRecv - recvCap,
                                  portMAX_DELAY);

        if (len > 0) {
            recvCap += len;

            // At most one full frame can exist in the buffer by construction
            uint8_t consumed = ProcessFeedbackFrame(recvBuffer, recvCap);

            if (consumed > 0) {
                // Drop everything up to and including the found frame
                size_t remaining = recvCap - consumed;
                if (remaining > 0) {
                    memmove(recvBuffer, recvBuffer + consumed, remaining);
                }
                recvCap = remaining;
            }
        }
    }
}

void Axle::SendEventHandler()
{
    MotorCommand cmd;

    for (;;) {
        // Get latest command if available, otherwise keep previous or default
        if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            SendInternal(cmd.motL, cmd.motR, cmd.func);
        }
        
    }
}