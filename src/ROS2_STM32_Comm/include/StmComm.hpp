#pragma once

#include <iostream>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

constexpr bool COMM_OK = false;
constexpr bool COMM_ERROR = true;
constexpr bool TRIG_ENABLED = true;
constexpr bool TRIG_DISABLED = false;

namespace STComm {
    using namespace LibSerial;
    enum DeviceCommands : uint8_t {
        PING = 0x00,
        STATUS = 0x01,
    
    };
    
    enum Commands : uint16_t  {
        GET_NEXT = 0x0000,
        SET_TRIG_FREQ = 0x0001,
        START_TRIG = 0x0002,
        STOP_TRIG = 0x0003,
        GET_TRIG_FREQ = 0x0004,
        GET_TRIG_STATUS = 0x0005,
    };
    
    enum Status : uint8_t {
        STATUS_READY = 0b00000000,
        STATUS_BUSY = 0b00000001,
        STATUS_DATA_NEXT = 0b00000010,
        STATUS_CMD_ERROR = 0b01000000,
        STATUS_ERROR = 0b10000000,
    };
    
    struct SerialResponse {
        uint8_t status;
        uint8_t data[8];
    };

    // Raw commands
    class SerialComm {
        private:
            SerialPort* mPort;
        public:
            SerialComm(std::string busPath);
            ~SerialComm();
            SerialResponse* SendDeviceCommand(DeviceCommands c);
            SerialResponse* SendCommand(Commands c, uint16_t arg = 0);
            static SerialResponse* VecToSerialResponse(const std::vector<uint8_t>& v);
    };

    void PrintResponse(SerialResponse r);
}