//
// Created by phm on 14/10/24.
//

#ifndef FODCAMERADRIVER_WITMOTION_H
#define FODCAMERADRIVER_WITMOTION_H

#include <cstdint>
#include <unistd.h>
#include <stdexcept>
#include <memory>

#include "witmotion/REG.h"
#include "Serial.h"

#define WIT_HAL_OK      (0)     /**< There is no error */
#define WIT_HAL_BUSY    (-1)    /**< Busy */
#define WIT_HAL_TIMEOUT (-2)    /**< Timed out */
#define WIT_HAL_ERROR   (-3)    /**< A generic error happens */
#define WIT_HAL_NOMEM   (-4)    /**< No memory */
#define WIT_HAL_EMPTY   (-5)    /**< The resource is empty */
#define WIT_HAL_INVAL   (-6)    /**< Invalid argument */
#define WIT_HAL_PASS    (1)     /**< Pass */

#define DATA_BUFF_SIZE  256

//#define WIT_PROTOCOL_NORMAL        0
//#define WIT_PROTOCOL_MODBUS        1
//#define WIT_PROTOCOL_CAN           2
//#define WIT_PROTOCOL_I2C           3
//#define WIT_PROTOCOL_JY61          4
//#define WIT_PROTOCOL_905x_MODBUS   5
//#define WIT_PROTOCOL_905x_CAN      6

typedef enum {
    Normal = 0,
    Modbus = 1,
    CAN = 2,
    I2C = 3,
    JY61 = 4,
    Modbus_905x = 5,
    CAN_905x = 6
} Protocol;

typedef int32_t retcode;
typedef uint8_t Address;

namespace phm::witmotion {

    class NotImplementedError : std::logic_error {
    public:
        NotImplementedError() : std::logic_error("Function not yet implemented") { /* Empty body */ };
        explicit NotImplementedError(std::string & msg) :
            std::logic_error(msg) { /* Empty body */ }
    };

    class Witmotion {
    private:
        Address address = 0xff; // s_ucAddr
        Protocol protocol; // s_uiProtoclo
        uint8_t dataBuffer[DATA_BUFF_SIZE]; // s_ucWitDataBuff
        uint32_t dataCounter = 0; // s_uiWitDataCnt
        uint32_t readRegIndex = 0; // s_uiReadRegIndex
        int16_t regData[REGSIZE]; // sReg
        volatile int8_t dataUpdateFlag = 0; // s_cDataUpdate

        std::shared_ptr<Serial> serialPtr;
    public:
        Witmotion (Protocol, Address);
        ~Witmotion() = default;

        void setSerial (Serial * serial) { serialPtr = std::shared_ptr<Serial>(serial); }
        std::shared_ptr<Serial> getSerial() { return serialPtr; }

        retcode begin();
        retcode step();
        retcode end();
//        void write(uint8_t * p_ucData, uint32_t uiLen);
    private:
        static void deplay(uint16_t timeMs);
        void read_(uint8_t ucData);
        void copeWithData(uint8_t index, uint16_t *data, uint32_t len);
        void processData(uint32_t reg, uint32_t uiRegNum);
        retcode receive();
    };
}

#endif //FODCAMERADRIVER_WITMOTION_H
