//
// Parham Nooralishahi - PHM66
// @ 2024
//

#ifndef PHM_WITMOTION_SERIAL_H
#define PHM_WITMOTION_SERIAL_H

#include <string>

typedef int FileHandler;
typedef unsigned int Baudrate;

namespace phm::witmotion {
    class Serial {
    private:
        std::string devFile;
        Baudrate baudrate;
        FileHandler devHandler;
    private:
        Serial(std::string &, Baudrate);
    public:
        Serial() = delete;
        ~Serial();
        std::string getDevFile() {return devFile;}
        [[nodiscard]] Baudrate getBaudrate() const {return baudrate;}

        void begin();
        void end();
        int receive(uint8_t *, size_t length);
        int send(uint8_t *, size_t length);
        [[nodiscard]] bool isOpened() const { return devHandler >= 0; }

    public: // Static methods
        static Serial * build(std::string &, Baudrate baud);
    };

    typedef Serial * pSerial;
}

#endif //PHM_WITMOTION_SERIAL_H
