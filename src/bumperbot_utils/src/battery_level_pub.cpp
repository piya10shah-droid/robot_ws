#include <iostream>
#include <iomanip>
#include <cstdint>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define REG_CONFIG         0x00
#define REG_SHUNTVOLTAGE   0x01
#define REG_BUSVOLTAGE     0x02
#define REG_POWER          0x03
#define REG_CURRENT        0x04
#define REG_CALIBRATION    0x05

class INA219 {
private:
    int file;
    uint8_t addr;

    uint16_t cal_value;
    float current_lsb;
    float power_lsb;

public:
    INA219(const char* i2c_device = "/dev/i2c-7", uint8_t address = 0x40) {
        addr = address;

        file = open(i2c_device, O_RDWR);
        if (file < 0) {
            perror("Failed to open I2C bus");
            exit(1);
        }

        if (ioctl(file, I2C_SLAVE, addr) < 0) {
            perror("Failed to connect to INA219");
            exit(1);
        }

        setCalibration16V5A();
    }

    ~INA219() {
        close(file);
    }

    uint16_t readRegister(uint8_t reg) {
        uint8_t buffer[2];

        if (write(file, &reg, 1) != 1) {
            perror("Failed to write register address");
        }

        if (read(file, buffer, 2) != 2) {
            perror("Failed to read register");
        }

        return (buffer[0] << 8) | buffer[1];
    }

    void writeRegister(uint8_t reg, uint16_t value) {
        uint8_t buffer[3];

        buffer[0] = reg;
        buffer[1] = (value >> 8) & 0xFF;
        buffer[2] = value & 0xFF;

        if (write(file, buffer, 3) != 3) {
            perror("Failed to write register");
        }
    }

    void setCalibration16V5A() {
        current_lsb = 0.1524f;
        cal_value = 26868;
        power_lsb = 0.003048f;

        writeRegister(REG_CALIBRATION, cal_value);

        uint16_t config =
            (0x00 << 13) |   // 16V range
            (0x01 << 11) |   // Gain /2, 80mV
            (0x0D << 7)  |   // 12-bit ADC, 32 samples
            (0x0D << 3)  |   // 12-bit ADC, 32 samples
            (0x07);          // Continuous mode

        writeRegister(REG_CONFIG, config);
    }

    float getShuntVoltage_mV() {
        int16_t value = (int16_t)readRegister(REG_SHUNTVOLTAGE);
        return value * 0.01f;
    }

    float getBusVoltage_V() {
        uint16_t value = readRegister(REG_BUSVOLTAGE);
        return ((value >> 3) * 0.004f);
    }

    float getCurrent_mA() {
        int16_t value = (int16_t)readRegister(REG_CURRENT);
        return value * current_lsb;
    }

    float getPower_W() {
        int16_t value = (int16_t)readRegister(REG_POWER);
        return value * power_lsb;
    }
};

int main() {
    INA219 ina219("/dev/i2c-7", 0x41);

    while (true) {
        float bus_voltage = ina219.getBusVoltage_V();
        float shunt_voltage = ina219.getShuntVoltage_mV() / 1000.0f;
        float current = ina219.getCurrent_mA();
        float power = ina219.getPower_W();

        float percentage = (bus_voltage - 9.0f) / 3.6f * 100.0f;

        if (percentage < 0)
            percentage = 0;

        if (percentage > 100)
            percentage = 100;

        std::cout << std::fixed << std::setprecision(3);

        std::cout << "Load Voltage:  "
                  << bus_voltage
                  << " V" << std::endl;

        std::cout << "Current:       "
                  << current / 1000.0f
                  << " A" << std::endl;

        std::cout << "Power:         "
                  << power
                  << " W" << std::endl;

        std::cout << "Percentage:    "
                  << std::setprecision(2)
                  << percentage
                  << " %" << std::endl;

        std::cout << "-----------------------------"
                  << std::endl;

        sleep(2);
    }

    return 0;
}