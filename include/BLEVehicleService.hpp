#ifndef BLE_VEHICLE_SERVICE_HPP
#define BLE_VEHICLE_SERVICE_HPP

#define BLE_GATT_CPF_UNIT_UNITLESS 0x2700
#define BLE_GATT_CPF_FORMAT_UINT8 0x04
#define BLE_GATT_CPF_FORMAT_UTF8S 0x19

// M1: right wheel, M2: left wheel
#define LEFT_WHEEL_MOTOR_CH 1
#define RIGHT_WHEEL_MOTOR_CH 0

#include <ArduinoBLE.h>
#include <M5Unified.h>
#include <M5AtomicMotion.h>

namespace ble
{
    class BLEVehicleService : public BLEService
    {
    private:
        const uint8_t pwr_format_[7] = {BLE_GATT_CPF_FORMAT_UINT8,
                                        0b0,
                                        (uint8_t)BLE_GATT_CPF_UNIT_UNITLESS,
                                        (uint8_t)(BLE_GATT_CPF_UNIT_UNITLESS >> 8),
                                        0x01,
                                        0x00,
                                        0x00};

        const uint8_t cmd_format_[7] = {BLE_GATT_CPF_FORMAT_UTF8S,
                                        0b0,
                                        (uint8_t)BLE_GATT_CPF_UNIT_UNITLESS,
                                        (uint8_t)(BLE_GATT_CPF_UNIT_UNITLESS >> 8),
                                        0x01,
                                        0x00,
                                        0x00};

    public:
        BLEByteCharacteristic power_char;
        BLECharCharacteristic command_char;

        BLEVehicleService()
            : BLEService("c5408db0-cdfd-46ff-9a31-065f18f1fb4b"),
              command_char("c5408db1-cdfd-46ff-9a31-065f18f1fb4b", BLERead | BLEWrite),
              power_char("c5408db2-cdfd-46ff-9a31-065f18f1fb4b", BLERead | BLEWrite)

        {
            // NOTE: the following Android app sends a char value to the first characteristic.
            // BLE Controller – Arduino ESP32: https://play.google.com/store/apps/details?id=com.circuitmagic.iot
            addCharacteristic(command_char);
            addCharacteristic(power_char);

            // Add user description descriptors
            // F/B/R/L and A/B/X/Y like game pad buttons
            auto command_user_descriptor = BLEDescriptor("2901", "Command");
            command_char.addDescriptor(command_user_descriptor);
            auto power_user_descriptor = BLEDescriptor("2901", "Wheel Power");
            power_char.addDescriptor(power_user_descriptor);

            // Add Format descriptors
            auto command_format_descriptor = BLEDescriptor("2904", cmd_format_, 7);
            command_char.addDescriptor(command_format_descriptor);
            auto power_format_descriptor = BLEDescriptor("2904", pwr_format_, 7);
            power_char.addDescriptor(power_format_descriptor);

            // set initial power value
            power_char.writeValue((uint8_t)32);
        }

        int poll(M5AtomicMotion &motion, float p_gain)
        {

            int8_t left_current_speed = motion.getMotorSpeed(LEFT_WHEEL_MOTOR_CH);
            int8_t right_current_speed = motion.getMotorSpeed(RIGHT_WHEEL_MOTOR_CH);

            int8_t left_target_speed = 0;
            int8_t right_target_speed = 0;

            int is_written = 0;
            if (command_char.written())
            {
                char cmd = command_char.value();
                switch (cmd)
                {
                case 'F': // forward
                case 'f':
                    left_target_speed = power_char.value();
                    right_target_speed = power_char.value();
                    break;
                case 'B': // backward
                case 'b':
                    left_target_speed = -(int8_t)power_char.value();
                    right_target_speed = -(int8_t)power_char.value();
                    break;
                case 'L': // left, ccw turn
                case 'l':
                    left_target_speed = -(int8_t)power_char.value();
                    right_target_speed = power_char.value();
                    break;
                case 'R': // right
                case 'r':
                    left_target_speed = power_char.value();
                    right_target_speed = -(int8_t)power_char.value();
                    break;
                case 'S': // stop
                case 's':
                    right_target_speed = 0;
                    right_target_speed = 0;
                    break;
                }
                is_written = 1;
            }

            motion.setMotorSpeed(LEFT_WHEEL_MOTOR_CH,
                                 left_current_speed + (int8_t)(p_gain * (left_target_speed - left_current_speed)));
            motion.setMotorSpeed(RIGHT_WHEEL_MOTOR_CH,
                                 right_current_speed + (int8_t)(p_gain * (right_target_speed - right_current_speed)));

            return is_written;
        }
    };

} // namespace ble

#endif