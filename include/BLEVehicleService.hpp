#ifndef BLE_VEHICLE_SERVICE_HPP
#define BLE_VEHICLE_SERVICE_HPP

// M1: left wheel, M2: right wheel
#define LEFT_WHEEL_MOTOR_CH 0
#define RIGHT_WHEEL_MOTOR_CH 1

#include <ArduinoBLE.h>
#include <M5Unified.h>
#include <M5AtomicMotion.h>
#include <stdlib.h>
#include <string.h>

namespace ble
{
    // BLE service for the MicroBlue app (https://play.google.com/store/apps/details?id=com.snappyxo.microblue).
    //
    // MicroBlue writes an ASCII text to the characteristic: "d<k><x>,<y>"
    //   k: widget slot (0: d0 joystick/d-pad, 1: d1)
    //   x, y: 0-1023, 512 is the center
    // e.g. "d0512,512"
    //
    // Protocol reference: https://zenn.dev/takaki_maeda_99/articles/5c9152dfa748bb
    class BLEVehicleService : public BLEService
    {
    private:
        static const int MAX_SLOTS = 2;
        static const int PAYLOAD_SIZE = 32;
        static const int AXIS_CENTER = 512;
        static const int AXIS_DEADZONE = 40;   // ignore small stick tilt
        static const int MAX_MOTOR_SPEED = 127; // int8_t range of M5AtomicMotion

        int16_t axes_[MAX_SLOTS][2] = {{AXIS_CENTER, AXIS_CENTER}, {AXIS_CENTER, AXIS_CENTER}};
        int8_t left_target_speed_ = 0;
        int8_t right_target_speed_ = 0;

        // convert 0-1023 axis value to -1.0 .. 1.0 with dead zone
        static float normalizeAxis(int raw)
        {
            int v = raw - AXIS_CENTER;
            if (abs(v) < AXIS_DEADZONE)
            {
                return 0.0f;
            }
            float f = (float)v / (float)AXIS_CENTER;
            return constrain(f, -1.0f, 1.0f);
        }

        // parse "d<k><x>,<y>"
        bool parse(const uint8_t *data, int len)
        {
            char buf[PAYLOAD_SIZE + 1];
            int n = 0;
            for (int i = 0; i < len && n < PAYLOAD_SIZE; i++)
            {
                if (data[i] >= 0x20) // drop control bytes (0x01-0x03 etc.)
                {
                    buf[n++] = (char)data[i];
                }
            }
            buf[n] = '\0';

            // buf[0]: prefix 'd', buf[1]: slot index, buf[2..]: "x,y"
            if (n < 5 || buf[1] < '0' || buf[1] >= '0' + MAX_SLOTS)
            {
                return false;
            }
            const char *comma = strchr(buf + 2, ',');
            if (comma == nullptr)
            {
                return false;
            }
            int k = buf[1] - '0';
            axes_[k][0] = atoi(buf + 2);
            axes_[k][1] = atoi(comma + 1);
            return true;
        }

    public:
        BLECharacteristic control_char;

        BLEVehicleService()
            : BLEService("12345678-0000-0000-0000-123456789abc"),
              control_char("12345678-0000-0000-0000-abcdef123456",
                           BLERead | BLEWrite | BLEWriteWithoutResponse | BLENotify,
                           PAYLOAD_SIZE)
        {
            addCharacteristic(control_char);
        }

        // returns 1 if a new command was received
        int poll(M5AtomicMotion &motion, float p_gain)
        {
            int8_t left_current_speed = motion.getMotorSpeed(LEFT_WHEEL_MOTOR_CH);
            int8_t right_current_speed = motion.getMotorSpeed(RIGHT_WHEEL_MOTOR_CH);

            int is_written = 0;
            if (control_char.written())
            {
                uint8_t data[PAYLOAD_SIZE];
                int len = control_char.readValue(data, PAYLOAD_SIZE);
                if (parse(data, len))
                {
                    // d0: x = turn (right is positive), y = forward (up is positive)
                    // NOTE: flip signs here if the axes feel reversed on the real device.
                    float turn = normalizeAxis(axes_[0][0]);
                    float forward = normalizeAxis(axes_[0][1]);
                    float left = constrain(forward + turn, -1.0f, 1.0f);
                    float right = constrain(forward - turn, -1.0f, 1.0f);
                    left_target_speed_ = (int8_t)(left * MAX_MOTOR_SPEED);
                    right_target_speed_ = (int8_t)(right * MAX_MOTOR_SPEED);
                    is_written = 1;
                }
            }

            motion.setMotorSpeed(LEFT_WHEEL_MOTOR_CH,
                                 left_current_speed + (int8_t)(p_gain * (left_target_speed_ - left_current_speed)));
            motion.setMotorSpeed(RIGHT_WHEEL_MOTOR_CH,
                                 right_current_speed + (int8_t)(p_gain * (right_target_speed_ - right_current_speed)));

            return is_written;
        }

        int16_t axisX(int slot = 0) const { return axes_[slot][0]; }
        int16_t axisY(int slot = 0) const { return axes_[slot][1]; }
    };

} // namespace ble

#endif
