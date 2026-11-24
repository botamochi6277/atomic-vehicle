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
    // MicroBlue writes a frame to the characteristic: SOH <type><index> STX <x>,<y> ETX
    //   SOH/STX/ETX: control bytes 0x01/0x02/0x03
    //   type: 'd' (used by both the joystick and the d-pad)
    //   index: number of the MicroBlue control interface in decimal (0: d0, 1: d1)
    //          Each interface can be set to a joystick or a d-pad in the app.
    //   x, y: ASCII decimal 0-1023, 512 is the center
    //         joystick: continuous values, d-pad: 0 / 512 / 1023
    // e.g. 01 'd' '0' 02 "0,512" 03 (d0, x=0, y=512)
    //
    // Protocol reference: https://zenn.dev/takaki_maeda_99/articles/5c9152dfa748bb
    class BLEVehicleService : public BLEService
    {
    private:
        static const uint8_t SOH = 0x01;
        static const uint8_t STX = 0x02;
        static const uint8_t ETX = 0x03;
        static const int MAX_SLOTS = 2;
        static const int PAYLOAD_SIZE = 32;
        static const int AXIS_CENTER = 512;
        static const int AXIS_DEADZONE = 40;   // ignore small stick tilt
        static const int MAX_MOTOR_SPEED = 127; // int8_t range of M5AtomicMotion
        // 0.0: no reduction (sharp turn), 1.0: no turn at full forward. Larger value = gentler curve.
        static constexpr float TURN_REDUCTION_ON_MOVE = 0.7f;
        // backward input (0..1) over which the turn direction changes from normal to inverted
        static constexpr float REVERSE_TURN_BLEND = 0.3f;

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

        // log a received payload as printable text and hex (to investigate MicroBlue widgets)
        static void logRaw(const uint8_t *data, int len)
        {
            char text[PAYLOAD_SIZE + 1];
            char hex[PAYLOAD_SIZE * 3 + 1];
            int n = 0;
            int h = 0;
            for (int i = 0; i < len && i < PAYLOAD_SIZE; i++)
            {
                text[n++] = (data[i] >= 0x20 && data[i] < 0x7F) ? (char)data[i] : '.';
                h += snprintf(hex + h, sizeof(hex) - h, "%02X ", data[i]);
            }
            text[n] = '\0';
            hex[h] = '\0';
            M5_LOGD("raw (%d bytes): \"%s\" | %s", len, text, hex);
        }

        // parse a MicroBlue frame: SOH <type><index> STX <x>,<y> ETX
        // e.g. 01 'd' '0' 02 "0,512" 03
        bool parse(const uint8_t *data, int len)
        {
            // locate the delimiters
            int soh = -1, stx = -1, etx = -1;
            for (int i = 0; i < len; i++)
            {
                if (data[i] == SOH && soh < 0)
                {
                    soh = i;
                }
                else if (data[i] == STX && stx < 0)
                {
                    stx = i;
                }
                else if (data[i] == ETX && etx < 0)
                {
                    etx = i;
                }
            }
            // header needs at least <type> and one <index> digit
            if (soh < 0 || stx < soh + 3 || etx < stx)
            {
                return false;
            }

            // header: <type><index>
            char type = (char)data[soh + 1];
            int k = 0;
            for (int i = soh + 2; i < stx; i++)
            {
                if (data[i] < '0' || data[i] > '9' || k >= 100)
                {
                    return false;
                }
                k = k * 10 + (data[i] - '0');
            }
            if (type != 'd')
            {
                M5_LOGW("unsupported widget type '%c' (index %d)", type, k);
                return false;
            }
            if (k >= MAX_SLOTS)
            {
                return false;
            }

            // body: "<x>,<y>"
            char body[16];
            int body_len = etx - stx - 1;
            if (body_len <= 0 || body_len >= (int)sizeof(body))
            {
                return false;
            }
            memcpy(body, data + stx + 1, body_len);
            body[body_len] = '\0';
            const char *comma = strchr(body, ',');
            if (comma == nullptr)
            {
                return false;
            }
            axes_[k][0] = constrain(atoi(body), 0, 1023);
            axes_[k][1] = constrain(atoi(comma + 1), 0, 1023);
            M5_LOGD("parsed: type='%c' k=%d x=%d y=%d", type, k, axes_[k][0], axes_[k][1]);
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
                logRaw(data, len);
                if (parse(data, len))
                {
                    // d0: x = turn (right is positive), y = forward (up is positive)
                    // NOTE: flip signs here if the axes feel reversed on the real device.
                    float turn = normalizeAxis(axes_[0][0]);
                    float forward = normalizeAxis(axes_[0][1]);

                    // Weaken the turn while moving forward/backward so that a diagonal input
                    // makes a gentle curve. A pure turn (forward = 0) is a spin turn as before.
                    float turn_eff = turn * (1.0f - TURN_REDUCTION_ON_MOVE * fabsf(forward));
                    // While reversing, invert the turn so that the rear follows the stick
                    // (e.g. stick at bottom-right: the rear moves to the right).
                    // The sign changes smoothly over REVERSE_TURN_BLEND to avoid a sudden
                    // flip of the spin direction when the stick is slightly tilted backward.
                    if (forward < 0.0f)
                    {
                        float t = constrain(-forward / REVERSE_TURN_BLEND, 0.0f, 1.0f);
                        turn_eff *= (1.0f - 2.0f * t);
                    }
                    float left = forward + turn_eff;
                    float right = forward - turn_eff;
                    // scale down to keep the left/right ratio instead of clipping
                    float peak = fmaxf(fabsf(left), fabsf(right));
                    if (peak > 1.0f)
                    {
                        left /= peak;
                        right /= peak;
                    }
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
