#include <Arduino.h>
#include <M5Unified.h>

#include <M5AtomicMotion.h>
#include "BLEVehicleService.hpp"

const int SERVO_L_PIN = 26;
const int SERVO_R_PIN = 32;

ble::BLEVehicleService vehicle_service;

M5AtomicMotion atomic_motion;
float p_gain = 0.2f; // proportional gain for speed control

void connectToAtomicMotion(M5AtomicMotion &motion);

void setup()
{
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);

    connectToAtomicMotion(atomic_motion);

    // BLE setup
    if (!BLE.begin())
    {
        Serial.println("starting BLE failed!");
        while (1)
            ;
    }
    Serial.println("starting BLE success!");

    String ble_address = BLE.address();
    String local_name = "AtomicVehicle" + ble_address;
    BLE.setDeviceName(local_name.c_str());
    BLE.setLocalName("AtomicVehicle");
    BLE.setAdvertisedService(vehicle_service);

    // add service
    BLE.addService(vehicle_service);
    // start advertising
    BLE.advertise();
}

void loop()
{
    M5.update();
    BLE.poll();
    auto is_written = vehicle_service.poll(atomic_motion, p_gain);
    if (is_written)
    {
        Serial.printf("Motor Speeds: L=%d, R=%d, %c\n",
                      atomic_motion.getMotorSpeed(LEFT_WHEEL_MOTOR_CH),
                      atomic_motion.getMotorSpeed(RIGHT_WHEEL_MOTOR_CH),
                      vehicle_service.command_char.value());
    }

    delay(1000 / 30); // 30 FPS
}

void connectToAtomicMotion(M5AtomicMotion &motion)
{
    m5::board_t board = M5.getBoard();

    uint8_t sda = 0, scl = 0;

    if (board == m5::board_t::board_M5AtomLite || board == m5::board_t::board_M5AtomMatrix ||
        board == m5::board_t::board_M5AtomEcho)
    {
        sda = 25;
        scl = 21;
    }
    else if (board == m5::board_t::board_M5AtomS3 || board == m5::board_t::board_M5AtomS3R ||
             board == m5::board_t::board_M5AtomS3Lite || board == m5::board_t::board_M5AtomS3RExt ||
             board == m5::board_t::board_M5AtomS3RCam)
    {
        sda = 38;
        scl = 39;
    }

    while (!motion.begin(&Wire, M5_ATOMIC_MOTION_I2C_ADDR, sda, scl, 100000))
    {
        Serial.println("Atomic Motion begin failed");
        delay(1000);
    }
}