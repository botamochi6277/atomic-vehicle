#include <Arduino.h>
#include <M5Unified.h>

#include <M5AtomicMotion.h>
#include <FastLED.h>
#include "BLEVehicleService.hpp"

const int SERVO_L_PIN = 26;
const int SERVO_R_PIN = 32;
bool is_ble_connected = false;

ble::BLEVehicleService vehicle_service;

M5AtomicMotion atomic_motion;
float p_gain = 0.2f; // proportional gain for speed control

// LED setup
// NOTE: reference is https://docs.m5stack.com/ja/arduino/m5unified/helloworld
#define NUM_LEDS 1
#define LED_DATA_PIN 27
CRGB leds[NUM_LEDS];
uint32_t count = 0;

// ArduinoBLE does not tell the Arduino-ESP32 core that BLE is in use.
// Without this, newer cores release the BT controller memory at boot and
// BLE.begin() fails (esp_bt_controller_init returns ESP_ERR_INVALID_STATE).
extern "C" bool btInUse() { return true; }

// Function prototypes
void connectToAtomicMotion(M5AtomicMotion &motion);
void bleBegin();

void setup()
{
    auto cfg = M5.config();
    cfg.serial_baudrate = 115200;
    M5.begin(cfg);
    delay(500); // wait for the serial monitor to attach
    Serial.println("[setup] M5.begin done");
    Serial.printf("[setup] board id = %d\n", (int)M5.getBoard());

    // LED setup as indicator
    FastLED.addLeds<WS2811, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(20);
    Serial.println("[setup] FastLED.addLeds done");

    // LED self test: white for 500 ms
    leds[0] = CRGB::White;
    FastLED.show();
    Serial.println("[setup] LED set to white (self test)");
    delay(500);

    // AtomMotion & BLE setup
    Serial.println("[setup] connecting to Atomic Motion...");
    connectToAtomicMotion(atomic_motion);
    Serial.println("[setup] Atomic Motion connected");

    Serial.println("[setup] starting BLE...");
    bleBegin();
    Serial.println("[setup] setup finished");
}

void loop()
{
    M5.update();
    BLE.poll();
    is_ble_connected = BLE.connected();

    // LED indicator
    if (is_ble_connected)
    {
        leds[0] = count % 30 == 0 ? CRGB::Blue : CRGB::Black;
    }
    else
    {
        leds[0] = count % 30 == 0 ? CRGB::Green : CRGB::Black;
    }
    FastLED.show();

    auto is_written = vehicle_service.poll(atomic_motion, p_gain);
    if (is_written)
    {
        Serial.printf("Motor Speeds: L=%d, R=%d, d0=(%d,%d)\n",
                      atomic_motion.getMotorSpeed(LEFT_WHEEL_MOTOR_CH),
                      atomic_motion.getMotorSpeed(RIGHT_WHEEL_MOTOR_CH),
                      vehicle_service.axisX(0), vehicle_service.axisY(0));
    }

    delay(1000 / 30); // 30 FPS
    count++;
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
        Serial.printf("Atomic Motion begin failed (sda=%d, scl=%d, addr=0x%02X)\n", sda, scl, M5_ATOMIC_MOTION_I2C_ADDR);
        leds[0] = leds[0] == CRGB::Black ? CRGB::Red : CRGB::Black; // blink red
        FastLED.show();
        delay(1000);
    }
}

void bleBegin()
{
    if (!BLE.begin())
    {
        leds[0] = CRGB::Purple; // BLE init failure indicator
        FastLED.show();
        while (1)
        {
            Serial.println("starting BLE failed!");
            delay(1000);
        }
    }
    Serial.println("starting BLE success!");

    String ble_address = BLE.address();
    String local_name = "AtomicVehicle" + ble_address;
    BLE.setDeviceName(local_name.c_str());
    BLE.setLocalName("AtomicVehicle");
    BLE.setAdvertisedService(vehicle_service);
    Serial.println("[ble] setAdvertisedService done");

    // add service
    BLE.addService(vehicle_service);
    Serial.println("[ble] addService done");
    // start advertising
    bool adv_ok = BLE.advertise();
    Serial.printf("[ble] advertise() = %d, address = %s\n", (int)adv_ok, ble_address.c_str());
}