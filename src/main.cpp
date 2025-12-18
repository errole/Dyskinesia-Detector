
#include "mbed.h"
#include "arm_math.h" // CMSIS-DSP for FFT
#include <cmath>
#include "ble/BLE.h"
#include "ble/gatt/GattService.h"
#include "ble/gatt/GattCharacteristic.h"
#include "ble/Gap.h"
#include "ble/gap/AdvertisingDataBuilder.h"
#include "events/EventQueue.h"
#include <chrono>
#include <string>

// Bring in namespaces so we don't have to type them each time
using namespace ble;
using namespace events;
using namespace std::chrono;

// 1.Hardware Definitions

// Initialize I2C on pins PB_11 (SDA) and PB_10 (SCL)
I2C i2c(PB_11, PB_10);

// LSM6DSL I2C address (0x6A shifted left by 1 for Mbed's 8-bit addressing)
#define LSM6DSL_ADDR        (0x6A << 1)

// Register addresses (Explicit Definitions as requested)
#define WHO_AM_I            0x0F  // Device identification register
#define CTRL1_XL            0x10  // Accelerometer control register
#define CTRL2_G             0x11  // Gyroscope Config
#define CTRL3_C             0x12  // Control Register 3
#define DRDY_PULSE_CFG      0x0B  // Data Ready Pulse Config
#define INT1_CTRL           0x0D  // Interrupt Control
#define STATUS_REG          0x1E  // Status Register

// Explicit Axis Output Registers (Low/High bytes)
#define OUTX_L_XL           0x28
#define OUTX_H_XL           0x29
#define OUTY_L_XL           0x2A
#define OUTY_H_XL           0x2B
#define OUTZ_L_XL           0x2C
#define OUTZ_H_XL           0x2D

// INT1 interrupt pin (wired to PD_11 on the Discovery board)
#define LSM6DSL_INT1_PIN    PD_11

// Interrupt input and data-ready flag
InterruptIn int1(LSM6DSL_INT1_PIN, PullDown);
volatile bool data_ready = false;

// 2.Parameters

// Sampling Parameters (Project Requirement: 52Hz)
const float SAMPLE_RATE_HZ = 52.0f;

// Window duration (3 seconds as per prompt)
const int   WINDOW_SECONDS = 3;

// Number of samples per window (52Hz * 3s = 156 samples)
const int   WINDOW_SIZE = (int)(SAMPLE_RATE_HZ * WINDOW_SECONDS); 

// FFT Parameters (Must be power of 2, >= WINDOW_SIZE)
#define FFT_SIZE            256 
const float FREQ_BIN_SIZE = SAMPLE_RATE_HZ / FFT_SIZE; // Resolution ~0.2Hz

// Sensitivity for +/- 2g 
const float ACC_SENSITIVITY = 0.061f / 1000.0f; 


// 3.Globals & Buffers

// Circular buffer for window storage
float window_buf[WINDOW_SIZE];
int   window_index = 0;
bool  window_full  = false;

// FFT Buffers (CMSIS-DSP)
float fft_input[FFT_SIZE];       // Input for FFT (Real)
float fft_output[FFT_SIZE];      // Output of FFT (Complex)
float fft_mag[FFT_SIZE / 2];     // Magnitude of FFT

// CMSIS-DSP Instance
arm_rfft_fast_instance_f32 S;   

//last 3s movement is walking
bool was_walking = false;

// ISR: set flag when IMU asserts DRDY
void data_ready_isr() {
    data_ready = true;
}

// Write a single IMU register
bool write_reg(uint8_t reg, uint8_t val) {
    char buf[2] = { (char)reg, (char)val };
    return (i2c.write(LSM6DSL_ADDR, buf, 2) == 0);
}

// Read a single IMU register
bool read_reg(uint8_t reg, uint8_t &val) {
    char r = (char)reg;
    if (i2c.write(LSM6DSL_ADDR, &r, 1, true) != 0) return false;
    // Read the register value
    if (i2c.read(LSM6DSL_ADDR, &r, 1) != 0) return false;
    val = (uint8_t)r;
    return true;
}

// Read a 16-bit signed value from two consecutive registers
bool read_int16(uint8_t reg_low, int16_t &val) {
    uint8_t lo, hi;
    if (!read_reg(reg_low, lo)) return false;
    if (!read_reg(reg_low + 1, hi)) return false; 
    val = (int16_t)((hi << 8) | lo);
    return true;
}

// Initialize the LSM6DSL IMU
bool init_sensor() {
    uint8_t who;
    // Read WHO_AM_I register and verify it's 0x6A
    if (!read_reg(WHO_AM_I, who) || who != 0x6A) {
        printf("Sensor not found! WHO_AM_I = 0x%02X\r\n", who);
        return false;
    }

    // Configure Block Data Update (BDU)
    write_reg(CTRL3_C, 0x44);

    // Configure Accelerometer for 52Hz, +/- 2g
    write_reg(CTRL1_XL, 0x30); 

    // Power down gyro to save power
    write_reg(CTRL2_G, 0x00);

    // Route Data Ready to INT1 pin
    write_reg(INT1_CTRL, 0x01); // INT1_DRDY_XL = 1

    // Enable DRDY pulse mode 
    write_reg(DRDY_PULSE_CFG, 0x80);

    // Clear stale data (Classmate's good practice)
    uint8_t dummy;
    read_reg(STATUS_REG, dummy);
    int16_t temp;
    read_int16(OUTX_L_XL, temp);
    read_int16(OUTY_L_XL, temp);
    read_int16(OUTZ_L_XL, temp);

    // Attach interrupt handler
    int1.rise(&data_ready_isr);

    return true;
}

// 5. Processing Logic

// Process full window: FFT Analysis (Based on main3.cpp logic)
void process_full_window() {
    
    // Prepare Data Copy window buffer to FFT input & Remove DC Offset
    float sum = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        fft_input[i] = window_buf[i];
        sum += window_buf[i];
    }
    
    float mean = sum / WINDOW_SIZE;
    
    // Fill FFT buffer (Subtract Mean + Zero Padding)
    for (int i = 0; i < FFT_SIZE; i++) {
        if (i < WINDOW_SIZE) {
            fft_input[i] -= mean; // Remove Gravity/DC
        } else {
            fft_input[i] = 0.0f;  // Zero padding for samples beyond 3s
        }
    }
    // forward FFT
    arm_rfft_fast_f32(&S, fft_input, fft_output, 0); 
    
    // Compute Magnitude
    arm_cmplx_mag_f32(fft_output, fft_mag, FFT_SIZE / 2);

    // Find Dominant Frequency (main3 logic)
    float max_val = 0.0f;
    int max_bin = 0;
    
    // Skip DC bin (0)
    for (int i = 1; i < FFT_SIZE / 2; i++) {
        if (fft_mag[i] > max_val) {
            max_val = fft_mag[i];
            max_bin = i;
        }
    }

    float dom_freq = max_bin * FREQ_BIN_SIZE;

    // Output Result 
    printf("--- Window Analysis (3s) ---\n");
    printf("Samples: %d | Freq Resolution: %.2f Hz\n", WINDOW_SIZE, FREQ_BIN_SIZE);
    printf("Dominant Freq: %.2f Hz (Mag: %.2f)\n", dom_freq, max_val);

    printf(">detect_freq:%.2f\n", dom_freq); 
    printf(">max_mag:%.2f\n", max_val);

    //status logic and output
    //Tremor / Dyskinesia / Walking / FOG
    if (max_val > 10.0f) { 
        if (dom_freq >= 3.0f && dom_freq <= 5.0f) {
            printf(">>> DETECTED: Tremor (3-5 Hz)\n");
            was_walking = false;
        } 
        else if (dom_freq > 5.0f && dom_freq <= 20.0f) {
            printf(">>> DETECTED: Dyskinesia\n");
            was_walking = false;
        }
        else if (dom_freq >= 0.5f && dom_freq < 3.0f) {
            printf(">>> Status: Walking\n");
            was_walking = true;
        }
        else {
            printf(">>> Status: Unknown High Fre\n");
            was_walking = false;
        }
    } else if(was_walking == true){
        printf(">>> DETECTED: FOG \n");
        was_walking = false;
    } else{
        printf(">>> Status: No Movement\n");
    }

}

// Read Sensor and Push to Window 
void read_sensor_and_push_to_window() {
    int16_t x_raw, y_raw, z_raw;

    // Read 3 Axes Explicitly 
    read_int16(OUTX_L_XL, x_raw);
    read_int16(OUTY_L_XL, y_raw);
    read_int16(OUTZ_L_XL, z_raw);

    // Convert to g (2g: 0.061 mg/LSB)
    float ax = x_raw * ACC_SENSITIVITY;
    float ay = y_raw * ACC_SENSITIVITY;
    float az = z_raw * ACC_SENSITIVITY;

    // Calculate Magnitude (3-axis factor consideration)
    float acc_mag = sqrt(ax*ax + ay*ay + az*az);

    //graph the raw data
    printf(">ax:%.2f\n>ay:%.2f\n>az:%.2f\n>mag:%.2f\n", ax, ay, az, acc_mag);

    // Add to Circular Buffer
    window_buf[window_index] = acc_mag;
    window_index++;

    // Wrap around
    if (window_index >= WINDOW_SIZE) {
        window_index = 0;
        window_full = true;
    }

    // Process only when buffer is full 
    if (window_full) {
        process_full_window();
        window_full = false; // Reset flag to collect next 3s batch
    }
}

// 6. Bluetooth

// Create serial and bind it to printf (Required for Teleplot)
BufferedSerial serial_port(USBTX, USBRX, 115200);

FileHandle *mbed::mbed_override_console(int) {
    return &serial_port;
}

// Get the BLE instance and set up event queue
BLE &ble_interface = BLE::Instance();
EventQueue event_queue;
// LED for visual feedback when counter updates
DigitalOut led(LED1);

// Unique IDs for our service and characteristic - these are made-up values
const UUID TREMOR_SERVICE_UUID("A0E1B2C3-D4E5-F6A7-B8C9-D0E1F2A3B4C5");
const UUID STATUS_MSG_CHAR_UUID("A1E2B3C4-D5E6-F7A8-B9C0-D1E2F3A4B5C6");

// Main counter and stats counters
uint8_t counter = 0;
uint8_t preMedCount = 0;
uint8_t Dyskinesia = 0;

// Buffer to hold our status message - big enough for a detailed message
uint8_t statusBuffer[100] = {0};
uint8_t statusLength = 0;

// BLE characteristic - allows read and notify operations
ReadOnlyArrayGattCharacteristic<uint8_t, sizeof(statusBuffer)>statusMsgCharacteristic(
    STATUS_MSG_CHAR_UUID,
    statusBuffer,
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY
);

// Add characteristic to table and create service
GattCharacteristic *charTable[] = { &statusMsgCharacteristic };
GattService tremorService(TREMOR_SERVICE_UUID, charTable, 1);

// Timer for regular counter updates
Ticker counter_ticker;
// Flag to track if anyone's connected
bool device_connected = false;

// Creates and sends the status message with current state
void update_status_message() {
    // Determine state based on counter value
    // Using modulo to create patterns - every 3rd count is pre-med, every 5th is dyskinesia
    bool isPre = (counter % 3 == 0);
    bool isPost = (counter % 5 == 0);
    char stateStr[20] = "";

    // Handle all possible state combinations
    if (isPre && isPost) {
    strcpy(stateStr, "Pre+Dyskinesia");
    } else if (isPre) {
    strcpy(stateStr, "Pre-med");
    } else if (isPost) {
    strcpy(stateStr, "Dyskinesia");
    } else {
    strcpy(stateStr, "Neither");
    }

    // Format all the data into our status buffer
    snprintf((char*)statusBuffer, sizeof(statusBuffer),
        "Count: %d, State: %s, Pre: %d, dys: %d",
        counter, stateStr, preMedCount, Dyskinesia);
    statusLength = strlen((char*)statusBuffer);

    // Print locally for debugging
    printf("%s\n", statusBuffer);
    // Only send notification if a device is connected

    if (device_connected) {
        ble_interface.gattServer().write(
        statusMsgCharacteristic.getValueHandle(),
        statusBuffer,
        statusLength
        );
        printf("Notification sent\n");
    }
}

// Called every second to update the counter and state
void update_counter() {
    counter++;
    // Reset after 100 to keep values in reasonable range
    if (counter > 100) {
        counter = 1;
        preMedCount = 0;
        Dyskinesia = 0;
    }
    // Check states and update corresponding counters
    bool isPre = (counter % 3 == 0);
    bool isPost = (counter % 5 == 0);
    if (isPre) preMedCount++;
    if (isPost) Dyskinesia++;
    // Toggle LED for visual feedback
    led = !led;
    // Update and possibly send the status message
    update_status_message();
}

// Handler for connection events
class ConnectionEventHandler : public ble::Gap::EventHandler {
public:
    // Called when a device connects
    virtual void onConnectionComplete(const ble::ConnectionCompleteEvent &event) {
        if (event.getStatus() == BLE_ERROR_NONE) {
            printf("Device connected!\n");
            device_connected = true;
            update_status_message();
        }
    }
    // Called when a device disconnects
    virtual void onDisconnectionComplete(const ble::DisconnectionCompleteEvent &event) {
        printf("Device disconnected!\n");
        device_connected = false;
        // Restart advertising so others can find us
        ble_interface.gap().startAdvertising(ble::LEGACY_ADVERTISING_HANDLE);
        printf("Restarted advertising\n");
    }
};

// Create our connection handler
ConnectionEventHandler connection_handler;

// Called when BLE finishes initializing
void on_ble_init_complete(BLE::InitializationCompleteCallbackContext*params) {
    if (params->error != BLE_ERROR_NONE) {
    printf("BLE initialization failed.\n");
    return;
    }
    // Register our service with BLE
    ble_interface.gattServer().addService(tremorService);
    // Create advertising data buffer
    uint8_t adv_buffer[LEGACY_ADVERTISING_MAX_SIZE];
    // Helper to build the advertising data
    AdvertisingDataBuilder adv_data(adv_buffer);
    // Set required flags
    adv_data.setFlags();
    // Name that shows up in scanning
    adv_data.setName("Tremor-Counter");
    // Set advertising parameters - 100ms intervals
    ble_interface.gap().setAdvertisingParameters(
        LEGACY_ADVERTISING_HANDLE,
        AdvertisingParameters(advertising_type_t::CONNECTABLE_UNDIRECTED,
        adv_interval_t(160))
    );
    // Set the payload (the actual data)
    ble_interface.gap().setAdvertisingPayload(
        LEGACY_ADVERTISING_HANDLE,
        adv_data.getAdvertisingData()
    );

    // Register our connection handler
    ble_interface.gap().setEventHandler(&connection_handler);

    // Start broadcasting our presence
    ble_interface.gap().startAdvertising(LEGACY_ADVERTISING_HANDLE);

    printf("BLE advertising started as Tremor-Counter\n");
    printf("Starting counter from 1 to 100...\n");

    // Reset counter and start the ticker
    counter = 0;
    counter_ticker.attach([]() {
    event_queue.call(update_counter);
    }, 1s);
}

// Handle BLE events by adding them to our event queue
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext*context) {
    event_queue.call(callback(&ble_interface, &BLE::processEvents));
}



int main() {
    printf("Starting BLE Tremor Counter...\n");

    // Set up event handler for BLE
    //ble_interface.onEventsToProcess(schedule_ble_events);

    // Initialize BLE with our callback
    //ble_interface.init(on_ble_init_complete);

    // Run the event queue forever - this loops until power off
    //event_queue.dispatch_forever();

    // Configure high-speed I2C
    i2c.frequency(400000);

    // Initialize FFT structure
    arm_rfft_fast_init_f32(&S, FFT_SIZE);

    // Initialize Sensor
    if (!init_sensor()) {
        while(1) { 
            printf("Sensor Init Failed.\r\n");
            ThisThread::sleep_for(1s); 
        }
    }

    while (true) {
        if (data_ready) {
            data_ready = false;
            read_sensor_and_push_to_window();
        }
    }
}