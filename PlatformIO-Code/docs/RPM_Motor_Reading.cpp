/**
 * @file RPM_Motor_Reading.cpp
 * @brief Pembacaan dan Kalkulasi RPM Motor menggunakan Rotary Encoder
 * 
 * @description
 * Kode ini mengukur RPM motor propeller menggunakan rotary encoder.
 * Menggunakan interrupt untuk menghitung pulse dan moving average untuk smoothing.
 * 
 * Hardware:
 * - ESP32-S3
 * - Rotary Encoder Motor Propeller 1 (GPIO 9)
 * - Rotary Encoder Motor Propeller 2 (GPIO 10)
 * 
 * @author Extracted from Ship Model Control System
 * @version 1.0
 * @date 2025
 */

#include <Arduino.h>

// ============================================================================
// Rotary Encoder Configuration (RPM Measurement)
// ============================================================================

/**
 * @brief Konfigurasi rotary encoder untuk mengukur RPM motor propeller
 * 
 * @details
 * Rotary encoder menghasilkan pulse setiap kali motor berputar.
 * RPM dihitung dengan menghitung jumlah pulse per interval waktu.
 * Menggunakan moving average untuk smoothing data.
 */
#define PULSE_PIN_Motor_prop_1 9     ///< GPIO pin untuk encoder motor propeller 1
#define PULSE_PIN_Motor_prop_2 10    ///< GPIO pin untuk encoder motor propeller 2

volatile uint32_t pulse_counter_motor_prop_1 = 0;  ///< Counter kumulatif untuk motor 1
volatile uint32_t pulse_counter_motor_prop_2 = 0;  ///< Counter kumulatif untuk motor 2

/**
 * @brief Moving Average Configuration untuk smoothing RPM data
 * 
 * @details
 * Moving average digunakan untuk mengurangi noise pada pengukuran RPM.
 * Buffer menyimpan N sampel terakhir, kemudian dihitung rata-ratanya.
 */
#define RPM_SAMPLES_motor_prop_1 10  ///< Jumlah sampel untuk moving average motor 1
#define RPM_SAMPLES_motor_prop_2 10  ///< Jumlah sampel untuk moving average motor 2

uint32_t pulse_buffer_motor_prop_1[RPM_SAMPLES_motor_prop_1] = {0};  ///< Buffer untuk motor 1
uint32_t pulse_buffer_motor_prop_2[RPM_SAMPLES_motor_prop_2] = {0};  ///< Buffer untuk motor 2
uint8_t buffer_index_motor_prop_1 = 0;  ///< Index saat ini dalam buffer motor 1 (circular buffer)
uint8_t buffer_index_motor_prop_2 = 0;  ///< Index saat ini dalam buffer motor 2 (circular buffer)

// ============================================================================
// Timing Configuration
// ============================================================================

unsigned long previousMillis = 0;    ///< Waktu millis() sebelumnya untuk interval timing
const long intervaltime = 100;       ///< Interval waktu dalam ms (100ms = 10 Hz)

// ============================================================================
// RPM Data Storage
// ============================================================================

uint16_t rpm_prop_1 = 0;  ///< RPM motor propeller 1
uint16_t rpm_prop_2 = 0;  ///< RPM motor propeller 2

// ============================================================================
// Interrupt Handlers
// ============================================================================

/**
 * @brief Interrupt handler untuk rotary encoder motor propeller 1
 * 
 * @details
 * Function ini dipanggil setiap kali terjadi rising edge pada PULSE_PIN_Motor_prop_1.
 * Increment counter untuk menghitung jumlah pulse (putaran motor).
 * 
 * @note
 * Function ini harus dideklarasikan dengan IRAM_ATTR karena dipanggil dari interrupt.
 * Counter harus volatile karena diakses dari interrupt dan main loop.
 */
void IRAM_ATTR pulse_interrupt_handler_motor_prop_1() {
    pulse_counter_motor_prop_1++;  // Increment counter pada rising edge
}

/**
 * @brief Interrupt handler untuk rotary encoder motor propeller 2
 * 
 * @details
 * Function ini dipanggil setiap kali terjadi rising edge pada PULSE_PIN_Motor_prop_2.
 * Increment counter untuk menghitung jumlah pulse (putaran motor).
 * 
 * @note
 * Function ini harus dideklarasikan dengan IRAM_ATTR karena dipanggil dari interrupt.
 * Counter harus volatile karena diakses dari interrupt dan main loop.
 */
void IRAM_ATTR pulse_interrupt_handler_motor_prop_2() {
    pulse_counter_motor_prop_2++;  // Increment counter pada rising edge
}

// ============================================================================
// Setup Function
// ============================================================================

/**
 * @brief Setup function - Inisialisasi hardware
 * 
 * @details
 * Function ini melakukan inisialisasi:
 * 1. Serial Monitor (115200 baud)
 * 2. Pulse counter interrupt untuk rotary encoder
 */
void setup() {
    // ========== Serial Monitor Initialization ==========
    Serial.begin(115200);
    delay(500);
    Serial.println("RPM Motor Reading Initialized");
    
    // ========== Setup Pulse Counter for Motor Prop 1 ==========
    pinMode(PULSE_PIN_Motor_prop_1, INPUT);
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN_Motor_prop_1), 
                    pulse_interrupt_handler_motor_prop_1, RISING);
    Serial.println("Encoder Motor 1 attached to GPIO 9");
    
    // ========== Setup Pulse Counter for Motor Prop 2 ==========
    pinMode(PULSE_PIN_Motor_prop_2, INPUT);
    attachInterrupt(digitalPinToInterrupt(PULSE_PIN_Motor_prop_2), 
                    pulse_interrupt_handler_motor_prop_2, RISING);
    Serial.println("Encoder Motor 2 attached to GPIO 10");
    
    Serial.println("Setup complete. Reading RPM...");
}

// ============================================================================
// Loop Function
// ============================================================================

/**
 * @brief Loop function - Kalkulasi dan tampilkan RPM
 * 
 * @details
 * Function ini berjalan setiap 100ms (10 Hz) dan melakukan:
 * 1. Menghitung delta pulse dari interval sebelumnya
 * 2. Menyimpan ke circular buffer untuk moving average
 * 3. Menghitung rata-rata pulse
 * 4. Mengkonversi ke RPM
 * 5. Menampilkan ke Serial Monitor
 * 
 * Formula RPM:
 * - PPR (Pulse Per Revolution) = 1
 * - Loop interval = 100ms = 0.1 detik
 * - RPM = (pulses_per_loop / 0.1s) * 60s = pulses_per_loop * 600
 */
void loop() {
    unsigned long currentMillis = millis();

    // ========== Main Loop - Run setiap 100ms (10 Hz) ==========
    if (currentMillis - previousMillis >= intervaltime) {
        previousMillis = currentMillis;

        // ========== Calculate RPM for Motor Prop 1 ==========
        // Calculate pulses per loop (100ms) using delta method
        static uint32_t previous_pulse_counter_motor_prop_1 = 0;
        uint32_t pulses_per_loop_motor_prop_1 = pulse_counter_motor_prop_1 - previous_pulse_counter_motor_prop_1;
        previous_pulse_counter_motor_prop_1 = pulse_counter_motor_prop_1;
        
        // Add to moving average buffer (circular buffer)
        pulse_buffer_motor_prop_1[buffer_index_motor_prop_1] = pulses_per_loop_motor_prop_1;
        buffer_index_motor_prop_1 = (buffer_index_motor_prop_1 + 1) % RPM_SAMPLES_motor_prop_1;  // Circular index
        
        // Calculate moving average
        uint32_t pulse_sum_motor_prop_1 = 0;
        for (uint8_t i = 0; i < RPM_SAMPLES_motor_prop_1; i++) {
            pulse_sum_motor_prop_1 += pulse_buffer_motor_prop_1[i];
        }
        float avg_pulses_per_loop_motor_prop_1 = pulse_sum_motor_prop_1 / (float)RPM_SAMPLES_motor_prop_1;
        
        // Calculate RPM (PPR = 1, loop interval = 100ms = 0.1s)
        // RPM = (avg_pulses_per_loop / 0.1s) * 60 = avg_pulses_per_loop * 600
        rpm_prop_1 = (uint16_t)(avg_pulses_per_loop_motor_prop_1 * 600.0);

        // ========== Calculate RPM for Motor Prop 2 ==========
        // Calculate pulses per loop (100ms) using delta method
        static uint32_t previous_pulse_counter_motor_prop_2 = 0;
        uint32_t pulses_per_loop_motor_prop_2 = pulse_counter_motor_prop_2 - previous_pulse_counter_motor_prop_2;
        previous_pulse_counter_motor_prop_2 = pulse_counter_motor_prop_2;
        
        // Add to moving average buffer (circular buffer)
        pulse_buffer_motor_prop_2[buffer_index_motor_prop_2] = pulses_per_loop_motor_prop_2;
        buffer_index_motor_prop_2 = (buffer_index_motor_prop_2 + 1) % RPM_SAMPLES_motor_prop_2;  // Circular index
        
        // Calculate moving average
        uint32_t pulse_sum_motor_prop_2 = 0;
        for (uint8_t i = 0; i < RPM_SAMPLES_motor_prop_2; i++) {
            pulse_sum_motor_prop_2 += pulse_buffer_motor_prop_2[i];
        }
        float avg_pulses_per_loop_motor_prop_2 = pulse_sum_motor_prop_2 / (float)RPM_SAMPLES_motor_prop_2;
        
        // Calculate RPM (PPR = 1, loop interval = 100ms = 0.1s)
        // RPM = (avg_pulses_per_loop / 0.1s) * 60 = avg_pulses_per_loop * 600
        rpm_prop_2 = (uint16_t)(avg_pulses_per_loop_motor_prop_2 * 600.0);

        // ========== Print RPM to Serial Monitor ==========
        Serial.print("RPM_prop_1: ");
        Serial.print(rpm_prop_1);
        Serial.print(" | RPM_prop_2: ");
        Serial.print(rpm_prop_2);
        Serial.print(" | Total_pulses_1: ");
        Serial.print(pulse_counter_motor_prop_1);
        Serial.print(" | Total_pulses_2: ");
        Serial.println(pulse_counter_motor_prop_2);
    }
}
