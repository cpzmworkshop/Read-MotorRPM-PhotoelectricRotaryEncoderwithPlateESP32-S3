Pembacaan dan Kalkulasi Nilai RPM Motor Menggunakan Sensor PhotoElectric Rotary Encoder with Plate dan Mikrokontroller ESP32-S3
---

## 1. Konfigurasi Rotary Encoder (Pin & Variabel)

```cpp
/**
 * @brief Konfigurasi rotary encoder untuk mengukur RPM motor propeller
 * 
 * @details
 * Rotary encoder menghasilkan pulse setiap kali motor berputar.
 * RPM dihitung dengan menghitung jumlah pulse per interval waktu.
 * Menggunakan moving average untuk smoothing data.
 */
#define PULSE_PIN_Motor_prop_1 9     // GPIO pin untuk encoder motor propeller 1
#define PULSE_PIN_Motor_prop_2 10    // GPIO pin untuk encoder motor propeller 2
volatile uint32_t pulse_counter_motor_prop_1 = 0;  // Counter kumulatif untuk motor 1
volatile uint32_t pulse_counter_motor_prop_2 = 0;  // Counter kumulatif untuk motor 2

/**
 * @brief Moving Average Configuration untuk smoothing RPM data
 * 
 * @details
 * Moving average digunakan untuk mengurangi noise pada pengukuran RPM.
 * Buffer menyimpan N sampel terakhir, kemudian dihitung rata-ratanya.
 */
#define RPM_SAMPLES_motor_prop_1 10  // Jumlah sampel untuk moving average motor 1
#define RPM_SAMPLES_motor_prop_2 10  // Jumlah sampel untuk moving average motor 2
uint32_t pulse_buffer_motor_prop_1[RPM_SAMPLES_motor_prop_1] = {0};  // Buffer untuk motor 1
uint32_t pulse_buffer_motor_prop_2[RPM_SAMPLES_motor_prop_2] = {0};  // Buffer untuk motor 2
uint8_t buffer_index_motor_prop_1 = 0;  // Index saat ini dalam buffer motor 1 (circular buffer)
uint8_t buffer_index_motor_prop_2 = 0;  // Index saat ini dalam buffer motor 2 (circular buffer)
```

---

## 2. Interrupt Handler untuk Rotary Encoder

```cpp
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
```

---

## 3. Setup Pulse Counter (dalam fungsi `setup()`)

```cpp
// Setup Pulse Counter for Motor Prop 1
pinMode(PULSE_PIN_Motor_prop_1, INPUT);
attachInterrupt(digitalPinToInterrupt(PULSE_PIN_Motor_prop_1), pulse_interrupt_handler_motor_prop_1, RISING);

// Setup Pulse Counter for Motor Prop 2
pinMode(PULSE_PIN_Motor_prop_2, INPUT);
attachInterrupt(digitalPinToInterrupt(PULSE_PIN_Motor_prop_2), pulse_interrupt_handler_motor_prop_2, RISING);
```

---

## 4. Kalkulasi RPM (dalam fungsi `loop()`, dijalankan setiap 100ms)

```cpp
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

// Calculate RPM and cast directly to uint16_t (PPR = 1, loop interval = 100ms = 0.1s)
// RPM = (avg_pulses_per_loop / 0.1s) * 60 = avg_pulses_per_loop * 600
dataToSend.rpm_prop_1 = (uint16_t)(avg_pulses_per_loop_motor_prop_1 * 600.0);  //rpm motor propeller 1

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

// Calculate RPM and cast directly to uint16_t (PPR = 1, loop interval = 100ms = 0.1s)
// RPM = (avg_pulses_per_loop / 0.1s) * 60 = avg_pulses_per_loop * 600
dataToSend.rpm_prop_2 = (uint16_t)(avg_pulses_per_loop_motor_prop_2 * 600.0);  //rpm motor propeller 2
```

---

## 5. Struktur Data untuk Pengiriman RPM

```cpp
// Dalam struct DatatoSend:
struct DatatoSend {
  // ... data lainnya ...
  uint16_t rpm_prop_1;    // rpm motor propeller 1 (× 100)
  uint16_t rpm_prop_2;    // rpm motor propeller 2 (× 100)
  // ... data lainnya ...
};
```

---

## 6. Ringkasan Formula Kalkulasi RPM

### Parameter

| Parameter | Nilai |
|-----------|-------|
| PPR (Pulse Per Revolution) | 1 |
| Loop interval | 100ms = 0.1 detik |
| Jumlah sampel moving average | 10 |

### Formula

1. **Hitung delta pulse:**
   ```
   pulses_per_loop = current_counter - previous_counter
   ```

2. **Simpan ke circular buffer** untuk moving average

3. **Hitung rata-rata:**
   ```
   avg_pulses = sum_of_buffer / jumlah_sampel
   ```

4. **Hitung RPM:**
   ```
   RPM = avg_pulses * 600
   ```
   
   **Penjelasan:**
   - `pulses_per_loop` adalah jumlah pulse dalam 100ms (0.1 detik)
   - Untuk mengkonversi ke RPM (putaran per menit):
     ```
     RPM = (pulses / 0.1s) * 60s = pulses * 600
     ```

### Catatan

- Menggunakan interrupt **RISING edge** untuk mendeteksi pulse
- **Moving average** (10 sampel) digunakan untuk smoothing data
- Data RPM disimpan sebagai `uint16_t` untuk pengiriman via ESP-NOW

