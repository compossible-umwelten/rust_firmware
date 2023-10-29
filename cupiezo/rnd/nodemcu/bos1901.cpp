#include <Arduino.h>
#include <SPI.h>
#define CS 15

#define REG_REFERENCE /* ----- */ 0x0  // 6.4.1 Table 12
#define REG_ION_BL /* -------- */ 0x1  // 6.4.1 Table 13
#define REG_DEADTIME /* ------ */ 0x2  // 6.4.1 Table 14
#define REG_KP /* ------------ */ 0x3  // 6.4.1 Table 15
#define REG_KPA_KI /* -------- */ 0x4  // 6.4.1 Table 16
#define REG_CONFIG /* -------- */ 0x5  // 6.4.1 Table 17
#define REG_PARCAP /* -------- */ 0x6  // 6.4.1 Table 18
#define REG_SUP_RISE /* ------ */ 0x7  // 6.4.1 Table 19
#define REG_DAC /* ----------- */ 0x8  // 6.4.1 Table 20
#define REG_IC_STATUS /* ----- */ 0xC  // 6.4.1 Table 21
#define REG_SENSE /* --------- */ 0xD  // 6.4.1 Table 22
#define REG_TRIM /* ---------- */ 0xE  // 6.4.1 Table 23

#define REFERENCE_ZERO /* ---- */ 0x0000
#define REG_READ_VFEEDBACK_MASK 0x03FF
const char *bit_rep[16] = {
    [0] = "0000",  [1] = "0001",  [2] = "0010",  [3] = "0011",
    [4] = "0100",  [5] = "0101",  [6] = "0110",  [7] = "0111",
    [8] = "1000",  [9] = "1001",  [10] = "1010", [11] = "1011",
    [12] = "1100", [13] = "1101", [14] = "1110", [15] = "1111",
};

void print_byte(uint8_t byte) {
  Serial.printf("%s%s", bit_rep[byte >> 4], bit_rep[byte & 0x0F]);
}
uint16_t spiReadWriteReg(uint16_t data) {
  uint8_t lsb = data & 0xFF;
  uint8_t msb = (data >> 8) & 0xFF;
  Serial.printf("> %0.4x =", data);
  Serial.printf(" %x | %s", msb >> 4, bit_rep[msb & 0x0F]);
  Serial.printf(" %s %s\n", bit_rep[lsb >> 4], bit_rep[lsb & 0x0F]);
  uint16_t ret = SPI.transfer16(data);
  lsb = ret & 0xFF;
  msb = (ret >> 8) & 0xFF;
  Serial.printf("< %0.4x =", ret);
  Serial.printf(" %x | %s", msb >> 4, bit_rep[msb & 0x0F]);
  Serial.printf(" %s %s\n\n", bit_rep[lsb >> 4], bit_rep[lsb & 0x0F]);
  return ret;
}

int16_t offset = 0;
uint16_t counter = 0;

class Config {
 private:
  /*
    +----+----+----+----+----+----+----+----+----+----+----+----+
    | 11 | 10 |  9 |  8 |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |
    +----+----+----+----+----+----+----+----+----+----+----+----+
    | BC                     |LOCK|RST |OE  |DS  | PLAY         |
    +----+----+----+----+----+----+----+----+----+----+----+----+
  */
  uint16_t config = REG_CONFIG << 12;

 public:
  void set_play(uint16_t value) {
    value &= 0x7;    // keep only 3 bits
    config &= ~0x7;  // clear existing bits
    config |= value;
  }
  void enable_ds /*------*/ () { config |= 0x1 << 3; }
  void disable_ds /*-----*/ () { config &= ~(0x1 << 3); }
  void enable_oe /*------*/ () { config |= 0x1 << 4; }
  void disable_oe /*-----*/ () { config &= ~(0x1 << 4); }
  void enable_reset /*---*/ () { config |= 0x1 << 5; }
  void disable_reset /*--*/ () { config &= ~(0x1 << 5); }
  void enable_lock /*----*/ () { config |= 0x1 << 6; }
  void disable_lock /*---*/ () { config &= ~(0x1 << 6); }
  void set_bc(uint16_t value) {
    value &= 0x1F;           // keep only 5 bits
    config &= ~(0x1F << 7);  // clear existing bits
    config |= value << 7;
  }
  uint16_t send() { return spiReadWriteReg(config); }
};

Config config;

void drivingInit() {
  // Reset IC, set SDO broadcast to SENSE register to
  // read VFEEDBACK, set PLAY sampling rate to 8kSPS.
  // spiReadWriteReg(0x56A7);
  config.enable_reset();
  config.set_play(0x7);
  config.set_bc(0xd);
  config.send();
  delay(50);

  config.enable_oe();
  config.disable_reset();
  config.send();

  config.set_bc(REG_IC_STATUS);
  config.send();

  spiReadWriteReg(0xC000);

  // Set SENSE = 0 & VDD = 31 (5V supply), TI_RISE = default
  spiReadWriteReg(0x77E7);
  spiReadWriteReg(REFERENCE_ZERO);
  delay(50);
  uint16_t reg_read = spiReadWriteReg(REFERENCE_ZERO);
  offset = reg_read & REG_READ_VFEEDBACK_MASK;
}

#define REFERENCE_PLUS_1LSB 0x0001
void drivingPressSetup() {
  spiReadWriteReg(0x7FE7);               // set SENSE = 1
  spiReadWriteReg(0x5697);               // Set BC = SENSE
  spiReadWriteReg(REFERENCE_PLUS_1LSB);  // write 0x0001 to set the
                                         // bridge to positive polarity
  // drivingNextState();              // go to next phase
}

#define PRESS_THRESHOLD 12   // in LSBs of VFEEDBACK
#define PRESS_HOLD_TIME 180  // Cycles of 125us (8kHz)
void drivingPress() {
  int16_t vfeedback =
      spiReadWriteReg(REFERENCE_PLUS_1LSB) &
      REG_READ_VFEEDBACK_MASK;          // Write 0x0001 and read VFEEDBACK
  int16_t vsense = vfeedback - offset;  // subtract the ADC offset value
  if (vsense >= PRESS_THRESHOLD) {      // compare to threshold
    counter++;  // increase counter value to implement hold time check
    if (counter >= PRESS_HOLD_TIME) {  // detection successful
      counter = 0;                     // reset counter
      // drivingNextState();              // go to next phase
    }
  }
}

#define PRESS_SIGNAL_FREQ 180  // Hertz
#define SAMPLING_RATE \
  8000  // Hertz, defined by PLAY parameter in CONFIG register
#define SIGNAL_SIZE_MAX 256        // Maximum table size
#define PRESS_SIGNAL_VOLTAGE 60    // Volts
#define RELEASE_SIGNAL_VOLTAGE 45  // Volts
#define RELEASE_SIGNAL_FREQ 180    // Hertz
static uint16_t
    press_waveform[SIGNAL_SIZE_MAX];  // Press feedback waveform data points
static uint16_t press_waveform_size =
    0;  // Press feedback waveform number of data points
static uint16_t
    release_waveform[SIGNAL_SIZE_MAX];  // Release feedback waveform data points
static uint16_t release_waveform_size =
    0;  // Release feedback waveform number of data points
int16_t utilsVolt2Amplitude(float volt) {
  int16_t amplitude = volt * 2047 / 3.6 / 31;
  return amplitude;
}

// Calculate Press and Release Feedback Waveforms
void drivingCalculateWaveforms(void) {
  // Press Feedback Waveform : single sine pulse
  press_waveform_size = SAMPLING_RATE / PRESS_SIGNAL_FREQ;
  for (uint8_t i = 0; i < press_waveform_size; i++) {
    press_waveform[i] = utilsVolt2Amplitude(
        PRESS_SIGNAL_VOLTAGE / 2 *
        (sin(2 * PI * i / (press_waveform_size)-PI / 2) + 1));
  }

  // Release Feedback Waveform : single sine pulse
  release_waveform_size = SAMPLING_RATE / RELEASE_SIGNAL_FREQ;
  for (uint8_t i = 0; i < release_waveform_size; i++) {
    release_waveform[i] = utilsVolt2Amplitude(
        RELEASE_SIGNAL_VOLTAGE / 2 *
        (sin(2 * PI * i / (release_waveform_size)-PI / 2) + 1));
  }
}

void drivingWaitFifoEmpty() {
  bool fifoempty = 0;
  // Set up broadcast to read IC_STATUS
  spiReadWriteReg(0x5617);  // Set BC = IC_STATUS

  // loop until FIFO is empty
  while (!fifoempty) {
    delayMicroseconds(1E6 / SAMPLING_RATE / 2);  // wait half a PLAY period.
    uint16_t ic_status_reg =
        SPI.transfer16(0xC000);  // dummy write, get IC_STATUS value
    uint8_t lsb = ic_status_reg & 0xFF;
    uint8_t msb = (ic_status_reg >> 8) & 0xFF;
    Serial.printf("< %0.4x =", ic_status_reg);
    Serial.printf(" %x | %s", msb >> 4, bit_rep[msb & 0x0F]);
    Serial.printf(" %s %s\n", bit_rep[lsb >> 4], bit_rep[lsb & 0x0F]);
    fifoempty = (ic_status_reg >> 6) & 0x1;  // extract EMPTY value.
  }
}

void setup() {
  Serial.begin(115200);
  Serial.printf("\n");
  delay(500);
  Serial.printf("Started\n");
  pinMode(CS, OUTPUT);
  digitalWrite(CS, LOW);
  SPI.begin();
  drivingInit();
  drivingPressSetup();
  drivingCalculateWaveforms();
}

void loop() {
  // for (int i = 0; i < press_waveform_size; i++) {
  //   SPI.transfer16(press_waveform[i]);
  // }
  // for (int i = 0; i < release_waveform_size; i++) {
  //   SPI.transfer16(release_waveform[i]);
  // }
  // drivingWaitFifoEmpty();

  uint16_t ret = SPI.transfer16(0xD000);  // dummy write
  uint8_t lsb = ret & 0xFF;
  uint8_t msb = (ret >> 8) & 0xFF;
  Serial.printf("< %0.4x =", ret);
  Serial.printf(" %x | %s", msb >> 4, bit_rep[msb & 0x0F]);
  Serial.printf(" %s %s\n", bit_rep[lsb >> 4], bit_rep[lsb & 0x0F]);
}
