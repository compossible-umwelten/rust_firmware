#include <Arduino.h>
#include <SPI.h>

/*
  Attempt to adapt c code to work with esp.
*/

/********************************************************
 *			        DEFINES / PARAMETERS
 ********************************************************/
// Channels
#define BOS1901_CHANNEL_A 0
#define BOS1901_CHANNEL_B 1
#define NB_CHANNELS 2

// Reference Values
#define REFERENCE_ZERO 0x0000
#define REFERENCE_PLUS_1LSB 0x0001
#define REFERENCE_MINUS_1LSB 0x0FFF

// Sensing
#define PRESS_HOLD_TIME 180   // Cycles of 125us (8kHz)
#define RELEASE_HOLD_TIME 30  // Cycles of 125us (8kHz)
#define PRESS_THRESHOLD 12    // in LSBs of VFEEDBACK
#define RELEASE_THRESHOLD 0   // in LSBs of VFEEDBACK

// Feedback
#define SAMPLING_RATE \
  8000  // Hertz, defined by PLAY parameter in CONFIG register
#define SIGNAL_SIZE_MAX 256        // Maximum table size
#define PRESS_SIGNAL_VOLTAGE 60    // Volts
#define PRESS_SIGNAL_FREQ 180      // Hertz
#define RELEASE_SIGNAL_VOLTAGE 45  // Volts
#define RELEASE_SIGNAL_FREQ 180    // Hertz
#define REG_READ_VFEEDBACK_MASK 0x03FF

// Piezo Creep
#define CREEP_HOLD_TIME 40  // Cycles of 125us (8kHz)

// Timer
#define TIMER1_CLOCK_FREQ 64000000
#define TIMER1_FREQ_TRIM 8200
#define TIMER1_FREQ_DEFAULT 8000

// Trimming
#define FIFO_SPACE_MASK (0x7F)
#define MAX_TRY 10800
#define TRIM_OSC_MAX_POS 0x1F
#define TRIM_OSC_MASK 0x3F

/********************************************************
 *						STRUCTURES
 ********************************************************/

// Sensing Phases
typedef enum {
  DrivingState_A_init,
  DrivingState_B_press_setup,
  DrivingState_B_press,
  DrivingState_C_press_feedback,
  DrivingState_D_creep,
  DrivingState_E_release_setup,
  DrivingState_E_release,
  DrivingState_F_release_feedback,
  DrivingState_G_creep
} DrivingState;

// Container for sensing variables
typedef struct {
  DrivingState state;  // current sensing state
  int16_t offset;      // ADC offset for sensing
  uint16_t counter;    // counts time in 8kHz cycles
} Bos1901;

/********************************************************
 *						VARIABLES
 ********************************************************/

// one container of variables per channel
static Bos1901 bos1901 = {DrivingState_A_init, 0, 0};  // Channel A
static uint16_t
    press_waveform[SIGNAL_SIZE_MAX];  // Press feedback waveform data points
static uint16_t press_waveform_size =
    0;  // Press feedback waveform number of data points
static uint16_t
    release_waveform[SIGNAL_SIZE_MAX];  // Release feedback waveform data points
static uint16_t release_waveform_size =
    0;  // Release feedback waveform number of data points

// Flag variable associated with interrupt timer1
static volatile int timer1_driving_flag = 1;

/*
 * Private Section
 */

/********************************************************
 *				 FUNCTIONS DEFINTION
 ********************************************************/

/////////////////////////////////////////////////////////
//              SUPPORT FUNCTIONS
/////////////////////////////////////////////////////////

// Change to next sensing phase
static void drivingNextState() {
  // Check current sensing state
  switch (bos1901.state) {
    case DrivingState_A_init:
      bos1901.state = DrivingState_B_press_setup;
      break;
    case DrivingState_B_press_setup:
      bos1901.state = DrivingState_B_press;
      break;
    case DrivingState_B_press:
      bos1901.state = DrivingState_C_press_feedback;
      break;
    case DrivingState_C_press_feedback:
      bos1901.state = DrivingState_D_creep;
      break;
    case DrivingState_D_creep:
      bos1901.state = DrivingState_E_release_setup;
      break;
    case DrivingState_E_release_setup:
      bos1901.state = DrivingState_E_release;
      break;
    case DrivingState_E_release:
      bos1901.state = DrivingState_F_release_feedback;
      break;
    case DrivingState_F_release_feedback:
      bos1901.state = DrivingState_G_creep;
      break;
    case DrivingState_G_creep:
      bos1901.state = DrivingState_B_press_setup;
      break;
    default:
      bos1901.state = DrivingState_A_init;
      break;
  }
}

// Disable all timers
void utilsDisableAllUserIntr() {
  //  Timer1_Handler_Disable();
}

// Enable all timers
void utilsEnableAllUserIntr() {
  // Timer1_Handler_Enable();
}

// **********************************
// OTHER UTILS FUNCTIONS
// **********************************

// Convert value in volts to Amplitude FIFO code
int16_t utilsVolt2Amplitude(float volt) {
  int16_t amplitude = volt * 2047 / 3.6 / 31;

  return amplitude;
}

// Calculate Press and Release Feedback Waveforms
static void drivingCalculateWaveforms(void) {
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

// Writes data to SPI and reads returned data
uint16_t spiReadWriteReg(uint16_t data) { return SPI.transfer16(data); }

// Wait until BOS1901 internal FIFO is empty
static void drivingWaitFifoEmpty() {
  bool fifoempty = 0;

  // Set up broadcast to read IC_STATUS
  spiReadWriteReg(0x5617);  // Set BC = IC_STATUS

  // loop until FIFO is empty
  while (!fifoempty) {
    yield();
    delayMicroseconds(1E6 / SAMPLING_RATE / 2);  // wait half a PLAY period.
    uint16_t ic_status_reg =
        spiReadWriteReg(0xC000);             // dummy write, get IC_STATUS value
    fifoempty = (ic_status_reg >> 6) & 0x1;  // extract EMPTY value.
  }
}

// Software trimming element function
bool drivingSoftwareTrim() {
  uint16_t reg = 0;
  bool succeed = 0;

  // Need to disable the channel to do software trim
  spiReadWriteReg(0x5607);

  // Set the mode to read trim values from register. Value will be available in
  // TRIM_REGISTER and can be read from CONFIG BC
  spiReadWriteReg(0xE800);  // TRIMRW = 2, SDOBP = 0, TRIM_OSC = 0, TRIM_REG = 0

  // Sets the CONFIG BC to read TRIM register
  spiReadWriteReg(0x5707);

  // Read TRIM register and add 1 to TRIM_OSC
  reg = spiReadWriteReg(0x5707);
  uint16_t regMask =
      0x3 << 10 |
      0x3F << 3;  // mask to manipulate register paramters TRIMRW and TRIM_OSC
  uint16_t TRIM_OSC = ((reg >> 3) + 0x1) &
                      TRIM_OSC_MASK;  // gets TRIM_OSC value and increments 1
  succeed = (((reg >> 3) & TRIM_OSC_MASK) & TRIM_OSC_MAX_POS) !=
            TRIM_OSC_MAX_POS;  // check if reached maximum TRIM_OSC value
  reg &= ~regMask;             // set TRIM_OSC bits to 0
  reg |= 0x3 << 10;            // set TRIMRW to write mode
  reg |= TRIM_OSC << 3;        // set new TRIM_OSC value

  // Write the register with new TRIM_OSC parameter
  if (succeed) {
    spiReadWriteReg(reg);
  }

  // Reenable output and set BC back to IC_STATUS
  spiReadWriteReg(0x5617);
  // Clear fifo before exiting
  drivingWaitFifoEmpty();

  delay(1);  // temp

  return succeed;
}

// Trim internal oscillator to fit MCU sampling rate
static void drivingTrimming(void) {
  // Timer1_Handler_Disable();

  // Initialization
  // Reset IC
  spiReadWriteReg(0x56A7);  // Reset IC, set SDO broadcast to SENSE register to
                            // read VFEEDBACK, set PLAY sampling rate to 8kSPS.
  delayMicroseconds(50);

  // Set the mode to latch hardware fuses to trim block. Data are available in
  // TRIM register. Trimming will start at factory-trimmed value.
  spiReadWriteReg(0xE400);  // TRIMRW = 1, SDOBP = 0, TRIM_OSC = 0, TRIM_REG = 0

  // sets BC to IC_STATUS & enables output, PLAY = 8kSPS
  spiReadWriteReg(0x5617);

  // set the timer at 8.2kHz to have BOS1901 FIFO clear faster than 8kHz
  // Timer_1_WritePeriod(TIMER1_CLOCK_FREQ / TIMER1_FREQ_TRIM);
  // Timer1_Handler_Enable();

  // For each channel : perfom trimming
  for (int channel = 0; channel < NB_CHANNELS; channel++) {
    uint16 nbTry = 0;
    uint16 FifoSpacePrev = 0;
    uint16_t FifoSpace = 0;
    bool FifoFull = 0;
    bool FifoEmpty = 0;

    for (;;) {
      Serial.printf("inside loop timer1_driving_flag:%d\n",
                    timer1_driving_flag);
      // Timer1_Handler_Disable();

      // try to fill the IC FIFO
      if (timer1_driving_flag == 1)  // enter everytime timer1 expires (8.2kHz)
      {
        timer1_driving_flag = 0;

        // Send 0 V and try to fill the fifo
        uint16_t reg = spiReadWriteReg(0x0000);
        FifoSpace = reg & FIFO_SPACE_MASK;
        FifoFull = (reg >> 7) & 1;
        FifoEmpty = (reg >> 6) & 1;

        // First time the fifo will be the initial value (should be 0)
        if (nbTry == 0) {
          FifoSpacePrev = FifoSpace;
        }
        nbTry++;

        // If FIFO has less space than before, data is accumulating in the FIFO
        if ((FifoSpace < FifoSpacePrev && !FifoEmpty) || FifoFull) {
          if (drivingSoftwareTrim())  // increase oscillator speed
          {
            nbTry = 0;  // redo the loop until the trimming is OK
          } else        // error
          {
            // Led_1_Write(1);  // turn off green LED to show error.
            // while (1)
            //   ;
          }
        }

        // If you tried for a long time and did not succeed accumulating points
        // in the fifo
        if (nbTry > MAX_TRY) {
          break;  // exit trimming loop
        }
      }
      // Timer1_Handler_Enable();
    }
    spiReadWriteReg(0x5607);  // disable output once trimming is done.
  }

  // set the timer back to 8kHz
  // Timer_1_WritePeriod(TIMER1_CLOCK_FREQ / TIMER1_FREQ_DEFAULT);
  // Timer1_Handler_Enable();
}

/////////////////////////////////////////////////////////
//            SENSING PHASES FUNCTIONS
/////////////////////////////////////////////////////////

// Phase A - Sensing Initialization
// Single entry function - executed once when called
static void drivingInit() {
  // Reset
  spiReadWriteReg(0x56A7);  // Reset IC, set SDO broadcast to SENSE register to
                            // read VFEEDBACK, set PLAY sampling rate to 8kSPS.
  delay(50);

  // Get ADC offset value
  spiReadWriteReg(
      0x77E7);  // Set SENSE = 0 & VDD = 31 (5V supply), TI_RISE = default
  spiReadWriteReg(
      0x5697);  // Set OE = 1, uses same values as before for other parameters
  spiReadWriteReg(REFERENCE_ZERO);  // Write 0x0000 to set the output to 0V
  delay(50);                        // wait 50ms
  uint16_t reg_read = spiReadWriteReg(

      REFERENCE_ZERO);  // Write 0x0000 again to get the ADC offset value.
  bos1901.offset = reg_read & REG_READ_VFEEDBACK_MASK;  // save for later

  drivingNextState();  // to go next phase
}

// Entering Phase B - Press Sensing Setup
// Single entry function - executed once when called
static void drivingPressSetup() {
  spiReadWriteReg(0x7FE7);               // set SENSE = 1
  spiReadWriteReg(0x5697);               // Set BC = SENSE
  spiReadWriteReg(REFERENCE_PLUS_1LSB);  // write 0x0001 to set the
                                         // bridge to positive polarity

  drivingNextState();  // go to next phase
}

// Phase B - Press Sensing
// Multiple entry function - entered every time the timer expires
static void drivingPress() {
  int16_t vfeedback =
      spiReadWriteReg(REFERENCE_PLUS_1LSB) &
      REG_READ_VFEEDBACK_MASK;  // Write 0x0001 and read VFEEDBACK
  int16_t vsense = vfeedback - bos1901.offset;  // subtract the ADC offset value
  if (vsense >= PRESS_THRESHOLD)                // compare to threshold
  {
    bos1901.counter++;  // increase counter value to implement hold time check
    if (bos1901.counter >= PRESS_HOLD_TIME) {  // detection successful
      bos1901.counter = 0;                     // reset counter
      drivingNextState();                      // go to next phase
    }
  }
}

// Phase C - Press Feedback Waveform
// Multiple entry function - entered every time the timer expires
static void drivingPressFeedback() {
  if (bos1901.counter == 0)  // when starting this phase only
  {
    drivingWaitFifoEmpty();   // wait until BOS1901 internal FIFO is empty
                              // before sending the waveform points.
    spiReadWriteReg(0x77E7);  // set SENSE = 0 to drive the output
  }

  if (bos1901.counter < press_waveform_size)  // playing the waveform
  {
    spiReadWriteReg(

        press_waveform[bos1901.counter++]);  // Timer expired: send a
                                             // new point
  } else                                     // waveform reached its last point
  {
    bos1901.counter = 0;                 // cleanup
    spiReadWriteReg(press_waveform[0]);  // completing the waveform by playing
                                         // the initial point again.
    drivingNextState();                  // to go next phase
  }
}

// Phases D - Press Creep Stabilization
// Single entry function - executed once when called
static void drivingPressCreepStabilization() {
  spiReadWriteReg(REFERENCE_MINUS_1LSB);          // set FIFO to 0x0FFF
  delay(1000 * CREEP_HOLD_TIME / SAMPLING_RATE);  // wait defined time

  drivingNextState();  // to go next phase
}

// Entering Phase E - Release Sensing Setup
// Single entry function - executed once when called
static void drivingReleaseSetup() {
  spiReadWriteReg(0x7FE7);                // set SENSE = 1
  spiReadWriteReg(0x5697);                // Set BC = SENSE
  spiReadWriteReg(REFERENCE_MINUS_1LSB);  // write 0x0FFF to set the bridge to
                                          // negative polarity

  drivingNextState();  // go to next phase
}

// Phase E - Release Sensing
// Multiple entry function - entered every time the timer expires
static void drivingRelease() {
  int16_t vfeedback = spiReadWriteReg(REFERENCE_MINUS_1LSB) &
                      REG_READ_VFEEDBACK_MASK;  // Write 0x0FFF and read
                                                // VFEEDBACK (use on read value)
  int16_t vsense = vfeedback - bos1901.offset;  // subtract the ADC offset value
  if (vsense >=
      RELEASE_THRESHOLD)  // compare to threshold, increase counter value
  {
    bos1901.counter++;  // increase counter value to implement hold time check
    if (bos1901.counter >= RELEASE_HOLD_TIME) {  // detection successful
      bos1901.counter = 0;                       // reset counter
      drivingNextState();                        // go to next phase
    }
  }
}

// Phase F - Release Feedback Waveform
// Multiple entry function - entered every time the timer expires
static void drivingReleaseFeedback() {
  if (bos1901.counter == 0)  // when starting this phase only
  {
    drivingWaitFifoEmpty();   // wait until BOS1901 internal FIFO is empty
                              // before sending the waveform points.
    spiReadWriteReg(0x77E7);  // set SENSE = 0 to drive the output
  }

  if (bos1901.counter < release_waveform_size)  // playing the waveform
  {
    spiReadWriteReg(

        release_waveform[bos1901.counter++]);  // Timer expired: send a
                                               // new point
  } else  // waveform reached its last point
  {
    bos1901.counter = 0;                   // cleanup
    spiReadWriteReg(release_waveform[0]);  // completing the waveform by playing
                                           // the initial point again.
    drivingNextState();                    // to go next phase
  }
}

// Phases G - Release Creep Stabilization
// Single entry function - executed once when called
static void drivingReleaseCreepStabilization() {
  spiReadWriteReg(REFERENCE_MINUS_1LSB);          // set FIFO to 0x0FFF
  delay(1000 * CREEP_HOLD_TIME / SAMPLING_RATE);  // wait defined time

  drivingNextState();  // to go next phase
}

// Enter selected phass
// Multiple entry function - entered every time the timer expires
static void drivingEnterPhase() {
  // check current state and execute associated function
  switch (bos1901.state) {
    case DrivingState_A_init:
      Serial.printf("DrivingState_A_init\n");
      drivingInit();
      break;
    case DrivingState_B_press_setup:
      Serial.printf("DrivingState_B_press_setup\n");
      drivingPressSetup();
      break;
    case DrivingState_B_press:
      Serial.printf("DrivingState_B_press\n");
      drivingPress();
      break;
    case DrivingState_C_press_feedback:
      Serial.printf("DrivingState_C_press_feedback\n");
      drivingPressFeedback();
      break;
    case DrivingState_D_creep:
      Serial.printf("DrivingState_D_creep\n");
      drivingPressCreepStabilization();
      break;
    case DrivingState_E_release_setup:
      Serial.printf("DrivingState_E_release_setup\n");
      drivingReleaseSetup();
      break;
    case DrivingState_E_release:
      Serial.printf("DrivingState_E_release\n");
      drivingRelease();
      break;
    case DrivingState_F_release_feedback:
      Serial.printf("DrivingState_F_release_feedback\n");
      drivingReleaseFeedback();
      break;
    case DrivingState_G_creep:
      Serial.printf("DrivingState_G_creep\n");
      drivingReleaseCreepStabilization();
      break;
    default:
      Serial.printf("default\n");
      bos1901.state = DrivingState_A_init;
      drivingInit();
      break;
  }
}

// Executing sensing
static void drivingExecuteSensing() {
  // Enters every time the 8kHz timer expires
  // if (timer1_driving_flag == 1) {
  //   timer1_driving_flag = 0;  // reset flag for time-based operations

  // for each channel
  drivingEnterPhase();
  // }
}

/*
 * Public Section
 */

/********************************************************
 *				    MAIN EXAMPLE
 ********************************************************/
void sensingRealTimeDriveMain(void) {
  // Variables initialization
  // Serial.printf("drivingCalculateWaveforms(); begin\n");
  drivingCalculateWaveforms();
  // Serial.printf("drivingCalculateWaveforms(); end\n");

  // Init timer and interrupt
  // Timer_1_Start();
  // Timer1_Handler_StartEx(
  //     timer1_driving_ISR);  // initialize the custom defined ISR

  // Oscillator Trimming
  // Serial.printf("drivingTrimming(); begin\n");
  drivingTrimming();
  // Serial.printf("drivingTrimming(); end\n");

  // Infinite loop
  drivingExecuteSensing();
}

/*
 * Private Section
 */

/* [] END OF FILE */

#define CS 15

void setup() {
  Serial.begin(115200);
  Serial.printf("\n");
  delay(500);
  Serial.printf("Started\n");
  pinMode(CS, OUTPUT);
  SPI.begin();
  digitalWrite(CS, 0);

  int config = 0x0;
  config |= 0x1 << 5;  // Set the RST bit
  config |= 0x1 << 4;  // Set the OE bit
  uint8_t config_reg = 0x5;
  uint8_t sup_rise_reg = 0x7;
  SPI.transfer(config_reg);
  SPI.transfer(config);
  SPI.transfer(sup_rise_reg);
  SPI.transfer(0x0);
}

void loop() {
  // Serial.println("High wave");
  // uint8_t reference_reg = 0x0;
  // uint8_t high_data[] = {reference_reg, 0x0f, 0xff};
  // SPI.transfer(reference_reg);
  // SPI.transfer(0x0f);
  // SPI.transfer(0xff);
  // delay(10);
  // Serial.println("Low wave");
  // uint8_t low_data[] = {reference_reg, 0x00, 0x00};
  // SPI.transfer(reference_reg);
  // SPI.transfer(0x00);
  // SPI.transfer(0x00);
  // delay(10);
  sensingRealTimeDriveMain();
}
