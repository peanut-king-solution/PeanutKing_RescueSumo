#ifndef PeanutKing_RescueSumo_H
#define PeanutKing_RescueSumo_H

#include <Arduino.h>
// #include <Wire.h>

#include <Stepper.h>
#include <Servo.h>

#include <nI2C.h>
// #include <I2C.h>

// #include <math.h>
// #include <stdlib.h>

#define TCAADDR 0x70



// Defines /////////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define ADDRESS_DEFAULT 0b0101001

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (io_timeout > 0 && ((uint16_t)millis() - timeout_start_ms) > io_timeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)


/* =============================================================================
 *                             Color sensor TCS34725
 * ============================================================================= */

#define TCS34725_CONTROL        (0x0F)    /* Set the gain level for the sensor */
#define TCS34725_ID             (0x12)    /* 0x44 = TCS34721/TCS34725, 0x4D = TCS34723/TCS34727 */
#define TCS34725_ADDRESS        (0x29)

#define TCS34725_COMMAND_BIT      (0x80)

#define TCS34725_ENABLE           (0x00)
#define TCS34725_ENABLE_AIEN      (0x10)    /* RGBC Interrupt Enable */
#define TCS34725_ENABLE_WEN       (0x08)    /* Wait enable - Writing 1 activates the wait timer */
#define TCS34725_ENABLE_AEN       (0x02)    /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
#define TCS34725_ENABLE_PON       (0x01)    /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define TCS34725_ATIME            (0x01)    /* Integration time */
#define TCS34725_WTIME            (0x03)    /* Wait time (if TCS34725_ENABLE_WEN is asserted) */
#define TCS34725_WTIME_2_4MS      (0xFF)    /* WLONG0 = 2.4ms   WLONG1 = 0.029s */
#define TCS34725_WTIME_204MS      (0xAB)    /* WLONG0 = 204ms   WLONG1 = 2.45s  */
#define TCS34725_WTIME_614MS      (0x00)    /* WLONG0 = 614ms   WLONG1 = 7.4s   */
#define TCS34725_AILTL            (0x04)    /* Clear channel lower interrupt threshold */
#define TCS34725_AILTH            (0x05)
#define TCS34725_AIHTL            (0x06)    /* Clear channel upper interrupt threshold */
#define TCS34725_AIHTH            (0x07)
#define TCS34725_PERS             (0x0C)    /* Persistence register - basic SW filtering mechanism for interrupts */
#define TCS34725_PERS_NONE        (0b0000)  /* Every RGBC cycle generates an interrupt                                */
#define TCS34725_PERS_1_CYCLE     (0b0001)  /* 1 clean channel value outside threshold range generates an interrupt   */
#define TCS34725_PERS_2_CYCLE     (0b0010)  /* 2 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_3_CYCLE     (0b0011)  /* 3 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_5_CYCLE     (0b0100)  /* 5 clean channel values outside threshold range generates an interrupt  */
#define TCS34725_PERS_10_CYCLE    (0b0101)  /* 10 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_15_CYCLE    (0b0110)  /* 15 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_20_CYCLE    (0b0111)  /* 20 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_25_CYCLE    (0b1000)  /* 25 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_30_CYCLE    (0b1001)  /* 30 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_35_CYCLE    (0b1010)  /* 35 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_40_CYCLE    (0b1011)  /* 40 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_45_CYCLE    (0b1100)  /* 45 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_50_CYCLE    (0b1101)  /* 50 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_55_CYCLE    (0b1110)  /* 55 clean channel values outside threshold range generates an interrupt */
#define TCS34725_PERS_60_CYCLE    (0b1111)  /* 60 clean channel values outside threshold range generates an interrupt */
#define TCS34725_CONFIG           (0x0D)
#define TCS34725_CONFIG_WLONG     (0x02)    /* Choose between short and long (12x) wait times via TCS34725_WTIME */

#define TCS34725_STATUS           (0x13)
#define TCS34725_STATUS_AINT      (0x10)    /* RGBC Clean channel interrupt */
#define TCS34725_STATUS_AVALID    (0x01)    /* Indicates that the RGBC channels have completed an integration cycle */
#define TCS34725_CDATAL           (0x14)    /* Clear channel data */
#define TCS34725_CDATAH           (0x15)
#define TCS34725_RDATAL           (0x16)    /* Red channel data */
#define TCS34725_RDATAH           (0x17)
#define TCS34725_GDATAL           (0x18)    /* Green channel data */
#define TCS34725_GDATAH           (0x19)
#define TCS34725_BDATAL           (0x1A)    /* Blue channel data */
#define TCS34725_BDATAH           (0x1B)

typedef struct {
  uint16_t r;
  uint16_t g;
  uint16_t b;
} rgb_t;

typedef struct {
  uint16_t h;
  uint16_t s;
  uint16_t v;
} hsv_t;

typedef struct {
  volatile uint8_t *port;
  uint8_t mask;
  uint8_t numLEDs;
  uint8_t numBytes;
  uint8_t *pixels;     // Holds LED color values (3 or 4 bytes each)
} led_t;

typedef struct {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
  uint16_t k;
  uint16_t l;
} colorSensor_t;

typedef struct {
  Servo    s;
  uint32_t t;
  int16_t  v;
} servos_t;

typedef struct {
  Stepper  s;
  uint32_t t;
  int16_t  v;
} stepper_t;

typedef enum { front = 0, left, right, back } sensorNum;

typedef enum {
  black=0,  white,   grey,
  red,      green,   blue, 
  yellow,   cyan,    magenta
} color_t;

enum regAddr {
  SYSRANGE_START                              = 0x00,

  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,

  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

  MSRC_CONFIG_CONTROL                         = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,

  OSC_CALIBRATE_VAL                           = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

typedef enum {
  TCS34725_INTEGRATIONTIME_2_4MS = 0xFF,  /**<  2.4ms - 1 cycle    - Max Count: 1024  */
  TCS34725_INTEGRATIONTIME_24MS  = 0xF6,  /**<  24ms  - 10 cycles  - Max Count: 10240 */
  TCS34725_INTEGRATIONTIME_50MS  = 0xEB,  /**<  50ms  - 20 cycles  - Max Count: 20480 */
  TCS34725_INTEGRATIONTIME_101MS = 0xD5,  /**<  101ms - 42 cycles  - Max Count: 43008 */
  TCS34725_INTEGRATIONTIME_154MS = 0xC0,  /**<  154ms - 64 cycles  - Max Count: 65535 */
  TCS34725_INTEGRATIONTIME_700MS = 0x00   /**<  700ms - 256 cycles - Max Count: 65535 */
} tcs34725IntegrationTime_t;

typedef enum {
  TCS34725_GAIN_1X               = 0x00,   /**<  No gain  */
  TCS34725_GAIN_4X               = 0x01,   /**<  4x gain  */
  TCS34725_GAIN_16X              = 0x02,   /**<  16x gain */
  TCS34725_GAIN_60X              = 0x03    /**<  60x gain */
} tcs34725Gain_t;

const float pi = 3.1415926535897;


class PeanutKing_RescueSumo {
 public:
  // Constructor 
  PeanutKing_RescueSumo(void);
  PeanutKing_RescueSumo(uint8_t);

/* =============================================================================
 *                              Functions
 * ============================================================================= */
  void
    init(void),

    ledShow(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t, uint8_t = 0),
    ledSetPixels(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t),
    ledClear(void),
    ledUpdate(uint8_t = 0);
  void 
    ledSetup(uint8_t, uint8_t, uint8_t);

  void motorSet(uint8_t motor_no, int16_t speed);
  
  hsv_t rgb2hsv(rgb_t in);


  float rawCompass(int8_t addr, int8_t cmd);
  uint16_t compassRead(void);

  void tcaselect(uint8_t i);
  void colorSensorInit(uint8_t i);
  void laserSensorInit(uint8_t i);

  // motors
  void setStepperSpeed(int s);
  void servoMove(uint8_t i, int16_t val);
  void stepperMove(uint8_t i, int32_t val);


  // tcs34725
 // bool tcs34725Begin(uint8_t addr, TwoWire *theWire);
  bool tcs34725Begin(uint8_t addr);
  bool tcs34725Begin();
  bool tcs34725Init();

  void setIntegrationTime(tcs34725IntegrationTime_t it);
  void setGain(tcs34725Gain_t gain);
  void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
  void getRGB(float *r, float *g, float *b);
  void getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
  uint16_t calculateColorTemperature(uint16_t r, uint16_t g, uint16_t b);
  uint16_t calculateColorTemperature_dn40(uint16_t r, uint16_t g, uint16_t b, uint16_t c);
  uint16_t calculateLux(uint16_t r, uint16_t g, uint16_t b);
  void write8(uint8_t reg, uint32_t value);
  uint8_t read8(uint8_t reg);
  uint16_t read16(uint8_t reg);
  void setInterrupt(boolean flag);
  void clearInterrupt();
  void setIntLimits(uint16_t l, uint16_t h);
  void enable();
  void disable();

  // VL53L0X

  bool VL53L0XInit(bool io_2v8 = true);

  void writeReg(uint8_t reg, uint8_t value);
  void writeReg16Bit(uint8_t reg, uint16_t value);
  void writeReg32Bit(uint8_t reg, uint32_t value);
  uint8_t readReg(uint8_t reg);
  uint16_t readReg16Bit(uint8_t reg);
  uint32_t readReg32Bit(uint8_t reg);

  void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
  void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

  bool setSignalRateLimit(float limit_Mcps);
  float getSignalRateLimit(void);

  bool setMeasurementTimingBudget(uint32_t budget_us);
  uint32_t getMeasurementTimingBudget(void);

  bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
  uint8_t getVcselPulsePeriod(vcselPeriodType type);

  void startContinuous(uint32_t period_ms = 0);
  void stopContinuous(void);
  uint16_t readRangeContinuousMillimeters(void);
  uint16_t readRangeSingleMillimeters(void);

  inline void setTimeout(uint16_t timeout) { io_timeout = timeout; }
  inline uint16_t getTimeout(void) { return io_timeout; }
  bool timeoutOccurred(void);


  // Constant  ===========================================================

  const uint8_t
    dirPin[2]   = {8, 13},
    stepPin[2]  = {9, 12},
    servoPin[2] = {11, 10};
  
  const int8_t  compass_address = 8;

  const uint8_t
    GET_READING = 0x55,
    SET_HOME    = 0x54;

  CI2C::Handle i2cHandleCompass;
  CI2C::Handle i2cHandleSelector;
  CI2C::Handle i2cHandleLaser;
  CI2C::Handle i2cHandleColor;
// change this to fit the number of steps per revolution
  int stepsPerRevolution = 25600;
  int stepperSpeed = 100;
/*
  // motors
  stepper_t *stepperMotor;
  servos_t  *servoMotor;  // create servo object to control a servo

  // motors
  stepperMotor = new stepper_t{ 
    {Stepper(stepsPerRevolution, dirPin[0], stepPin[1]), 0, 0}, 
    {Stepper(stepsPerRevolution, dirPin[1], stepPin[1]), 0, 0} 
  };
  servoMotor = new servos_t[2];  // create servo object to control a servo
*/

  // motors
  stepper_t stepperMotor[2] = {
    {Stepper(stepsPerRevolution, dirPin[0], stepPin[0]), 0, 0}, 
    {Stepper(stepsPerRevolution, dirPin[1], stepPin[1]), 0, 0} 
  };
  servos_t servoMotor[2];  // create servo object to control a servo


  led_t
    leds[2];


  // tcs34725

  // VL53L0X
    uint8_t last_status; // status of last I2C transmission
    
  private:
    boolean _tcs34725Initialised;
    tcs34725Gain_t _tcs34725Gain;
    tcs34725IntegrationTime_t _tcs34725IntegrationTime;

    // TCC: Target CentreCheck
    // MSRC: Minimum Signal Rate Check
    // DSS: Dynamic Spad Selection

    struct SequenceStepEnables {
      boolean tcc, msrc, dss, pre_range, final_range;
    };

    struct SequenceStepTimeouts {
      uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

      uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
      uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
    };

    uint16_t io_timeout;
    bool did_timeout;
    uint16_t timeout_start_ms;

    uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
    uint32_t measurement_timing_budget_us;

    bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

    void getSequenceStepEnables(SequenceStepEnables * enables);
    void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

    bool performSingleRefCalibration(uint8_t vhv_init_byte);

    static uint16_t decodeTimeout(uint16_t value);
    static uint16_t encodeTimeout(uint16_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
};

#endif


