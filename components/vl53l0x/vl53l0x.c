// VL53L0X control
// Copyright © 2019 Adrian Kennard, Andrews & Arnold Ltd. See LICENCE file for details. GPL 3.0
// Modified by Tomas Hak to support ESP-IDF I2C driver v2
// Based on https://github.com/pololu/vl53l0x-arduino

#include "vl53l0x.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <i2cdev.h>

#define TIMEOUT	(10/portTICK_PERIOD_MS) // I2C command timeout

//#define CONFIG_VL53L0X_DEBUG
#ifdef	CONFIG_VL53L0X_DEBUG
#define VL53L0X_LOG   ESP_LOGI        // Set to allow I2C logging
#endif

#ifndef VL53L0X_LOG
#define VL53L0X_LOG(tag,...) err=err;
#endif

#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)
#define CHECK_ARG_LOGE(VAL, msg, ...) do { if (!(VAL)) { ESP_LOGE(TAG_VL53L0X, msg, ## __VA_ARGS__); return ESP_ERR_INVALID_ARG; } } while (0)
#define CHECK(x) do { esp_err_t __; if ((__ = x) != ESP_OK) return __; } while (0)

enum
{
   SYSRANGE_START = 0x00,

   SYSTEM_THRESH_HIGH = 0x0C,
   SYSTEM_THRESH_LOW = 0x0E,

   SYSTEM_SEQUENCE_CONFIG = 0x01,
   SYSTEM_RANGE_CONFIG = 0x09,
   SYSTEM_INTERMEASUREMENT_PERIOD = 0x04,

   SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A,

   GPIO_HV_MUX_ACTIVE_HIGH = 0x84,

   SYSTEM_INTERRUPT_CLEAR = 0x0B,

   RESULT_INTERRUPT_STATUS = 0x13,
   RESULT_RANGE_STATUS = 0x14,

   RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN = 0xBC,
   RESULT_CORE_RANGING_TOTAL_EVENTS_RTN = 0xC0,
   RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF = 0xD0,
   RESULT_CORE_RANGING_TOTAL_EVENTS_REF = 0xD4,
   RESULT_PEAK_SIGNAL_RATE_REF = 0xB6,

   ALGO_PART_TO_PART_RANGE_OFFSET_MM = 0x28,

   MSRC_CONFIG_CONTROL = 0x60,

   PRE_RANGE_CONFIG_MIN_SNR = 0x27,
   PRE_RANGE_CONFIG_VALID_PHASE_LOW = 0x56,
   PRE_RANGE_CONFIG_VALID_PHASE_HIGH = 0x57,
   PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT = 0x64,

   FINAL_RANGE_CONFIG_MIN_SNR = 0x67,
   FINAL_RANGE_CONFIG_VALID_PHASE_LOW = 0x47,
   FINAL_RANGE_CONFIG_VALID_PHASE_HIGH = 0x48,
   FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

   PRE_RANGE_CONFIG_SIGMA_THRESH_HI = 0x61,
   PRE_RANGE_CONFIG_SIGMA_THRESH_LO = 0x62,

   PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51,
   PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x52,

   SYSTEM_HISTOGRAM_BIN = 0x81,
   HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT = 0x33,
   HISTOGRAM_CONFIG_READOUT_CTRL = 0x55,

   FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71,
   FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO = 0x72,
   CROSSTALK_COMPENSATION_PEAK_RATE_MCPS = 0x20,

   MSRC_CONFIG_TIMEOUT_MACROP = 0x46,

   I2C_SLAVE_DEVICE_ADDRESS = 0x8A,

   SOFT_RESET_GO2_SOFT_RESET_N = 0xBF,
   IDENTIFICATION_MODEL_ID = 0xC0,
   IDENTIFICATION_REVISION_ID = 0xC2,

   OSC_CALIBRATE_VAL = 0xF8,

   GLOBAL_CONFIG_VCSEL_WIDTH = 0x32,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_1 = 0xB1,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_2 = 0xB2,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_3 = 0xB3,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_4 = 0xB4,
   GLOBAL_CONFIG_SPAD_ENABLES_REF_5 = 0xB5,

   GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6,
   DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E,
   DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F,
   POWER_MANAGEMENT_GO1_POWER_FORCE = 0x80,

   VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV = 0x89,

   ALGO_PHASECAL_LIM = 0x30,
   ALGO_PHASECAL_CONFIG_TIMEOUT = 0x30,
};

struct vl53l0x_s
{
   i2c_dev_t dev;

   gpio_num_t xshut;
   uint8_t io_2v8:1;

   vl53l0x_error_t err_type;
};

typedef struct
{
   uint8_t tcc:1;
   uint8_t msrc:1;
   uint8_t dss:1;
   uint8_t pre_range:1;
   uint8_t final_range:1;
} SequenceStepEnables;

typedef struct
{
   uint16_t pre_range_vcsel_period_pclks,
     final_range_vcsel_period_pclks;
   uint16_t msrc_dss_tcc_mclks,
     pre_range_mclks,
     final_range_mclks;
   uint32_t msrc_dss_tcc_us,
     pre_range_us,
     final_range_us;
}
SequenceStepTimeouts;

static uint8_t stop_variable;
static uint16_t timeout_start_ms;
static uint32_t measurement_timing_budget_us;
#define millis() (esp_timer_get_time()/1000LL)

// Record the current time to check an upcoming timeout against
#define startTimeout() (timeout_start_ms = millis())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired(io_timeout) ((io_timeout) > 0 && ((millis() - timeout_start_ms) > (io_timeout)))

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

#define SET_ERROR(v, err) (v->err_type = err)


inline static esp_err_t write_reg(i2c_dev_t *dev, uint8_t reg, uint8_t val)
{
   return i2c_dev_write_reg(dev, reg, &val, 1);
}

inline static esp_err_t write_reg16bit(i2c_dev_t *dev, uint8_t reg, uint16_t val)
{
   uint8_t u8val[2];
   u8val[0] = (uint8_t)(val >> 8);
   u8val[1] = (uint8_t)(val);
   return i2c_dev_write_reg(dev, reg, &u8val, 2);
}

inline static esp_err_t write_reg32bit(i2c_dev_t *dev, uint8_t reg, uint32_t val)
{
   uint8_t u8val[4];
   u8val[0] = (uint8_t)(val >> 24);
   u8val[1] = (uint8_t)(val >> 16);
   u8val[2] = (uint8_t)(val >> 8);
   u8val[3] = (uint8_t)(val);
   return i2c_dev_write_reg(dev, reg, &u8val, 4);
}

inline static esp_err_t write_reg_multi(i2c_dev_t *dev, uint8_t reg, uint8_t *val, size_t count)
{
   return i2c_dev_write_reg(dev, reg, val, count);
}

inline static esp_err_t read_reg(i2c_dev_t *dev, uint8_t reg, uint8_t *data)
{
   return i2c_dev_read_reg(dev, reg, data, 1);
}

inline static esp_err_t read_reg16bit(i2c_dev_t *dev, uint8_t reg, uint16_t *data)
{
   uint8_t u8data[2];
   esp_err_t err = i2c_dev_read_reg(dev, reg, u8data, 2);
   *data = (u8data[0] << 8) | u8data[1];
   return err;
}

inline static esp_err_t read_reg_multi(i2c_dev_t *dev, uint8_t reg, uint8_t *data, size_t count)
{
   return i2c_dev_read_reg(dev, reg, data, count);
}

static esp_err_t update_reg(i2c_dev_t *dev, uint8_t reg, uint8_t mask, uint8_t val)
{
    uint8_t v;

    CHECK(read_reg(dev, reg, &v));
    v = (v & ~mask) | val;
    CHECK(write_reg(dev, reg, v));

    return ESP_OK;
}

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
static uint16_t decodeTimeout(uint16_t reg_val)
{
   // format: "(LSByte * 2^MSByte) + 1"
   return (uint16_t) ((reg_val & 0x00FF) << (uint16_t) ((reg_val & 0xFF00) >> 8)) + 1;
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod (vcsel_period_pclks);
   return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// based on VL53L0X_perform_single_ref_calibration()
esp_err_t performSingleRefCalibration(vl53l0x_t *v, uint8_t vhv_init_byte)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   I2C_DEV_TAKE_MUTEX(dev);

   I2C_DEV_CHECK(dev, write_reg(dev, SYSRANGE_START, 0x01 | vhv_init_byte));      // VL53L0X_REG_SYSRANGE_MODE_START_STOP
   uint8_t data = 0xFF;
   startTimeout();
   do {
      I2C_DEV_CHECK(dev, read_reg(dev, RESULT_INTERRUPT_STATUS, &data));
      if (checkTimeoutExpired(1000)) {
         I2C_DEV_GIVE_MUTEX(dev);
         SET_ERROR(v, CAL_timeout);
         return ESP_FAIL;
      }
   }
   while ((data & 0x07) == 0);
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, SYSRANGE_START, 0x00));

   I2C_DEV_GIVE_MUTEX(dev);

   return ESP_OK;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
   // format: "(LSByte * 2^MSByte) + 1"

   uint32_t ls_byte = 0;
   uint16_t ms_byte = 0;

   if (timeout_mclks > 0)
   {
      ls_byte = timeout_mclks - 1;

      while ((ls_byte & 0xFFFFFF00) > 0)
      {
         ls_byte >>= 1;
         ms_byte++;
      }

      return (ms_byte << 8) | (ls_byte & 0xFF);
   } else
   {
      return 0;
   }
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
static esp_err_t getSequenceStepEnables(i2c_dev_t *dev, SequenceStepEnables * enables)
{
   CHECK_ARG(dev);

   uint8_t sequence_config = 0x00;

   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, read_reg(dev, SYSTEM_SEQUENCE_CONFIG, &sequence_config));
   I2C_DEV_GIVE_MUTEX(dev);

   enables->tcc = (sequence_config >> 4) & 0x1;
   enables->dss = (sequence_config >> 3) & 0x1;
   enables->msrc = (sequence_config >> 2) & 0x1;
   enables->pre_range = (sequence_config >> 6) & 0x1;
   enables->final_range = (sequence_config >> 7) & 0x1;

   return ESP_OK;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
static esp_err_t getSequenceStepTimeouts(i2c_dev_t *dev, SequenceStepEnables const *enables, SequenceStepTimeouts * timeouts)
{
   CHECK_ARG(dev);

   uint8_t u8data = 0x00;
   uint16_t u16data = 0x0000;

   I2C_DEV_TAKE_MUTEX(dev);
   vl53l0x_getVcselPulsePeriod(dev, VcselPeriodPreRange, &timeouts->pre_range_vcsel_period_pclks);

   I2C_DEV_CHECK(dev, read_reg(dev, MSRC_CONFIG_TIMEOUT_MACROP, &u8data));
   u8data += 1;
   timeouts->msrc_dss_tcc_mclks = u8data;
   timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

   I2C_DEV_CHECK(dev, read_reg16bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &u16data));
   timeouts->pre_range_mclks = decodeTimeout(u16data);
   timeouts->pre_range_us = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

   vl53l0x_getVcselPulsePeriod(dev, VcselPeriodFinalRange, &timeouts->final_range_vcsel_period_pclks);

   I2C_DEV_CHECK(dev, read_reg16bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &u16data));
   timeouts->final_range_mclks = decodeTimeout(u16data);

   I2C_DEV_GIVE_MUTEX(dev);

   if (enables->pre_range)
   {
      timeouts->final_range_mclks -= timeouts->pre_range_mclks;
   }

   timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);

   return ESP_OK;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
   uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

   return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
esp_err_t vl53l0x_setSignalRateLimit(vl53l0x_t *v, float limit_Mcps)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   if (limit_Mcps < 0 || limit_Mcps > 511.99) {
      SET_ERROR(v, Bad_rate);
      return ESP_FAIL;
   }

   I2C_DEV_TAKE_MUTEX(dev);
   
   // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
   I2C_DEV_CHECK(dev, write_reg16bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7)));
   
   I2C_DEV_GIVE_MUTEX(dev);

   return ESP_OK;
}

esp_err_t vl53l0x_getSignalRateLimit(i2c_dev_t *dev, float *sig_rate_lim)
{
   // init the value in case of anything goes wrong (for example I2C write will fail)
   *sig_rate_lim = 0.0f;
   uint16_t tmp = 0x0000;

   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, read_reg16bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, &tmp));
   I2C_DEV_GIVE_MUTEX(dev);

   *sig_rate_lim = (float)tmp;
   *sig_rate_lim /= (1 << 7);

   return ESP_OK;
}

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
esp_err_t vl53l0x_getSpadInfo(vl53l0x_t *v, uint8_t *count, int *type_is_aperture)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   uint8_t tmp;
   uint8_t SPAD_data;

   I2C_DEV_TAKE_MUTEX(dev);

   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x06));
   I2C_DEV_CHECK(dev, update_reg(dev, 0x83, 0x00, 0x04));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x07));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x81, 0x01));

   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x01));

   I2C_DEV_CHECK(dev, write_reg(dev, 0x94, 0x6b));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x83, 0x00));

   startTimeout ();
   do {
      I2C_DEV_CHECK(dev, read_reg(dev, 0x83, &SPAD_data));
      if (checkTimeoutExpired (1000)) {
         I2C_DEV_GIVE_MUTEX(dev);
         SET_ERROR(v, SPAD_timeout);
         return ESP_FAIL;
      }
   }
   while (SPAD_data == 0x00);

   I2C_DEV_CHECK(dev, write_reg(dev, 0x83, 0x01));
   I2C_DEV_CHECK(dev, read_reg(dev, 0x92, &tmp));

   *count = tmp & 0x7f;
   *type_is_aperture = (tmp >> 7) & 0x01;

   I2C_DEV_CHECK(dev, write_reg(dev, 0x81, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x06));
   I2C_DEV_CHECK(dev, update_reg(dev, 0x83, 0x04, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x01));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x00));

   I2C_DEV_GIVE_MUTEX(dev);

   return ESP_OK;
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
esp_err_t vl53l0x_setMeasurementTimingBudget(vl53l0x_t *v, uint32_t budget_us)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   uint16_t const StartOverhead = 1320; // note that this is different than the value in get_
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   uint32_t const MinTimingBudget = 20000;

   if (budget_us < MinTimingBudget) {
      SET_ERROR(v, Low_budget);
      return ESP_FAIL;
   }

   uint32_t used_budget_us = StartOverhead + EndOverhead;

   I2C_DEV_CHECK(dev, getSequenceStepEnables(dev, &enables));
   I2C_DEV_CHECK(dev, getSequenceStepTimeouts(dev, &enables, &timeouts));

   if (enables.tcc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);

   if (enables.dss)
      used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   else if (enables.msrc)
      used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);

   if (enables.pre_range)
      used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);

   if (enables.final_range)
   {
      used_budget_us += FinalRangeOverhead;

      // "Note that the final range timeout is determined by the timing
      // budget and the sum of all other timeouts within the sequence.
      // If there is no room for the final range timeout, then an error
      // will be set. Otherwise the remaining time will be applied to
      // the final range."

      if (used_budget_us > budget_us) {
         SET_ERROR(v, High_budget);
         return ESP_FAIL;
      }

      uint32_t final_range_timeout_us = budget_us - used_budget_us;

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      uint16_t
         final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

      if (enables.pre_range)
         final_range_timeout_mclks += timeouts.pre_range_mclks;

      I2C_DEV_TAKE_MUTEX(dev);
      I2C_DEV_CHECK(dev, write_reg16bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks)));
      I2C_DEV_GIVE_MUTEX(dev);
      // set_sequence_step_timeout() end

      measurement_timing_budget_us = budget_us; // store for internal reuse
   }
   return ESP_OK;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
esp_err_t vl53l0x_getMeasurementTimingBudget(i2c_dev_t *dev, uint32_t *budget_us)
{
   CHECK_ARG(dev);

   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   uint16_t const StartOverhead = 1910; // note that this is different than the value in set_
   uint16_t const EndOverhead = 960;
   uint16_t const MsrcOverhead = 660;
   uint16_t const TccOverhead = 590;
   uint16_t const DssOverhead = 690;
   uint16_t const PreRangeOverhead = 660;
   uint16_t const FinalRangeOverhead = 550;

   // "Start and end overhead times always present"
   *budget_us = StartOverhead + EndOverhead;

   I2C_DEV_CHECK(dev, getSequenceStepEnables(dev, &enables));
   I2C_DEV_CHECK(dev, getSequenceStepTimeouts(dev, &enables, &timeouts));

   if (enables.tcc)
   {
      *budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
   }

   if (enables.dss)
   {
      *budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
   } else if (enables.msrc)
   {
      *budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
   }

   if (enables.pre_range)
   {
      *budget_us += (timeouts.pre_range_us + PreRangeOverhead);
   }

   if (enables.final_range)
   {
      *budget_us += (timeouts.final_range_us + FinalRangeOverhead);
   }

   measurement_timing_budget_us = *budget_us;    // store for internal reuse

   return ESP_OK;
}


// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
esp_err_t vl53l0x_setVcselPulsePeriod(vl53l0x_t *v, vl53l0x_vcselPeriodType type, uint8_t period_pclks)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

   SequenceStepEnables enables;
   SequenceStepTimeouts timeouts;

   getSequenceStepEnables(dev, &enables);
   getSequenceStepTimeouts(dev, &enables, &timeouts);

   // "Apply specific settings for the requested clock period"
   // "Re-calculate and apply timeouts, in macro periods"

   // "When the VCSEL period for the pre or final range is changed,
   // the corresponding timeout must be read from the device using
   // the current VCSEL period, then the new VCSEL period can be
   // applied. The timeout then must be written back to the device
   // using the new VCSEL period.
   //
   // For the MSRC timeout, the same applies - this timeout being
   // dependant on the pre-range vcsel period."

   I2C_DEV_TAKE_MUTEX(dev);

   if (type == VcselPeriodPreRange) {
      // "Set phase check limits"
      switch (period_pclks) {
         case 12:
            I2C_DEV_CHECK(dev, write_reg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18));
            break;

         case 14:
            I2C_DEV_CHECK(dev, write_reg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30));
            break;

         case 16:
            I2C_DEV_CHECK(dev, write_reg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40));
            break;

         case 18:
            I2C_DEV_CHECK(dev, write_reg(dev, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50));
            break;

         default:
            // invalid period
            I2C_DEV_GIVE_MUTEX(dev);
            SET_ERROR(v, VCSEL_prerange_period_invalid);
            return ESP_FAIL;
      }
      I2C_DEV_CHECK(dev, write_reg(dev, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));

      // apply new VCSEL period
      I2C_DEV_CHECK(dev, write_reg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg));

      // update timeouts

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

      uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

      
      I2C_DEV_CHECK(dev, write_reg16bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks)));

      // set_sequence_step_timeout() end

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

      uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

      I2C_DEV_CHECK(dev, write_reg(dev, MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1)));

      // set_sequence_step_timeout() end
   } else if (type == VcselPeriodFinalRange)
   {
      switch (period_pclks) {
         case 8:
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10));
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
            I2C_DEV_CHECK(dev, write_reg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x02));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_LIM, 0x30));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
            break;

         case 10:
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28));
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
            I2C_DEV_CHECK(dev, write_reg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_LIM, 0x20));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
            break;

         case 12:
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38));
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
            I2C_DEV_CHECK(dev, write_reg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_LIM, 0x20));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
            break;

         case 14:
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48));
            I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08));
            I2C_DEV_CHECK(dev, write_reg(dev, GLOBAL_CONFIG_VCSEL_WIDTH, 0x03));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
            I2C_DEV_CHECK(dev, write_reg(dev, ALGO_PHASECAL_LIM, 0x20));
            I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
            break;

         default:
            // invalid period
            I2C_DEV_GIVE_MUTEX(dev);
            SET_ERROR(v, VCSEL_finalrange_period_invalid);
            return ESP_FAIL;
      }

      // apply new VCSEL period
      I2C_DEV_CHECK(dev, write_reg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg));

      // update timeouts

      // set_sequence_step_timeout() begin
      // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

      // "For the final range timeout, the pre-range timeout
      //  must be added. To do this both final and pre-range
      //  timeouts must be expressed in macro periods MClks
      //  because they have different vcsel periods."

      uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

      if (enables.pre_range)
         new_final_range_timeout_mclks += timeouts.pre_range_mclks;

      
      I2C_DEV_CHECK(dev, write_reg16bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_final_range_timeout_mclks)));

      // set_sequence_step_timeout end
   } else {
      // Invalid type
      I2C_DEV_GIVE_MUTEX(dev);
      SET_ERROR(v, Invalid_type);
      return ESP_FAIL;
   }

   I2C_DEV_GIVE_MUTEX(dev);

   // "Finally, the timing budget must be re-applied"
   
   I2C_DEV_CHECK(dev, vl53l0x_setMeasurementTimingBudget(v, measurement_timing_budget_us));

   // "Perform the phase calibration. This is needed after changing on vcsel period."
   // VL53L0X_perform_phase_calibration() begin
   I2C_DEV_TAKE_MUTEX(dev);
   uint8_t sequence_config = 0x00;
   I2C_DEV_CHECK(dev, read_reg(dev, SYSTEM_SEQUENCE_CONFIG, &sequence_config));
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02));
   I2C_DEV_GIVE_MUTEX(dev);

   performSingleRefCalibration(v, 0x0);

   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, sequence_config));
   I2C_DEV_GIVE_MUTEX(dev);
   // VL53L0X_perform_phase_calibration() end

   return ESP_OK;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
esp_err_t vl53l0x_getVcselPulsePeriod(i2c_dev_t *dev, vl53l0x_vcselPeriodType type, uint16_t *VcselPeriod)
{
   CHECK_ARG(dev);
   /* WARNING - I2C MUTEX handled above this function */

   uint8_t u8data = 0x00;

   switch (type) {
      case VcselPeriodPreRange:
         I2C_DEV_CHECK(dev, read_reg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD, &u8data));
         *VcselPeriod = decodeVcselPeriod(u8data);
         break;

      case VcselPeriodFinalRange:
         I2C_DEV_CHECK(dev, read_reg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &u8data));
         *VcselPeriod = decodeVcselPeriod(u8data);
         break;

      default:
         break;
   }

   return ESP_OK;
}


// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
esp_err_t vl53l0x_startContinuous(vl53l0x_t *v, uint32_t period_ms)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   I2C_DEV_TAKE_MUTEX(dev);

   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x91, stop_variable));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x00));

   if (period_ms != 0)
   {
      // continuous timed mode

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

      uint16_t osc_calibrate_val = 0x0000;
      I2C_DEV_CHECK(dev, read_reg16bit(dev, OSC_CALIBRATE_VAL, &osc_calibrate_val));

      if (osc_calibrate_val != 0)
      {
         period_ms *= osc_calibrate_val;
      }

      
      I2C_DEV_CHECK(dev, write_reg32bit(dev, SYSTEM_INTERMEASUREMENT_PERIOD, period_ms));

      // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end
      
      I2C_DEV_CHECK(dev, write_reg(dev, SYSRANGE_START, 0x04));   // VL53L0X_REG_SYSRANGE_MODE_TIMED
   } else
   {
      // continuous back-to-back mode
      I2C_DEV_CHECK(dev, write_reg(dev, SYSRANGE_START, 0x02));   // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
   }

   I2C_DEV_GIVE_MUTEX(dev);

   return ESP_OK;
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
esp_err_t vl53l0x_stopContinuous(vl53l0x_t *v)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   I2C_DEV_TAKE_MUTEX(dev);

   I2C_DEV_CHECK(dev, write_reg(dev, SYSRANGE_START, 0x01)); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x91, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));

   I2C_DEV_GIVE_MUTEX(dev);

   return ESP_OK;
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
esp_err_t vl53l0x_readRangeContinuousMillimeters(vl53l0x_t *v, uint16_t *range_value)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   uint8_t inter_sts_reg_value = 0x00;

   // init the value in case of anything goes wrong (for example I2C write will fail)
   *range_value = 0xFFFF;

   I2C_DEV_TAKE_MUTEX(dev);

   startTimeout ();
   do {
      I2C_DEV_CHECK(dev, read_reg(dev, RESULT_INTERRUPT_STATUS, &inter_sts_reg_value));
      if (checkTimeoutExpired(400)) {
         I2C_DEV_GIVE_MUTEX(dev);
         SET_ERROR(v, Interrupt_status_timeout);
         return ESP_FAIL;
      }
   }
   while ((inter_sts_reg_value  & 0x07) == 0);

   // assumptions: Linearity Corrective Gain is 1000 (default);
   // fractional ranging is not enabled

   I2C_DEV_CHECK(dev, read_reg16bit(dev, RESULT_RANGE_STATUS + 10, range_value));
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01));

   I2C_DEV_GIVE_MUTEX(dev);

   return ESP_OK;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
esp_err_t vl53l0x_readRangeSingleMillimeters(vl53l0x_t *v, uint16_t *range_value)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   // init the value in case of anything goes wrong (for example I2C write will fail)
   *range_value = 0xFFFF;

   I2C_DEV_TAKE_MUTEX(dev);
   
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x91, stop_variable));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, SYSRANGE_START, 0x01));

   // "Wait until start bit has been cleared"
   uint8_t sysrange_start_reg = 0x00;
   startTimeout();
   do {
      I2C_DEV_CHECK(dev, read_reg(dev, SYSRANGE_START, &sysrange_start_reg));
      if (checkTimeoutExpired(1000)) {
         I2C_DEV_GIVE_MUTEX(dev);
         SET_ERROR(v, Start_bit_clearing_timeout);
         return ESP_FAIL;
      }
   }
   while (sysrange_start_reg & 0x01);

   I2C_DEV_GIVE_MUTEX(dev);

   I2C_DEV_CHECK(dev, vl53l0x_readRangeContinuousMillimeters(v, range_value));

   return ESP_OK;
}


vl53l0x_t* vl53l0x_config(i2c_dev_t *dev, gpio_num_t xshut_pin, uint8_t io_2v8)
{
   if(!dev)
      return NULL;

   if (xshut_pin >= 0 && !GPIO_IS_VALID_OUTPUT_GPIO(xshut_pin))
      return NULL;

   if (xshut_pin >= 0) {
      gpio_reset_pin(xshut_pin);
      gpio_set_drive_capability(xshut_pin, GPIO_DRIVE_CAP_3);
      gpio_set_direction(xshut_pin, GPIO_MODE_OUTPUT);

      // XSHUT pin handling (power control)
      gpio_set_level(xshut_pin, 0);     // Chip Off
      usleep(100000);
      gpio_set_level(xshut_pin, 1);     // Chip On
      usleep(10000);           // Plenty of time to boot (datasheet says 1.2ms)
   }

   vl53l0x_t *v = malloc(sizeof(*v));
   if (!v)
      return NULL;

   memset(v, 0, sizeof(*v));
   v->dev = *dev;
   v->xshut = xshut_pin;
   v->io_2v8 = io_2v8;
   v->err_type = Device_unresponsive;     // initial value

   return v;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
esp_err_t vl53l0x_init(vl53l0x_t *v)
{
   CHECK_ARG(v);

   i2c_dev_t *dev = &(v->dev);

   I2C_DEV_TAKE_MUTEX(dev);

   // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
   if (v->io_2v8)
      I2C_DEV_CHECK(dev, update_reg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0x00, 0x01));    // set bit 0
   // "Set I2C standard mode"
   I2C_DEV_CHECK(dev, write_reg(dev, 0x88, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x00));
   I2C_DEV_CHECK(dev, read_reg(dev, 0x91, &stop_variable));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x00));

   // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
   I2C_DEV_CHECK(dev, update_reg(dev, MSRC_CONFIG_CONTROL, 0x00, 0x12));

   I2C_DEV_GIVE_MUTEX(dev);

   // set final range signal rate limit to 0.25 MCPS (million counts per second)
   I2C_DEV_CHECK(dev, vl53l0x_setSignalRateLimit(v, 0.25));

   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF));
   I2C_DEV_GIVE_MUTEX(dev);

   // VL53L0X_DataInit() end

   // VL53L0X_StaticInit() begin

   uint8_t spad_count;
   int spad_type_is_aperture;
   I2C_DEV_CHECK(dev, vl53l0x_getSpadInfo(v, &spad_count, &spad_type_is_aperture));
   // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
   // the API, but the same data seems to be more easily readable from
   // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
   uint8_t ref_spad_map[6];
   
   I2C_DEV_TAKE_MUTEX(dev);

   I2C_DEV_CHECK(dev, read_reg_multi(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6));

   // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4));

   uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;       // 12 is the first aperture spad
   uint8_t spads_enabled = 0;

   for (uint8_t i = 0; i < 48; i++)
   {
      if (i < first_spad_to_enable || spads_enabled == spad_count)
      {
         // This bit is lower than the first one that should be enabled, or
         // (reference_spad_count) bits have already been enabled, so zero this bit
         ref_spad_map[i / 8] &= ~(1 << (i % 8));
      } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
      {
         spads_enabled++;
      }
   }

   I2C_DEV_CHECK(dev, write_reg_multi(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6));

   // -- VL53L0X_set_reference_spads() end

   // -- VL53L0X_load_tuning_settings() begin
   // DefaultTuningSettings from vl53l0x_tuning.h

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x09, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x10, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x11, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0x24, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x25, 0xFF));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x75, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x4E, 0x2C));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x48, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x30, 0x20));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x30, 0x09));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x54, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x31, 0x04));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x32, 0x03));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x40, 0x83));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x46, 0x25));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x60, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x27, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x50, 0x06));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x51, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x52, 0x96));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x56, 0x08));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x57, 0x30));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x61, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x62, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x64, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x65, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x66, 0xA0));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x22, 0x32));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x47, 0x14));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x49, 0xFF));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x4A, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x7A, 0x0A));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x7B, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x78, 0x21));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x23, 0x34));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x42, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x44, 0xFF));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x45, 0x26));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x46, 0x05));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x40, 0x40));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x0E, 0x06));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x20, 0x1A));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x43, 0x40));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x34, 0x03));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x35, 0x44));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x31, 0x04));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x4B, 0x09));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x4C, 0x05));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x4D, 0x04));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x44, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x45, 0x20));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x47, 0x08));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x48, 0x28));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x67, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x70, 0x04));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x71, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x72, 0xFE));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x76, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x77, 0x00));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x0D, 0x01));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x01, 0xF8));

   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x8E, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x00, 0x01));
   I2C_DEV_CHECK(dev, write_reg(dev, 0xFF, 0x00));
   I2C_DEV_CHECK(dev, write_reg(dev, 0x80, 0x00));

   // -- VL53L0X_load_tuning_settings() end

   // "Set interrupt config to new sample ready"
   // -- VL53L0X_SetGpioConfig() begin

   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
   I2C_DEV_CHECK(dev, update_reg(dev, GPIO_HV_MUX_ACTIVE_HIGH, 0x10, 0x00)); // active low
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01));

   I2C_DEV_GIVE_MUTEX(dev);

   // -- VL53L0X_SetGpioConfig() end

   I2C_DEV_CHECK(dev, vl53l0x_getMeasurementTimingBudget(dev, &measurement_timing_budget_us));

   // "Disable MSRC and TCC by default"
   // MSRC = Minimum Signal Rate Check
   // TCC = Target CentreCheck
   // -- VL53L0X_SetSequenceStepEnable() begin

   I2C_DEV_TAKE_MUTEX(dev);

   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8));

   I2C_DEV_GIVE_MUTEX(dev);

   // -- VL53L0X_SetSequenceStepEnable() end

   // "Recalculate timing budget"
   //ESP_LOGI("[TOF sensor lib]", "measurement_timing_budget_us: %lu us", measurement_timing_budget_us);
   I2C_DEV_CHECK(dev, vl53l0x_setMeasurementTimingBudget(v, measurement_timing_budget_us));

   // VL53L0X_StaticInit() end

   // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

   // -- VL53L0X_perform_vhv_calibration() begin
   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01));
   I2C_DEV_GIVE_MUTEX(dev);

   I2C_DEV_CHECK(dev, performSingleRefCalibration(v, 0x40));
   // -- VL53L0X_perform_vhv_calibration() end

   // -- VL53L0X_perform_phase_calibration() begin
   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02));
   I2C_DEV_GIVE_MUTEX(dev);

   I2C_DEV_CHECK(dev, performSingleRefCalibration(v, 0x00));
   // -- VL53L0X_perform_phase_calibration() end

   // "restore the previous Sequence Config"
   I2C_DEV_TAKE_MUTEX(dev);
   I2C_DEV_CHECK(dev, write_reg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8));
   I2C_DEV_GIVE_MUTEX(dev);
   // VL53L0X_PerformRefCalibration() end

   return ESP_OK;
}

vl53l0x_error_t vl53l0x_getError(vl53l0x_t *v)
{
   return v->err_type;
}

char *vl53l0x_err_to_name(vl53l0x_error_t err_code)
{
   if (err_code >= 0 && err_code < Errors_count)
      return vl53l0x_error_messages_lookup_table[err_code];

   return "Error name not found!";
}