#include "ICM20649.hpp"

#include <iostream>

namespace ICM20649
{
Device::Device(const Config& config)
    : i2c_bus_(1),
      fid_(open(std::string("/dev/i2c-" + std::to_string(i2c_bus_)).c_str(),
                O_RDWR))
{
  // could not open the file
  if (fid_ < 0)
  {
    throw unreadable_file(std::string("/dev/i2c-" + std::to_string(i2c_bus_)));
  }
  // could not open the device on the bus
  if (ioctl(fid_, I2C_SLAVE, ICM20649_I2C_ADDR) < 0)
  {
    throw unreadable_device(i2c_bus_, ICM20649_I2C_ADDR);
  }

  // reset the device
  reset();

  const uint8_t device_id = who_am_i();
  // the device ID does not match
  if (device_id != ICM20649_DEVICE_ID)
  {
    throw invalid_device_id(device_id, ICM20649_DEVICE_ID);
  }

  // auto-selects the best available clock source
  write_register(ICM20649_REG_PWR_MGMT_1, ICM20649_BIT_CLK_PLL);
  usleep(30000);

  write_register(ICM20649_REG_INT_PIN_CFG,
                 ICM20649_BIT_INT_ACTL | ICM20649_BIT_INT_OPEN);

  // enable/disable accelerometer, gyroscope and temperature
  sensors(config.accelerometer, config.gyroscope, config.temperature);

  // set sample rate
  sample_rate(config.sample_rate);

  // set bandwidth
  accelerometer_bandwidth(config.accelerometer_bandwidth);
  gyroscope_bandwidth(config.gyroscope_bandwidth);

  // set resolution range
  accelerometer_resolution(config.accelerometer_scale);
  gyroscope_resolution(config.gyroscope_scale);
}

Device::~Device()
{
  if (fid_ > 0)
  {
    close(fid_);
  }
}

void
Device::accelerometer(float& ax, float& ay, float& az) const
{
  accelerometer_g(ax, ay, az);

  // convert from acceleration gravity to meter per second squared
  ax *= G_TO_METER_PER_SECOND_SQUARED;
  ay *= G_TO_METER_PER_SECOND_SQUARED;
  az *= G_TO_METER_PER_SECOND_SQUARED;
}

void
Device::accelerometer_g(float& ax, float& ay, float& az) const
{
  // retrieve the current resolution
  const float resolution = accelerometer_resolution();

  ax = resolution * static_cast<float>(
                        static_cast<int16_t>(
                            read_register(ICM20649_REG_ACCEL_XOUT_H_SH) << 8) |
                        read_register(ICM20649_REG_ACCEL_XOUT_L_SH));
  ay = resolution * static_cast<float>(
                        static_cast<int16_t>(
                            read_register(ICM20649_REG_ACCEL_YOUT_H_SH) << 8) |
                        read_register(ICM20649_REG_ACCEL_YOUT_L_SH));
  az = resolution * static_cast<float>(
                        static_cast<int16_t>(
                            read_register(ICM20649_REG_ACCEL_ZOUT_H_SH) << 8) |
                        read_register(ICM20649_REG_ACCEL_ZOUT_L_SH));
}

void
Device::accelerometer_bandwidth(const uint8_t bandwidth) const
{
  const uint8_t reg_val =
      (read_register(ICM20649_REG_ACCEL_CONFIG) & ~(ICM20649_MASK_ACCEL_BW)) |
      (bandwidth & ICM20649_MASK_ACCEL_BW);
  write_register(ICM20649_REG_ACCEL_CONFIG, reg_val);
}

float
Device::accelerometer_resolution() const
{
  // read the actual accelerometer full scale setting
  const uint8_t scale_setting =
      read_register(ICM20649_REG_ACCEL_CONFIG) & ICM20649_MASK_ACCEL_FULLSCALE;

  // calculate the resolution
  switch (scale_setting)
  {
    case ICM20649_ACCEL_FULLSCALE_2G:
      return 4.0 / 32768.0;
    case ICM20649_ACCEL_FULLSCALE_4G:
      return 8.0 / 32768.0;
    case ICM20649_ACCEL_FULLSCALE_8G:
      return 16.0 / 32768.0;
    case ICM20649_ACCEL_FULLSCALE_16G:
      return 30.0 / 32768.0;
  }
  return -1.0;
}

void
Device::accelerometer_resolution(const uint8_t scale_setting) const
{
  const uint8_t reg_val = (read_register(ICM20649_REG_ACCEL_CONFIG) &
                           ~(ICM20649_MASK_ACCEL_FULLSCALE)) |
                          (scale_setting & ICM20649_MASK_ACCEL_FULLSCALE);
  write_register(ICM20649_REG_ACCEL_CONFIG, reg_val);
}

float
Device::accelerometer_sample_rate(const float rate) const
{
  // calculate the sample rate divider
  const uint16_t sample_rate_divider = static_cast<uint16_t>(
      std::max(std::min((1125.0 / rate) - 1.0, 4095.0), 0.0));

  write_register(ICM20649_REG_ACCEL_SMPLRT_DIV_1,
                 static_cast<uint8_t>(sample_rate_divider >> 8));
  write_register(ICM20649_REG_ACCEL_SMPLRT_DIV_2,
                 static_cast<uint8_t>(sample_rate_divider & 0xFF));

  // return actual sample rate
  return static_cast<float>(1125.0 / (sample_rate_divider + 1));
}

void
Device::gyroscope(float& wx, float& wy, float& wz) const
{
  gyroscope_d(wx, wy, wz);
  wx *= DEGREE_TO_RADIAN;
  wy *= DEGREE_TO_RADIAN;
  wz *= DEGREE_TO_RADIAN;
}

void
Device::gyroscope_d(float& wx, float& wy, float& wz) const
{
  // retrieve the current resolution
  const float resolution = gyroscope_resolution();

  wx = resolution *
       static_cast<float>(static_cast<int16_t>(
                              read_register(ICM20649_REG_GYRO_XOUT_H_SH) << 8) |
                          read_register(ICM20649_REG_GYRO_XOUT_L_SH));
  wy = resolution *
       static_cast<float>(static_cast<int16_t>(
                              read_register(ICM20649_REG_GYRO_YOUT_H_SH) << 8) |
                          read_register(ICM20649_REG_GYRO_YOUT_L_SH));
  wz = resolution *
       static_cast<float>(static_cast<int16_t>(
                              read_register(ICM20649_REG_GYRO_ZOUT_H_SH) << 8) |
                          read_register(ICM20649_REG_GYRO_ZOUT_L_SH));
}

void
Device::gyroscope_bandwidth(const uint8_t bandwidth) const
{
  const uint8_t reg_val =
      (read_register(ICM20649_REG_GYRO_CONFIG_1) & ~(ICM20649_MASK_GYRO_BW)) |
      (bandwidth & ICM20649_MASK_GYRO_BW);
  write_register(ICM20649_REG_GYRO_CONFIG_1, reg_val);
}

float
Device::gyroscope_resolution() const
{
  // read the actual gyroscope full scale setting
  const uint8_t scale_setting =
      read_register(ICM20649_REG_GYRO_CONFIG_1) & ICM20649_MASK_GYRO_FULLSCALE;

  // calculate the resolution
  switch (scale_setting)
  {
    case ICM20649_GYRO_FULLSCALE_500DPS:
      return 500.0 / 32768.0;
    case ICM20649_GYRO_FULLSCALE_1000DPS:
      return 1000.0 / 32768.0;
    case ICM20649_GYRO_FULLSCALE_2000DPS:
      return 2000.0 / 32768.0;
    case ICM20649_GYRO_FULLSCALE_4000DPS:
      return 4000.0 / 32768.0;
  }
  return -1.0;
}

void
Device::gyroscope_resolution(const uint8_t scale_setting) const
{
  const uint8_t reg_val = (read_register(ICM20649_REG_GYRO_CONFIG_1) &
                           ~(ICM20649_MASK_GYRO_FULLSCALE)) |
                          (scale_setting & ICM20649_MASK_GYRO_FULLSCALE);
  write_register(ICM20649_REG_GYRO_CONFIG_1, reg_val);
}

float
Device::gyroscope_sample_rate(const float rate) const
{
  // calculate the sample rate divider
  const uint8_t sample_rate_divider = static_cast<uint8_t>(
      std::max(std::min((1125.0 / rate) - 1.0, 255.0), 0.0));

  write_register(ICM20649_REG_GYRO_SMPLRT_DIV, sample_rate_divider);

  // return actual sample rate
  return static_cast<float>(1125.0 / (sample_rate_divider + 1));
}

void
Device::reset() const
{
  write_register(ICM20649_REG_PWR_MGMT_1, ICM20649_BIT_H_RESET);
  usleep(100000);
}

void
Device::sample_rate(const float rate) const
{
  accelerometer_sample_rate(rate);
  gyroscope_sample_rate(rate);
}

void
Device::sensors(const bool accelerometer, const bool gyroscope,
                const bool temperature) const
{
  uint8_t bit_field = 0x00;

  if (accelerometer)
  {
    bit_field &= ~(ICM20649_BIT_PWR_ACCEL_STBY);
  }
  else
  {
    bit_field |= ICM20649_BIT_PWR_ACCEL_STBY;
  }

  if (gyroscope)
  {
    bit_field &= ~(ICM20649_BIT_PWR_GYRO_STBY);
  }
  else
  {
    bit_field |= ICM20649_BIT_PWR_GYRO_STBY;
  }
  write_register(ICM20649_REG_PWR_MGMT_2, bit_field);

  bit_field = read_register(ICM20649_REG_PWR_MGMT_1);
  if (temperature)
  {
    bit_field &= ~(ICM20649_BIT_TEMP_DIS);
  }
  else
  {
    bit_field |= ICM20649_BIT_TEMP_DIS;
  }
  write_register(ICM20649_REG_PWR_MGMT_1, bit_field);
}

float
Device::temperature() const
{
  return static_cast<float>(
             static_cast<int16_t>(read_register(ICM20649_REG_TEMPERATURE_H)
                                  << 8) |
             read_register(ICM20649_REG_TEMPERATURE_L)) /
             333.87 +
         21.0;
}

uint8_t
Device::who_am_i() const
{
  return read_register(ICM20649_REG_WHO_AM_I);
}

uint8_t
Device::read_register(const uint16_t reg) const
{
  if (i2c_smbus_write_byte(fid_, reg) < 0)
  {
    throw error_writing_register(reg);
  }
  const int to_return = i2c_smbus_read_byte(fid_);
  if (to_return < 0)
  {
    throw error_reading_register(reg);
  }
  return static_cast<uint8_t>(to_return);
}

void
Device::write_register(const uint16_t reg, const uint8_t val) const
{
  const int to_return = i2c_smbus_write_byte_data(fid_, reg, val);
  // wait a little bit to make sure it settles
  usleep(10000);
  if (to_return < 0)
  {
    throw error_writing_register(reg);
  }
}

}  // namespace ICM20649
