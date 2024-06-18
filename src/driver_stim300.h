
#ifndef DRIVER_STIM300_DRIVER_STIM300_H
#define DRIVER_STIM300_DRIVER_STIM300_H

#include <boost/crc.hpp>
#include "../src/stim300_constants.h"
#include "../src/datagram_parser.h"
#include "../src/serial_driver.h"
#include <vector>
#include <assert.h>
#include <eigen3/Eigen/Geometry> 

class DriverStim300
{
public:
  DriverStim300(SerialDriver& serial_driver,
                stim_300::DatagramIdentifier datagram_id = stim_300::DatagramIdentifier::RATE_ACC_INCL_TEMP_AUX,
                stim_300::GyroOutputUnit gyro_output_unit = stim_300::GyroOutputUnit::ANGULAR_RATE,
                stim_300::AccOutputUnit acc_output_unit = stim_300::AccOutputUnit::ACCELERATION,
                stim_300::InclOutputUnit incl_output_unit = stim_300::InclOutputUnit::ACCELERATION,
                SerialDriver::BAUDRATE baudrate = SerialDriver::BAUDRATE::BAUD_921600,
                uint16_t serial_read_timeout_ms = 1);
  ~DriverStim300();
  double getAccX() const;
  double getAccY() const;
  double getAccZ() const;
  double getGyroX() const;
  double getGyroY() const;
  double getGyroZ() const;
  /** \brief Return the Accelerometers values
   */
  Eigen::Vector3d getAccData();

  /** \brief Return the Gyroscopes values
   */
  Eigen::Vector3d getGyroData();

  /** \brief Return the Inclinometers values
   */
  Eigen::Vector3d getInclData();
  uint16_t getLatency_us() const;
  double getAverageTemp() const;
  bool isChecksumGood() const;
  bool isSensorStatusGood() const;
  uint8_t getInternalMeasurmentCounter() const;
  bool processPacket();

private:
  enum class Mode : uint8_t
  {
    Init,
    Normal,
    Service
  };
  Mode mode_;

  SerialDriver& serial_driver_;
  stim_300::DatagramIdentifier datagram_id_;
  stim_300::DatagramParser datagram_parser_;
  stim_300::SensorData sensor_data_;
  uint8_t datagram_size_;
  bool checksum_is_ok_;
  bool no_internal_error_;
  uint8_t crc_dummy_bytes_;
  uint16_t serial_read_timeout_ms_;
  std::vector<uint8_t> buffer_;
  size_t n_read_bytes_;
  bool in_sync_;

  bool verifyChecksum(std::vector<uint8_t>::const_iterator begin, std::vector<uint8_t>::const_iterator end,
                      uint32_t& expected_CRC);
};

#endif  // DRIVER_STIM300_DRIVER_STIM300_H
