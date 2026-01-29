/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBOMASTER_SERIAL_DEVICE_H
#define ROBOMASTER_SERIAL_DEVICE_H
#include <string>
#include <cstring>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include "hardware_interface.h"

namespace robomaster {
/**
 * @brief serial device class inherited from hardware interface
 */
class SerialDevice: public HardwareInterface {
 public:
  /**
   * @brief Constructor of serial device
   * @param port_name port name, i.e. /dev/ttyUSB0，设备路径
   * @param baudrate serial baudrate，波特率
   */
  SerialDevice(std::string port_name, int baudrate);
  /**
   * @brief Destructor of serial device to close the device
   */
  ~SerialDevice();
  /**
   * @brief Initialization of serial device to config and open the device
   * @return True if success
   */
  virtual bool Init() override;
  /**
   * @brief Serial device read function
   * @param buf Given buffer to be updated by reading
   * @param len Read data length
   * @return -1 if failed, else the read length
   */
  virtual int Read(uint8_t *buf, int len) override;
  /**
   * @brief Write the buffer data into device to send the data
   * @param buf Given buffer to be sent
   * @param len Send data length
   * @return < 0 if failed, else the send length
   */
  virtual int ReadUntil2(uint8_t *buf, uint8_t end1 , uint8_t end2,uint8_t max_len);

  virtual int Write(const uint8_t *buf, int len) override;
  
 private:
  /**
   * @brief Open the serial device
   * @return True if open successfully
   */
  bool OpenDevice();
  /**
   * @brief Close the serial device
   * @return True if close successfully
   */
  bool CloseDevice();
  /**
   * @brief Configure the device
   * @return True if configure successfully
   */
  bool ConfigDevice();
  //! port name of the serial device
  std::string port_name_;                                       //设备路径
  //! baudrate of the serial device
  int baudrate_;                                                //波特率
  //! stop bits of the serial device, as default
  char parity_bits_;                                            //奇偶校验位，用于检测传输错误
  int stop_bits_;                                               //停止位数量，标识一个字节传输结束，停止位越多，抗干扰性越强，但传输效率越低
  //! data bits of the serial device, as default
  int data_bits_;                                               //每个字符的数据位数
  //! parity bits of the serial device, as default
  // char parity_bits_;
  //! serial handler
  int serial_fd_;                                               //文件描述符，用于标识打开文件的整数，open()函数返回的值，所有读写操作都通过这个描述符进行，-1表示未打开
  //! set flag of serial handler
  fd_set serial_fd_set_;                                        //文件描述符集合，用于select()多路复用
  //! termios config for serial handler
  struct termios new_termios_, old_termios_;
  int serial_loss = 0;                                          //串口通信丢包计数器

};
}
#endif //ROBOMASTER_SERIAL_DEVICE_H
