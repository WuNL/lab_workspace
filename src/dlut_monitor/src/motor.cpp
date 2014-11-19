/*********************************************************************
*  Software License Agreement (BSD License)
*  Copyright (c) 2013, Intelligent Robotics Lab, DLUT.
*  All rights reserved.
*  Author:Zhao Cilang
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Intelligent Robotics Lab nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
#include "../include/dlut_monitor/motor.h"
#include "ros/ros.h"
namespace dlut_monitor {
Motor::Motor()
	: runtime_minute(0),
	motor_freq(0),
	is_stop(false),
	m_fd_(0),
	m_comport_(1)
{
	ROS_INFO("into motor class");
	comOpen ();
  	comSet (19200, 8, 'N', 1);    //set the port's parameters
}
Motor::Motor(const int minutes,const int freq)
	: runtime_minute(0),
	motor_freq(0),
	is_stop(false),
	m_fd_(0),
	m_comport_(1)
{
	runtime_minute = minutes;
	motor_freq = freq;
	comOpen ();
	comSet (19200, 8, 'N', 1);    //set the port's parameters
}

Motor::Motor(const char* direction,const int degree,const int freq)
  :motor_freq(0),
  is_stop(false),
  rotate_degree(360),
  m_fd_(0),
  m_comport_(1)
{
  rotate_degree = degree;
  motor_freq = freq;
  //char* dir = direction;
  comOpen ();
  comSet (19200, 8, 'N', 1);    //set the port's parameters
}

bool Motor::comOpen ()
{
  if (m_comport_ == 1)          //serial port 1
  {
    m_fd_ = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == m_fd_)
    {
      perror ("open port 1 FAIL！\n");
      return false;
    }
    else
    {
      ROS_INFO ("open port 1 SUCCESSFUL");
    }
  }
  else if (m_comport_ == 2)     //串口2
  {
    m_fd_ = open ("/dev/ttyMXUSB1", O_RDWR | O_NOCTTY | O_NDELAY);
    if (-1 == m_fd_)
    {
      perror ("open port 2 FAIL！\n");
    }
    else
      ROS_INFO ("open port 2 SUCCESSFUL");
  }

  /*set the port's status */
  if (fcntl (m_fd_, F_SETFL, 0) < 0)
    ROS_INFO ("set the port to Blocked FAIL!");
  else
    ROS_INFO ("set the port to Blocked");
  if (isatty (STDIN_FILENO) == 0)
    ROS_INFO ("It is NOT a terminal equipment");
  else
    ROS_INFO ("It is a tty equipment");

  return true;
}

bool Motor::comSet (int n_speed, int n_bits, char c_event, int n_stop)
{
  struct termios newtio, oldtio;
  if (tcgetattr (m_fd_, &oldtio) != 0)
  {
    perror ("save the old parameters FAIL！\n");
    return false;
  }
  bzero (&newtio, sizeof (newtio));
  /*set character size */
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;
  /*set data bit */
  switch (n_bits)
  {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }
  /*set parity bit */
  switch (c_event)
  {
    case 'O':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'E':
      newtio.c_iflag |= (INPCK | ISTRIP);
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      break;
    case 'N':
      newtio.c_cflag &= ~PARENB;
      break;
  }
  /*set baud rate */
  switch (n_speed)
  {
    case 2400:
      cfsetispeed (&newtio, B2400);
      cfsetospeed (&newtio, B2400);
      break;
    case 4800:
      cfsetispeed (&newtio, B4800);
      cfsetospeed (&newtio, B4800);
      break;
    case 9600:
      cfsetispeed (&newtio, B9600);
      cfsetospeed (&newtio, B9600);
      break;
    case 19200:
      cfsetispeed (&newtio, B19200);
      cfsetospeed (&newtio, B19200);
      break;
    case 38400:
      cfsetispeed (&newtio, B38400);
      cfsetospeed (&newtio, B38400);
      break;
    case 57600:
      cfsetispeed (&newtio, B57600);
      cfsetospeed (&newtio, B57600);
      break;
    case 115200:
      cfsetispeed (&newtio, B115200);
      cfsetospeed (&newtio, B115200);
      break;
    default:
      cfsetispeed (&newtio, B9600);
      cfsetospeed (&newtio, B9600);
      break;
  }
  /*set stop bit */
  if (n_stop == 1)
    newtio.c_cflag &= ~CSTOPB;
  else if (n_stop == 2)
    newtio.c_cflag |= CSTOPB;

  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  tcflush (m_fd_, TCIFLUSH);
  /*activate the new set */
  if ((tcsetattr (m_fd_, TCSANOW, &newtio)) != 0)
  {
    perror ("activate the new set FAIL");
  }
  ROS_INFO ("set port SUCCESSFUL!");
  return true;
}

bool Motor::comWrite (char *buf)
{
  int n_write;
  n_write = write (m_fd_, buf, strlen (buf));
  if (n_write < 0)
  {
    ROS_INFO ("write error");
    return false;
  }
  return true;
}

bool Motor::comClose ()
{
  close (m_fd_);
  return true;
}
int Motor::m_freq = 0;
void Motor::motorMoveWithStart()
{
  char buf[120];
  int round_num = static_cast<int>( runtime_minute*60*motor_freq/8240);
  sprintf(buf, "B0%06d%06d008240",motor_freq, round_num);    //get the motor control command with paraters that we set.
  comWrite (buf);
  
  m_freq = motor_freq;
  ROS_INFO ("The m_freq is %d", m_freq);
  ROS_INFO ("The command is %s", buf);
  comClose ();
}

void Motor::motorMoveWithStop()
{
  char buf[12] = "MS";
  comWrite (buf);
  ROS_INFO ("The command is %s", buf);
  strcpy(buf,"MU");
  comWrite(buf);
  comClose ();
}

void Motor::motorMoveWithStart_people()
{
  char buf[120];
  int rotate_pulse = static_cast<int>(rotate_degree*8240/360);
  sprintf(buf, "M0%06d%06d",motor_freq,rotate_pulse);    //get the motor control command with paraters that we set.
  comWrite (buf);
  
  m_freq = motor_freq;
  ROS_INFO ("The m_freq is %d", m_freq);
  ROS_INFO ("The command is %s", buf);
  comClose ();
}
}