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
#ifndef MOTOR_H
#define MOTOR_H

#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>
namespace dlut_monitor {
class Motor
{
public:
	Motor();
	Motor(const int minutes,const int freq);
	Motor(const char* direction,const int degree,const int freq);
	bool comOpen ();              //open the port
	bool comSet (int n_speed, int n_bits, char c_event, int n_stop);      //set the port
	bool comWrite (char *buf);    //write to the port
	bool comClose ();             //close the port

  	void motorMoveWithStart();
  	void motorMoveWithStop();
  	void motorMoveWithStart_people();
	static int m_freq;
	const int get_runtime_minute()
	{
		return runtime_minute;
	}
	const int get_motor_freq()
	{
		return motor_freq;
	}
	const bool get_is_stop()
	{
		return is_stop;
	}
	const int get_m_fd_()
	{
		return m_fd_;
	}
	const int get_m_comport_()
	{
		return m_comport_;
	}
private:
	//motor param
	int runtime_minute;
	int motor_freq;
	bool is_stop;
	int rotate_degree;



	//com param
	int m_fd_;
 	int m_comport_;
};
}  // namespace dlut_monitor
#endif
