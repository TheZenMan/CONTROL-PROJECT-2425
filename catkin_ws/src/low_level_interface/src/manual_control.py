#!/usr/bin/env python
u"""
 The Clear BSD License

  Copyright (c) 2018 Philipp Rothenhaeusler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted (subject to the limitations in the disclaimer
  below) provided that the following conditions are met:

       * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

       * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

       * Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.

  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
  THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
 """

import os
import time
import rospy as rp
import numpy as np
import signal as sg
import readchar as rc
from low_level_interface.msg import lli_ctrl_request as msg_out


class SPMBInterface:
    u"""
        Simple manual control class for testing the vehicle under ROS
    """
    def __init__(self):
        # define vehicle parameters
        self.gear = 1                                       # fixed to gear 1
        self.VELOCITY_FORWARD_MAXIMUM = 10.                 # in m/s
        self.VELOCITY_REVERSE_MAXIMUM = 3.

        self.STEERING_MAXIMUM = np.pi/4.                    # rad

        # define output variables (si)
        self.output_steering = 0.
        self.output_velocity = 0.

        # define output variables (%)
        self.output_steering_ppm = 0
        self.output_velocity_ppm = 0

        # define user input variables (%)
        self.key = ""
        self.user_input_steering_ppm = 0
        self.user_input_velocity_ppm = 0

        # define parametrs for user interface
        self.CONTROL_OUTPUT_SATURATION = 100                # set training mode here (e.g. only 20 % control output)
        self.STEP_VELOCITY = 5
        self.STEP_STEERING = 20

        self.USER_INPUT = False                             # boolean flag for key entry by user
        self.MANUAL_CONTROL_WATCHDOG_IS_GOOD = False        # if this is false something in the class went wrong

        # Initialise ROS
        self.PUBLISH_ACTIVE = False
        self.ROS_PUBLISH_RATE = 40
        self.ros_timer = 0.0
        self.ros_msg_out = msg_out()

        # ros related timing constraints on reading input (UNIX only)
        self.current_time = time.time()
        self.minimum_time_period = 1./self.ROS_PUBLISH_RATE
        sg.signal(sg.SIGALRM, self.ros_publish_sig_handle)
        sg.setitimer(sg.ITIMER_REAL, 0, self.minimum_time_period)

        # define node and publisher objects
        rp.init_node('manual_control', anonymous=True)
        self.ros_publisher = rp.Publisher('/lli/ctrl_request', msg_out, queue_size=0)
        self.ros_publish_rate = rp.Rate(self.ROS_PUBLISH_RATE)

        # define update and status flags
        self.CONTROL_REQUEST_SUBMITTED = False              # set to true on each publish event

    def update_timer(self):
        u"""
            update timer counter
        :return:
        """
        sg.setitimer(sg.ITIMER_REAL, self.minimum_time_period, self.minimum_time_period)

    @staticmethod
    def ms(time_in_seconds):
        return time_in_seconds*1000

    @staticmethod
    def cls():
        u"""
            Simple clear screen static method to improve readability
        """
        os.system("clear")

    def saturate_control_output(self, output):
        u"""
            Saturates the output based on the output value range as defined in __init__
        """
        if (-self.CONTROL_OUTPUT_SATURATION < output) and (output < self.CONTROL_OUTPUT_SATURATION):
            return output
        else:
            return np.sign(output)*self.CONTROL_OUTPUT_SATURATION

    def transform_steering_from_ppm_to_si(self, steering_ppm):
        u"""
            Transforms the ppm signal to a steering value in rad
        :return:
        """
        return self.STEERING_MAXIMUM*steering_ppm/100.

    def transform_velocity_from_ppm_to_si(self, velocity_ppm):
        u"""
            Transforms the ppm signal to a velocity value in m/s
        :return:
        """
        if np.sign(velocity_ppm) >= 0:
            return self.VELOCITY_FORWARD_MAXIMUM*velocity_ppm/100.
        else:
            return self.VELOCITY_REVERSE_MAXIMUM*velocity_ppm/100.

    def ros_publish_control_request(self):
        u"""
            Publishes the control request to the low level interface
        """
        # TODO: might require a lock on thread if publisher is active
        # read user_input values and apply saturation in ros_msg output range [-100,100]
        self.output_steering_ppm = self.user_input_steering_ppm
        self.output_velocity_ppm = self.user_input_velocity_ppm

        # transform output values in SI units
        self.output_steering = self.transform_steering_from_ppm_to_si(self.output_steering_ppm)
        self.output_velocity = self.transform_velocity_from_ppm_to_si(self.output_velocity_ppm)

        # acknowledge its successful transmission from the user input thread (allows thread to continue reading input)
        self.CONTROL_REQUEST_SUBMITTED = True

        # define ROS message
        self.ros_msg_out.velocity = self.output_velocity_ppm
        # TODO: steering might need to be inverted (+ to the left and - to satisfy the vehicle ISO coordinate system)
        # TODO: Will be adapted on spmb microcontroller in the future
        self.ros_msg_out.steering = self.output_steering_ppm

        self.ros_publisher.publish(self.ros_msg_out)

    def ros_publish_sig_handle(self, signum, frame):
        u"""
            signal interrupt after time expiration
        :param signum:
        :param frame:
        :return:
        """
        if (time.time() - self.ros_timer) >= 1./self.ROS_PUBLISH_RATE:
            self.ros_timer = time.time()
            spmb.ros_publish_control_request()
        else:
            pass
        self.update_timer()

    def read_user_input(self):
        u"""
            Reads the user input and updates the user_input variables
        """
        self.MANUAL_CONTROL_WATCHDOG_IS_GOOD = True

        self.key = ''
        self.key = rc.readchar()

        # check for velocity command
        if self.key == 'w':
            self.user_input_velocity_ppm += self.STEP_VELOCITY
        elif self.key == 's':
            self.user_input_velocity_ppm -= self.STEP_VELOCITY
        else:
            pass

        # check for steering command
        if self.key == 'a':
            self.user_input_steering_ppm += self.STEP_STEERING
        elif self.key == 'd':
            self.user_input_steering_ppm -= self.STEP_STEERING
        else:
            pass

        # check for braking command
        if self.key == 'b':
            # TODO: Depending on ESC settings there is some logic required for active breaking (negative Torque)
            self.user_input_steering_ppm = 0
            self.user_input_velocity_ppm = 0

        # check for reset command
        if self.key == 'r':
            self.user_input_steering_ppm = 0
            self.user_input_velocity_ppm = 0
        else:
            pass

        self.user_input_steering_ppm = self.saturate_control_output(self.user_input_steering_ppm)
        self.user_input_velocity_ppm = self.saturate_control_output(self.user_input_velocity_ppm)
        self.USER_INPUT = True

    def display_refresh(self):
        u"""
            Checks whether the interface needs to be refreshed
        :return:
        """
        if self.USER_INPUT:
            return True
        else:
            return False

    def display_interface(self):
        u"""
            Displays the user interface to visualise the current data and what controls are available
        :return:
        """
        # TODO: add telemetry feedback from lli_ctrl_actuated
        self.cls()
        self.USER_INPUT = False  # reset condition for refreshing display
        print('############################### \r')
        print("------------------------------- \r")
        print("          CONTROL-KEYS          \r")
        print("------------------------------- \r")
        print('Velocity: [ forward | reverse ] \r')
        print('          [    w    |    s    ] \r')
        print("------------------------------- \r")
        print('Steering  [  left   |  right  ] \r')
        print('          [    a    |    d    ] \r')
        print("------------------------------- \r")
        print('Brake:    [         b         ] \r')
        print("------------------------------- \r")
        print('Reset:    [         r         ] \r')
        print("------------------------------- \r")
        print('QUIT:     [         q         ] \r')
        print('############################### \r')
        print('Velocity (%):   {0:.2f}   ({1}%) \r'.format(self.output_velocity, self.output_velocity_ppm))
        print('Steering (%):   {0:.2f}   ({1}%) \r'.format(self.output_steering, self.output_steering_ppm))
        print('############################### \r')
        print('User input:   {0}\r'.format(self.key))


if __name__ == "__main__":
    u"""
        Main logic loop
    """
    try:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN STARTED                  #\r")
        print("#######################################################\r")
        print("... initialize manual controller                       \r")
        time.sleep(0.5)

        spmb = SPMBInterface()
        try:
            # Instantiate Signal&Power Management Board
            spmb.display_interface()
            spmb.update_timer()
            while spmb.key != 'q' and not rp.is_shutdown():
                if spmb.CONTROL_REQUEST_SUBMITTED:
                    if spmb.display_refresh():
                        spmb.display_interface()
                    else:
                        pass
                    spmb.read_user_input()
                else:
                    pass
        except rp.ROSInterruptException:
            spmb.MANUAL_CONTROL_WATCHDOG_IS_GOOD = False
            print("An exception occurred...")

    except rp.ROSInterruptException:
        print("#######################################################\r")
        print("#           PROGRAM HAS BEEN INTERRUPTED              #\r")
        print("#######################################################\r")
