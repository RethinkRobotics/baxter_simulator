#!/usr/bin/env python
#*********************************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2013, Open Source Robotics Foundation
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Open Source Robotics Foundation
#*     nor the names of its contributors may be
#*     used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#*********************************************************************/

#   Author: Dave Coleman
#   Desc:   Easily turn on or off sonar from roslaunch file

import getopt
import sys
import os
import roslib
roslib.load_manifest('head_control')
import rospy

import std_msgs.msg

def send_command(enable):
    """
    Enable or disable sonars

    @param enable - true or false - are sonars enabled?
    """
    
    if int(enable)==int(0):
        msg = 0 # turn off enable flags for all sonar channels
    else:
        msg = 4095 # turn back on enable flags for all sonar channels

    print(' Enabling sonars= '+enable+' message= '+str(msg));

    pub = rospy.Publisher('/robot/sonar/set_sonars_enabled', std_msgs.msg.UInt16, latch=False)
    rospy.Rate(1).sleep()

    pub.publish(data=int(msg))
    rospy.Rate(1).sleep()


def usage():
        print """
%s [ARGUMENTS]

    -h, --help          This screen
    -e, --enable        0 - disable sonars, 1 - enable sonars
    """ % (os.path.basename(sys.argv[0]),)

def main():
    try:
        opts, args = getopt.getopt(sys.argv[1:], 'hf:',
            ['help', 'enable='])
    except getopt.GetoptError as err:
        print str(err)
        usage()
        sys.exit(2)
    enable = 0
    for o, a in opts:
        if o in ('-h', '--help'):
            usage()
            sys.exit(0)
        elif o in ('-e', '--enable'):
            enable = a

    rospy.init_node('sonar_enable', anonymous=True)

    send_command(enable)
    sys.exit(0)

if __name__ == '__main__':
    main()
