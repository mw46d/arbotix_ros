#!/usr/bin/env python

"""
  ParallelConvert:  For Parallel/Prismatic joint, convert Angle to Width
  Copyright (c) 2014 Vanadium Labs LLC.  All right reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions are met:
      * Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.
      * Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.
      * Neither the name of the copyright holder nor the names of its
        contributors may be used to endorse or promote products derived
        from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND CONTRIBUTORS "AS IS" AND
  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THEY BE LIABLE FOR ANY DIRECT, INDIRECT,
  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""
import rospy
from math import asin, sin, cos, sqrt, acos

#  Parallel Gripper gap calculation:
#
#        o           (S) is servo axis
#       /:\    ||      R is radius of servo horn
#    R / :  \C ||      C is connector to finger        
#     /  :x   \||      Offset is for foam and offset from connection back to finger
#    /a  :      \        
#  (S). . . . . . o
#      n    y  ||
#              || <-- offset
#              ||finger 

PRISM_PARAM_NS = "/arbotix/joints/"

class ParallelConvert:
    """ For Parallel/Prismatic joint, convert Angle to Width and vice versa"""
    def __init__(self, joint_name):
        ns = PRISM_PARAM_NS + joint_name + "/"
        self.r = rospy.get_param(ns+'radius', .0078)      # Radius of servo horn
        self.c = rospy.get_param(ns+'connector', 0.024)   # connector from horn to finger
        # offset back from connection to actual foam pad      
        self.offset = rospy.get_param(ns+'offset', 0.016)
        self.angle_offset = rospy.get_param(ns+'angle_offset', 0.0)
        # print "mw ParallelConvert::__init__() r= %f  c= %f  offset= %f  aoffset= %f" % (self.r, self.c, self.offset, self.angle_offset)

    def widthToAngle(self, width):
        """ Convert width to servo angle """
        leg = (width / 2) + self.offset  # Remove double for two fingers and add offset
        # Law of Cosines
        # return -1 * acos ( (self.r * self.r + leg * leg - self.c * self.c) / (2 * self.r * leg) ) + self.angle_offset
        ret =  acos ( (self.r * self.r + leg * leg - self.c * self.c) / (2 * self.r * leg) )
        # print "mw ParallelConvert::widthToAngle(%f) len= %f  ret= %f  %f" % (width, leg, ret, ret + self.angle_offset)
        return ret + self.angle_offset

    def angleToWidth(self, ang):
        """ Convert angle to width for this gripper """
        a = ang - self.angle_offset
        n = cos(a) * self.r               # CAH
        x = sin(a) * self.r               # SOH
        y = sqrt(self.c * self.c - x * x)   # Pythagorean
        return (n + y - self.offset) * 2  # Remove offset and double to cover two fingers

