#!/usr/bin/env python

################################################################################
# Copyright (C) 2013 by Jerome Maye                                            #
# jerome.maye@gmail.com                                                        #
#                                                                              #
# This program is free software; you can redistribute it and/or modify         #
# it under the terms of the Lesser GNU General Public License as published by  #
# the Free Software Foundation; either version 3 of the License, or            #
# (at your option) any later version.                                          #
#                                                                              #
# This program is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                #
# Lesser GNU General Public License for more details.                          #
#                                                                              #
# You should have received a copy of the Lesser GNU General Public License     #
# along with this program. If not, see <http://www.gnu.org/licenses/>.         #
################################################################################

import sys, roslib, rospy
from velodyne.srv import *

def setRPM(rpm):
  rospy.wait_for_service("/velodyne/set_rpm")
  try:
    request = rospy.ServiceProxy("/velodyne/set_rpm", SetRPM)
    response = request(rpm)
    if response.response:
      print "RPM set to: %d" % response.spinRate
    else:
      print "Failed to set RPM to: %d" % response.spinRate
      print "Reason: %s" % response.message
  except rospy.ServiceException, exception:
    print "SetRPM request failed: %s" % exception

def usage():
  return "%s RPM" % sys.argv[0]

if __name__ == "__main__":
  if len(sys.argv) == 2:
    rpm = int(sys.argv[1])
  else:
    print usage()
    sys.exit(1)
  setRPM(rpm)
