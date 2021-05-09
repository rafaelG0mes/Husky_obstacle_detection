#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import sys

def pub_msg_select(argv):
  print ('[CLOUD PROCESSING] Running the ' + str(argv[1]) + ' Filter')

def main():

    if len(sys.argv) > 3:
      pub_msg_select(sys.argv)

      rospy.init_node('pub_msg', anonymous=True)
      rospy.spin()

if __name__ == '__main__':
    main()