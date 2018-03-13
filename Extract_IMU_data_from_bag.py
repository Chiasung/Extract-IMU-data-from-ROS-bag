#!/usr/bin/env python
import rospy
import sys
import rosbag
import csv
import rospkg
import os
import math


def main(args):

    rospy.init_node('Extract_IMU_data_from_bag_node')

    t0 = rospy.get_time()

    """"""""""""""
    " Parameters "
    """"""""""""""
    bagfile = rospy.get_param('~bagfile_path','~/IMU_2018001.bag')
    topic = rospy.get_param('~imu_topic_name','/imu/data')
    extractDataPath = ''

    """""""""""""""""""""""""""""
    " IMU data  Directory Path  "
    """""""""""""""""""""""""""""
    if extractDataPath is None:
        extractDataPath = '~/imu_data_raw/'
        if not os.path.isdir(extractDataPath):
            os.mkdir(extractDataPath)

    print "\nIMU raw data will be save in the following directory: \n\n\n\t %s\n"%extractDataPath

    """""""""""""""""
    " Parse Bagfile "
    """""""""""""""""
    bag = rosbag.Bag(bagfile)

    N = bag.get_message_count(topic) # number of measurement samples

    data = np.zeros( (6,N) ) # preallocate vector of measurements
    time_sample = np.zeros( (2,N) ) # preallocate vector of measurements

    cnt = 0
    avgSampleRate = 0
    for topic, msg, t in bag.read_messages(topics=[topic]):
        data[0,cnt] = msg.linear_acceleration.x
        data[1,cnt] = msg.linear_acceleration.y
        data[2,cnt] = msg.linear_acceleration.z
        data[3,cnt] = msg.angular_velocity.x
        data[4,cnt] = msg.angular_velocity.y
        data[5,cnt] = msg.angular_velocity.z
        time_sample[0,cnt] = msg.header.stamp.secs*pow(10,9) + msg.header.stamp.nsecs;
        if cnt > 1:
            time_sample[1,cnt] = pow(10,9) /(time_sample[0,cnt] - time_sample[0,cnt-1])
            avgSampleRate = avgSampleRate + time_sample[1,cnt]
        cnt = cnt + 1

    sampleRate = avgSampleRate/(cnt-3)
    bag.close()

    print "[%0.2f seconds] Bagfile parsed\n"%(rospy.get_time()-t0)

    """""""""""""""
    " write to cvs"
    """""""""""""""
    fname = 'allan_A_xyz_G_xyz_'
    f = open(extractDataPath + fname + '_rate_%f.csv'%sampleRate, 'wt')

    try:
        writer = csv.writer(f)
        writer.writerow( ('Time','Ax', 'Ay', 'Az', 'Gx', 'Gy', 'Gz') )

        for i in range(data[1,:].size):
            writer.writerow( (time_sample[0,i], data[0,i], data[1,i], data[2,i], data[3,i],data[4,i],data[5,i]))
    finally:
        f.close()


if __name__ == '__main__':
  main(sys.argv)
