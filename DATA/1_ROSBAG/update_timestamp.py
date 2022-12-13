import rosbag
import pdb

INPUT_FILE   = '221020_01_none_04.bag'
OUTPUT_FILE  = '221020_01_none_04_updated.bag'
TOPIC        = '/os_cloud_node/points'

if __name__ == '__main__':
    # Load ROBAG file
    bag = rosbag.Bag(INPUT_FILE, 'r')
    output = rosbag.Bag(OUTPUT_FILE, 'w')

    # msg : message header time stamp,  t : rosbag time stamp
    for topic, msg, t in bag.read_messages():
        if topic == TOPIC:
            msg.header.stamp.secs  = t.secs
            msg.header.stamp.nsecs = t.nsecs
        output.write(topic, msg, t)
    bag.close()
    output.close()
