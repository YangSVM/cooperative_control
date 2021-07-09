import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    data_list = data.data.split(',')
    lat = data_list[12]
    lon = data_list[13]
    with open('rawmapyyb', 'a') as f: # 'a'表示append,即在原来文件内容后继续写数据（不清楚原有数据）
        f.write(lat, lon)

    
def listener():

    rospy.init_node('listener', anonymous=True)


    rospy.Subscriber("/gpchc", String, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()