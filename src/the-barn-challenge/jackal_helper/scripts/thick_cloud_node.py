#!/usr/bin/env python3
import rospy
from laser_assembler.srv import AssembleScans2
from sensor_msgs.msg import PointCloud2

def call_assembler():
    rospy.init_node("thick_cloud_provider")

    # This topic will be the input for your YOLO node
    pub = rospy.Publisher("/thick_cloud", PointCloud2, queue_size=1)
    
    # Wait for the service to start
    rospy.wait_for_service("/assemble_scans2")
    assemble_scans = rospy.ServiceProxy('/assemble_scans2', AssembleScans2)
    
    rate = rospy.Rate(2) # 2Hz is plenty for 3D object detection in a tunnel
    
    while not rospy.is_shutdown():
        try:
            # Request data from the last 1.0 second
            now = rospy.get_rostime()

            # Check if simulation time has actually started (not 0)
            # AND ensure we have at least 1 second of history to look back on
            if now.to_sec() < 1.1:
                if now.to_sec() == 0:
                    rospy.logwarn_throttle(5, "Waiting for Gazebo clock to start...")
                else:
                    rospy.loginfo_throttle(5, "Buffering initial points (Time: %.2f)..." % now.to_sec())
                rate.sleep()
                continue

            past = now - rospy.Duration(1.0)
            
            resp = assemble_scans(past, now)
            pub.publish(resp.cloud)
            
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: %s" % e)
        
        rate.sleep()

if __name__ == '__main__':
    call_assembler()