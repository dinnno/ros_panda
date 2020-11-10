import tf

def pose_quat2rpy(pose): # type(pose) = geometry_msgs.msg.Pose
    quaternion = (pose.orientation.x,
                  pose.orientation.y,
                  pose.orientation.z,
                  pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]

    return (pose.position, roll, pitch, yaw)
    
