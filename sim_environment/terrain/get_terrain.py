import rospy
from gazebo_msgs.srv import GetPhysicsProperties, GetModelState
import numpy as np
import open3d as o3d

def get_terrain_data():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        # Define the area and resolution
        x_min, x_max, x_res = -10, 10, 0.1
        y_min, y_max, y_res = -10, 10, 0.1
        x_values = np.arange(x_min, x_max, x_res)
        y_values = np.arange(y_min, y_max, y_res)
        points = []

        for x in x_values:
            for y in y_values:
                response = get_model_state('ground_plane', 'world')
                z = response.pose.position.z
                points.append([x, y, z])
        
        return np.array(points)

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def save_point_cloud(points):
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points)
    o3d.io.write_point_cloud("terrain.pcd", pc)

if __name__ == "__main__":
    rospy.init_node('terrain_extractor', anonymous=True)
    points = get_terrain_data()
    if points is not None:
        save_point_cloud(points)
