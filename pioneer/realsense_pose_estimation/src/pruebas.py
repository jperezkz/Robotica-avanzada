import sys
import pyrealsense2 as rs
import numpy as np
import time
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot


def quaternion_to_euler_angle(quat):
    # print(w,x,y,z)
    qw = quat[0]
    qx = quat[1]
    qy = quat[2]
    qz = quat[3]

    a2 = np.arctan2(2 * qy * qw - 2 * qx * qz, 1 - 2 * qy * qy - 2 * qz * qz)
    a1 = np.arcsin(2 * qx * qy + 2 * qz * qw)
    a0 = np.arctan2(2 * qx * qw - 2 * qy * qz, 1 - 2 * qx * qx - 2 * qz * qz)

    if np.isclose(qx * qy + qz * qw, 0.5):
        a1 = 2.0 * np.arctan2(qx, qw)
        a0 = 0.0
    if np.isclose(qx * qy + qz * qw, -0.5):
        a1 = -2.0 * np.arctan2(qx, qw)
        a0 = 0.0
    return a0, a1, a2

try:
    config = rs.config()
    config.enable_device("925122110807")
    config.enable_stream(rs.stream.pose)
    pipeline = rs.pipeline()
    pipeline.start(config)

except Exception as e:
    print("Error initializing camera")
    print(e)
    sys.exit(-1)

tm = TransformManager()
tm.add_transform("robot", "origin", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0.0]),
                                         [2000, -30000, 0.0]))

tm.add_transform("slam_sensor", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0.0, 0.0, 0]),
                                        [0.0, 0.0, 0.0]))

while True:
    frames = pipeline.wait_for_frames()
    f = frames.first_or_default(rs.stream.pose)
    data = f.as_pose_frame().get_pose_data()
    #angles = quaternion_to_euler_angle([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z])

    tm.add_transform("world", "slam_sensor", pytr.transform_from_pq([data.translation.x*1000.0,
                                                                      -data.translation.z*1000.0,
                                                                       data.translation.y*1000.0,
                                                                       data.rotation.w,
                                                                       data.rotation.x,
                                                                       data.rotation.y,
                                                                       data.rotation.z]))

    t = tm.get_transform("world", "origin")
    q = pyrot.quaternion_from_matrix(t[0:3, 0:3])
    print(pytr.transform(t, [0,0,0,1])[0:3],  quaternion_to_euler_angle(q), data.mapper_confidence, data.tracker_confidence )

    ##print("\r Device Position: ", t[0][3], t[1][3], t[2][3], angles, end="\r")

# now = time.time()
# with open("back_side.txt", "w") as text_file:
#     while (time.time() - now) < 5:
#         frames = pipeline.wait_for_frames()
#         f = frames.first_or_default(rs.stream.pose)
#         data = f.as_pose_frame().get_pose_data()
#         angles = quaternion_to_euler_angle(data.rotation.w, data.rotation.x, data.rotation.y,data.rotation.z)
#         print("\r Device Position: ", -data.translation.x * 1000, data.translation.z * 1000, data.translation.y * 1000, angles, end="\r")
#         print(str(data.translation), str(data.rotation), file=text_file)


