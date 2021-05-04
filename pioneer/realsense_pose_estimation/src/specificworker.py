#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2021 by YOUR NAME HERE
#
#    This file is part of RoboComp
#
#    RoboComp is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    RoboComp is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
#

import sys
from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
import pyrealsense2 as rs
import numpy as np
from pytransform3d.transform_manager import TransformManager
import pytransform3d.transformations as pytr
import pytransform3d.rotations as pyrot
import threading

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        self.Period = 50
        self.lock = threading.Lock()
        self.firsttime = False

        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)


    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):

        self.num_cameras = params["num_cameras"]
        self.print = params["print"] == "true"
        self.cameras_list = {}
        name_lst = []

        for i in self.num_cameras :
            device_serial = params["device_serial_"+i]
            name_lst.append(params["name_"+i])
            rx = params["rx_"+i]
            ry = params["ry_"+i]
            rz = params["rz_"+i]
            tx = params["tx_"+i]
            ty = params["ty_"+i]
            tz = params["tz_"+i]
            self.cameras_list[name_lst[i]].append()

        # realsense configuration
        try:
            if self.side :
                print("HOLA");
                config = rs.config()
                config.enable_device(self.device_serial_side)
                config.enable_stream(rs.stream.pose)
                pipeline_side = rs.pipeline()
                pipeline_side.start(config)
                self.cameras_list.append(pipeline_side)

            if self.back:
                config = rs.config()
                config.enable_device(self.device_serial_back)
                config.enable_stream(rs.stream.pose)
                pipeline_back = rs.pipeline()
                pipeline_back.start(config)
                self.cameras_list.append(pipeline_back)

            for i in self.num_cameras :
                config = rs.config()
                config.enable_device()

        except Exception as e:
            print("Error initializing camera")
            print(e)
            sys.exit(-1)

        # Transform managet
        # self.tm = TransformManager()
        #
        # #initial empty translation
        # self.tm.add_transform("origin", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                              ([0.0, 0.0, 0.0]),
        #                                                              [0.0, 0.0, 0.0]))
        # self.tm.add_transform("slam_sensor", "measure", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             ([0.0,0.0,0.0]),
        #                                                              [0.0,0.0,0.0]))
        #
        #
        # # get slam_sensor_0 coordinates in the robot's frame. Read them from config file
        # self.tm.add_transform("robot", "slam_sensor",
        #                       pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([0, 0, 0]), [0, 0, 0])
        #                       )
        # insert here the second slam sensor
        #

        # when requested return an average of both sensors.


        self.angles = [0.0,0.0,0.0]
        return True


    @QtCore.Slot()
    def compute(self):
        #frames = self.pipeline.wait_for_frames()
        #f = frames.first_or_default(rs.stream.pose)
        self.data_list = []
        self.data_angles = []
        for frames in self.cameras_list :
            frame = frames.wait_for_frames()
            f = frame.first_or_default (rs.stream.pose)
            self.data_list.append(f.as_pose_frame().get_pose_data())


        # Cast the frame to pose_frame and get its data
        self.firsttime = True
        #self.data = f.as_pose_frame().get_pose_data()


        for data in self.data_list :
            tm.add_transform("world", "slam_sensor", pytr.transform_from_pq([data.translation.x * 1000.0,
                                                                             -data.translation.z * 1000.0,
                                                                             data.translation.y * 1000.0,
                                                                             data.rotation.w,
                                                                             data.rotation.x,
                                                                             data.rotation.y,
                                                                             data.rotation.z]))

            self.data_angles.append(self.quaternion_to_euler_angle(data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z))

<<<<<<< HEAD
        # self.tm.add_transform("slam_sensor", "measure", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #                                                             (self.angles),
        #                                                             [data.translation.x*1000.0,
        #                                                              -data.translation.z*1000.0,
        #                                                              data.translation.y*1000.0]))
        #
        #self.tm.add_transform("world", "robot", pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz
        #(angles),
        #[data.translation.x*1000.0,
        #-data.translation.z*1000.0,
        #data.translation.y*1000.0]))
        #self.angles = self.quaternion_to_euler_angle(data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z)

        # t = self.tm.get_transform("measure", "origin")
        # print("\r Device Position: ", t[0][3], t[1][3], t[2][3], self.angles, end="\r")
        #print(data.translation)
        #if self.print:
        print("\r Device Position: ", -self.data.translation.x*1000, self.data.translation.z*1000, self.data.translation.y*1000, self.angles, end="\r")
=======
>>>>>>> 64bf305ca6261c5ca8305d649f3f95e57582b85e

        i = 0
        for data in self.data_list :
            print("\r Device Position: ", i , " Datos: ", -data.translation.x*1000, data.translation.z*1000, data.translation.y*1000, end="\r")
            i = i + 1

    def quaternion_to_euler_angle(self, w, x, y, z):

        #print(w,x,y,z)
        qx = x
        qy = y
        qz = z
        qw = w

        a1 = np.arctan2(2*qy*qw-2*qx*qz, 1- 2*qy*qy - 2*qz*qz)
        a2 = np.arcsin(2*qx*qy + 2*qz*qw)
        a0 = np.arctan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz)

        if np.isclose(qx*qy + qz*qw, 0.5):
            a1 = 2.0 * np.arctan2(qx,qw)
            a0 = 0.0
        if np.isclose(qx*qy + qz*qw, -0.5):
            a1 = -2.0 * np.arctan2(qx,qw)
            a0 = 0.0
        return a0,a1,a2

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    # =============== Methods for Component Implements ==================
    # ===================================================================

    #
    # IMPLEMENTATION of getFullPoseEuler method from FullPoseEstimation interface
    #
    def FullPoseEstimation_getFullPoseEuler(self):
        ret = RoboCompFullPoseEstimation.FullPoseEuler()
        #t = self.tm.get_transform("measure", "origin")
        #angles = self.quaternion_to_euler_angle(data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z)

        # rot = t[0:3, 0:3]
        # angles = pyrot.extrinsic_euler_xyz_from_active_matrix(rot)
        # ret.x = t[0][3]
        # ret.y = t[1][3]
        # ret.z = t[2][3]
        # ret.rx = self.angles[0]
        # ret.ry = self.angles[1]
        # ret.rz = self.angles[2]

        ret.x = -self.data.translation.x*1000 + 3305
        ret.y = self.data.translation.z*1000 - 21699
        ret.z = self.data.translation.y*1000
        ret.rx = self.angles[0]
        ret.ry = self.angles[1]
        ret.rz = self.angles[2]

        print("\r Device Position: ", -self.data.translation.x, self.data.translation.z, self.data.translation.y, self.angles, end="\r")

        return ret
    #
    # IMPLEMENTATION of getFullPoseMatrix method from FullPoseEstimation interface
    #
    def FullPoseEstimation_getFullPoseMatrix(self):
        ret = RoboCompFullPoseEstimation.FullPoseMatrix()
        t = self.tm.get_transform("origin", "slam_sensor")
        m = RoboCompFullPoseEstimation.FullPoseMatrix()
        m.m00 = t[0][0]
        m.m01 = t[0][1]
        m.m02 = t[0][2]
        m.m03 = t[0][3]
        m.m10 = t[1][0]
        m.m11 = t[1][1]
        m.m12 = t[1][2]
        m.m13 = t[1][3]
        m.m20 = t[2][0]
        m.m21 = t[2][1]
        m.m22 = t[2][2]
        m.m23 = t[2][3]
        m.m30 = t[3][0]
        m.m31 = t[3][1]
        m.m32 = t[3][2]
        m.m33 = t[3][3]
        return m
    #
    # IMPLEMENTATION of setInitialPose method from FullPoseEstimation interface
    #
    def FullPoseEstimation_setInitialPose(self, x, y, z, rx, ry, rz):

        #self.tm.add_transform("origin", "world",
        #pytr.transform_from(pyrot.active_matrix_from_intrinsic_euler_xyz([rx, ry, rz]), [x, y, z])
        #)
        self.data.translation.x = x;
        self.data.translation.y = y;
        self.data.translation.z = z;

    # ===================================================================
    # ===================================================================


    ######################
    # From the RoboCompFullPoseEstimationPub you can publish calling this methods:
    # self.fullposeestimationpub_proxy.newFullPose(...)

    ######################
    # From the RoboCompFullPoseEstimation you can use this types:
    # RoboCompFullPoseEstimation.FullPoseMatrix
    # RoboCompFullPoseEstimation.FullPoseEuler

