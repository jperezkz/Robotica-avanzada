#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
#    Copyright (C) 2020 by YOUR NAME HERE
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

from genericworker import *
import os, time, queue
from bisect import bisect_left
from os.path import dirname, join, abspath
from pyrep import PyRep
#from pyrep.robots.mobiles.viriato import Viriato
#from pyrep.robots.mobiles.viriato import Viriato
#from pyrep.robots.mobiles.youbot import YouBot
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.shape import Object
from pyrep.objects.joint import Joint
import numpy as np
import numpy_indexed as npi
from itertools import zip_longest
import cv2

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        SCENE_FILE = '../etc/gen3-robolab.ttt'

        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        
        #self.robot = Viriato()
        #self.robot = YouBot()
        self.robot_object = Shape("gen3")

        self.cameras = {}
        self.camera_arm_name = "camera_arm"
        cam = VisionSensor(self.camera_arm_name)
        self.cameras[self.camera_arm_name] = {"handle": cam,
                                               "id": 0,
                                               "angle": np.radians(cam.get_perspective_angle()),
                                               "width": cam.get_resolution()[0],
                                               "height": cam.get_resolution()[1],
                                               "focal": (cam.get_resolution()[0] / 2) / np.tan(
                                                   np.radians(cam.get_perspective_angle() / 2)),
                                               "rgb": np.array(0),
                                               "depth": np.ndarray(0)}

        self.joystick_newdata = []
        self.last_received_data_time = 0

    #@QtCore.Slot()
    def compute(self):
        cont = 0
        start = time.time()
        while True:
            self.pr.step()
            #self.read_joystick()
            self.read_camera(self.cameras[self.camera_arm_name])

            elapsed = time.time()-start
            if elapsed < 0.05:
                time.sleep(0.05-elapsed)
            cont += 1
            if elapsed > 1:
                print("Freq -> ", cont)
                cont = 0
                start = time.time()

    ###########################################
    def read_camera(self, cam):
        image_float = cam["handle"].capture_rgb()
        depth = cam["handle"].capture_depth(True)
        image = cv2.normalize(src=image_float, dst=None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        cam["rgb"] = RoboCompCameraRGBDSimple.TImage(cameraID=cam["id"], width=cam["width"], height=cam["height"],
                                                     depth=3, focalx=cam["focal"], focaly=cam["focal"],
                                                     alivetime=time.time(), image=image.tobytes())
        cam["depth"] = RoboCompCameraRGBDSimple.TDepth(cameraID=cam["id"], width=cam["handle"].get_resolution()[0],
                                                       height=cam["handle"].get_resolution()[1],
                                                       focalx=cam["focal"], focaly=cam["focal"],
                                                       alivetime=time.time(), depthFactor=1.0, depth=depth.tobytes())

        # try:
        #    self.camerargbdsimplepub_proxy.pushRGBD(cam["rgb"], cam["depth"])
        # except Ice.Exception as e:
        #    print(e)

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata: #and (time.time() - self.joystick_newdata[1]) > 0.1:
            self.update_joystick(self.joystick_newdata[0])
            self.joystick_newdata = None
            self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                self.robot.set_base_angular_velocites([0, 0, 0])

    
    def update_joystick(self, datos):
        adv = 0.0
        rot = 0.0
        side = 0.0
        #linear_vel, ang_vel = self.robot_object.get_velocity()
        for x in datos.axes:
            if x.name == "advance":
                adv = x.value if np.abs(x.value) > 1 else 0
            if x.name == "rotate":
                rot = x.value if np.abs(x.value) > 0.01 else 0
            if x.name == "side":
                side = x.value if np.abs(x.value) > 1 else 0

        print("Joystick ", [adv, side, rot], converted)
        #self.robot.set_base_angular_velocites(converted)


    ##################################################################################
    # SUBSCRIPTION to sendData method from JoystickAdapter interface
    ###################################################################################
    def JoystickAdapter_sendData(self, data):
        self.joystick_newdata = [data, time.time()]

    ##################################################################################
    #                       Methods for CameraRGBDSimple
    # ===============================================================================
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self, camera):
        return RoboCompCameraRGBDSimple.TRGBD(self.cameras[camera]["rgb"], self.cameras[camera]["depth"])

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, camera):
        return self.cameras[camera]["depth"]
    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, camera):
        return self.cameras[camera]["rgb"]

    #######################################################
    #### Laser
    #######################################################

    #
    # getLaserAndBStateData
    #
    def Laser_getLaserAndBStateData(self):
        bState = RoboCompGenericBase.TBaseState()
        return self.ldata, bState

    #
    # getLaserConfData
    #
    def Laser_getLaserConfData(self):
        ret = RoboCompLaser.LaserConfData()
        return ret

    #
    # getLaserData
    #
    def Laser_getLaserData(self):
        return self.ldata

    ##############################################
    ## Omnibase
    #############################################

    #
    # correctOdometer
    #
    def OmniRobot_correctOdometer(self, x, z, alpha):
        pass

    #
    # getBasePose
    #
    def OmniRobot_getBasePose(self):
        #
        # implementCODE
        #
        x = self.bState.x
        z = self.bState.z
        alpha = self.bState.alpha
        return [x, z, alpha]

    #
    # getBaseState
    #
    def OmniRobot_getBaseState(self):
        return self.bState

    #
    # resetOdometer
    #
    def OmniRobot_resetOdometer(self):
        pass

    #
    # setOdometer
    #
    def OmniRobot_setOdometer(self, state):
        pass

    #
    # setOdometerPose
    #
    def OmniRobot_setOdometerPose(self, x, z, alpha):
        pass

    #
    # setSpeedBase
    #
    def OmniRobot_setSpeedBase(self, advx, advz, rot):
        self.speed_robot = self.convert_base_speed_to_radians(advz, advx, rot)

    #
    # stopBase
    #
    def OmniRobot_stopBase(self):
        pass

    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            dummy.set_name(name)
        else:
            dummy = Dummy(name)
            parent_frame_object = None
            #print("Coppelia ", name, pose.x/1000, pose.y/1000, pose.z/1000)
            dummy.set_position([pose.x / 1000., pose.y / 1000., pose.z / 1000.], parent_frame_object)
            #dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)

