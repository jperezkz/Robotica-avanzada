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
from pyrep.backend import simConst
import numpy as np
import numpy_indexed as npi
from itertools import zip_longest
import cv2

class TimeControl:
    def __init__(self, period_):
        self.counter = 0
        self.start = time.time()  # it doesn't exist yet, so initialize it
        self.start_print = time.time()  # it doesn't exist yet, so initialize it
        self.period = period_

    def wait(self):
        elapsed = time.time() - self.start
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        self.start = time.time()
        self.counter += 1
        if time.time() - self.start_print > 1:
            print("Freq -> ", self.counter, " Hz")
            self.counter = 0
            self.start_print = time.time()

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map):
        super(SpecificWorker, self).__init__(proxy_map)
        self.pinza = False
       
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        
        SCENE_FILE = '../etc/gen3-robolab.ttt'
        self.pr = PyRep()
        self.pr.launch(SCENE_FILE, headless=False)
        self.pr.start()
        self.mode = 0
        self.bloqueo=True
        
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

    def compute(self):
        tc = TimeControl(0.05)

        while True:
            self.pr.step()
            self.read_joystick()
            self.read_camera(self.cameras[self.camera_arm_name])

            if self.pinza:
                self.pr.script_call("close@RG2", 1)
            else:
                self.pr.script_call("open@RG2", 1)

            tc.wait()

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

    ###########################################
    ### JOYSITCK read and move the robot
    ###########################################
    def read_joystick(self):
        if self.joystick_newdata:
            print(self.joystick_newdata)
            self.update_joystick(self.joystick_newdata[0])
            self.joystick_newdata = None
            self.last_received_data_time = time.time()
        else:
            elapsed = time.time() - self.last_received_data_time
            if elapsed > 2 and elapsed < 3:
                pass
    
    def update_joystick(self, datos):
        adv = advR = 0.0
        rot = rotR = 0.0
        side = sideR = 0.0
        print(datos.buttons)
        for x in datos.buttons:
            if x.name == "mode":
                self.mode += x.step
                if self.mode%2==1:
                    self.bloqueo=True
                else:
                    self.bloqueo=False
        for x in datos.axes:
            print(x.name + "" + str(x.value))
            if x.name == "X_axis":
                adv = x.value if np.abs(x.value) > 1 else 0
                advR = x.value if np.abs(x.value) > 1 else 0
            if x.name == "Y_axis":
                rot = x.value if np.abs(x.value) > 0.01 else 0
                rotR = x.value if np.abs(x.value) > 0.01 else 0
            if x.name == "Z_axis":
                side = x.value if np.abs(x.value) > 1 else 0
                sideR = x.value if np.abs(x.value) > 1 else 0
            if x.name == "gripper":
                if x.value > 1:
                    self.pr.script_call("open@RG2", 1)
                    print("abriendo")
                if x.value < -1:
                    print("cerrando")
                    self.pr.script_call("close@RG2", 1)
            dummy = Dummy("target")
            parent_frame_object = None
            position = dummy.get_position()
            orientation= dummy.get_orientation()
            if self.bloqueo==False:
                print("modo0\n\n")
                dummy.set_position([position[0]+adv/1000, position[1]+rot/1000, position[2]+side/1000], parent_frame_object)
            else:
                print("modo1\n\n")
                dummy.set_orientation([orientation[0]+advR/1000, orientation[1]+rotR/1000, orientation[2]+sideR/1000], parent_frame_object)


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


    # ===================================================================
    # CoppeliaUtils
    # ===================================================================
    def CoppeliaUtils_addOrModifyDummy(self, type, name, pose):
        if not Dummy.exists(name):
            dummy = Dummy.create(0.1)
            dummy.set_name(name)
        else:
            dummy = Dummy("target")
            #parent_frame_object = None
            parent_frame_object = Shape("gen3")
            #print("Coppelia ", name, pose.x/1000, pose.y/1000, pose.z/1000)
            #dummy.set_position([pose.x / 1000., pose.y / 1000., pose.z / 1000.], parent_frame_object)
            dummy.set_position([pose.x, pose.y, pose.z], parent_frame_object)
            print(pose.x,pose.y,pose.z)
            print(dummy.get_position())
            dummy.set_orientation([pose.rx, pose.ry, pose.rz], parent_frame_object)
    def KinovaArm_getCenterOfTool(self, referencedTo):
        ret = RoboCompKinovaArm.TPose()
        parent_frame_object = Shape('gen3')
        tip = Dummy("tip")
        pos = tip.get_position(parent_frame_object)
        rot = tip.get_orientation(parent_frame_object)
        ret.x = pos[0]
        ret.y = pos[1]
        ret.z = pos[2]
        ret.rx = rot[0]
        ret.ry = rot[1]
        ret.rz = rot[2]
        return ret

    def KinovaArm_openGripper(self):
        #self.pr.script_call("open@RG2", 1)
        print("Opening gripper")
        self.pinza = False

    def KinovaArm_closeGripper(self):
        #self.pr.script_call("close@RG2", 1)
        print("Closing gripper")
        self.pinza = True

    #
    # IMPLEMENTATION of setCenterOfTool method from KinovaArm interface
    #
    def KinovaArm_setCenterOfTool(self, pose, referencedTo):
        target = Dummy("target")
        parent_frame_object = Shape('gen3')
        position = target.get_position(parent_frame_object)
        #target.set_position([position[0] + pose.x / 1000, position[1] + pose.y / 1000, position[2] + pose.z / 1000], parent_frame_object)
        target.set_position([position[0] + pose.x / 1000, position[1] + pose.y / 1000, position[2] + pose.z / 1000],parent_frame_object)
    def KinovaArm_setPosition(self, pose, referencedTo):
        target = Dummy("target")
        parent_frame_object = Shape('gen3')
        position = target.get_position(parent_frame_object)
        target.set_position([pose.x, pose.y, pose.z], parent_frame_object)
