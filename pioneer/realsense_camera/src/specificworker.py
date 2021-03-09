#!/usr/bin/python3
# -*- coding: utf-8 -*-
#
# Copyright (C) 2019 by YOUR NAME HERE
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
import pyrealsense2 as rs
import numpy as np
import time
import cv2
from PySide2.QtCore import QMutexLocker


class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check):
        super(SpecificWorker, self).__init__(proxy_map)
        self.params = {}
        self.width = 0
        self.height = 0
        self.cameraid = 0
        self.adepth = []
        self.bdepth = []
        self.acolor = []
        self.bcolor = []
        self.points = []
        self.viewimage = False
        self.timer.timeout.connect(self.compute)
        self.Period = 1

    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        self.params = params
        self.width = int(self.params["width"])
        self.height = int(self.params["height"])
        self.depth = int(self.params["depth"])
        self.cameraid = int(self.params["cameraid"])
        self.viewimage = "true" in self.params["viewimage"]
        self.publishimage = "true" in self.params["publishimage"]
        self.verticalflip = "true" in self.params["verticalflip"]
        self.horizontalflip = "true" in self.params["horizontalflip"]
        self.depth_focal_x = int(self.params["depth_focal_x"])
        self.depth_focal_y = int(self.params["depth_focal_y"])
        self.color_focal_x = int(self.params["color_focal_x"])
        self.color_focal_y = int(self.params["color_focal_y"])
        self.contFPS = 0
        self.start = time.time()
        self.capturetime = time.time()
        self.initialize()
        self.timer.start(self.Period)

        return True

    def initialize(self):
        print("Initialize")

        # realsense configuration
        try:
            config = rs.config()
            config.enable_device(self.params["device_serial"])

            # OJO AJUSTAR CON LOS VLAORES DE CONFIG

            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)

            self.pointcloud = rs.pointcloud()
            self.pipeline = rs.pipeline()
            cfg = self.pipeline.start(config)
        #            profile = cfg.get_stream(rs.stream.color) # Fetch stream profile for depth stream
        #            intr = profile.as_video_stream_profile().get_intrinsics() # Downcast to video_stream_profile and fetch intrinsics
        #            print (intr.fx, intr.fy)
        #            depth_scale = cfg.get_device().first_depth_sensor().get_depth_scale()
        #            print("Depth Scale is: " , depth_scale)
        #            sys.exit(-1)
        except Exception as e:
            print("Error initializing camera")
            print(e)
            sys.exit(-1)

    @QtCore.Slot()
    def compute(self):
        frames = self.pipeline.wait_for_frames()
        if not frames:
            return

        self.capturetime = time.time()
        depthData = frames.get_depth_frame()
        self.bdepth = np.asanyarray(depthData.get_data(), dtype=np.float32)
        self.bcolor = np.asanyarray(frames.get_color_frame().get_data())

        #if self.width != 640 or self.height != 480:
        #    self.bcolor = cv2.resize(self.bcolor, (self.width, self.height), interpolation=cv2.INTER_AREA)

        if self.horizontalflip:
            self.bcolor = cv2.flip(self.bcolor, 1)
            self.bdepth = cv2.flip(self.bdepth, 1)
        if self.verticalflip:
            self.bcolor = cv2.flip(self.bcolor, 0)
            self.bdepth = cv2.flip(self.bdepth, 0)

        # SWAP
        self.mutex.lock()
        self.acolor, self.bcolor = self.bcolor, self.acolor
        self.adepth, self.bdepth = self.bdepth, self.adepth
        self.mutex.unlock()

        if self.viewimage:
            cv2.imshow("Color_frame", self.bcolor)
            cv2.setMouseCallback("Color_frame", self.mousecallback)
            cv2.waitKey(1)

        if self.publishimage:
            im = TImage()
            im.cameraID = self.cameraid
            im.width = self.width
            im.height = self.height
            im.focalx = self.color_focal_x
            im.focaly = self.color_focal_y
            im.depth = self.depth
            im.image = self.acolor

            dep = TDepth()
            dep.cameraID = self.cameraid
            dep.width = self.width
            dep.height = self.height
            dep.focalx = self.depth_focal_x
            dep.focaly = self.depth_focal_y
            dep.depth = self.adepth

            try:
                dep.alivetime = (time.time() - self.capturetime) * 1000
                im.alivetime = (time.time() - self.capturetime) * 1000
                self.camerargbdsimplepub_proxy.pushRGBD(im, dep)
            except Exception as e:
                print("Error on camerabody data publication")
                print(e)

        if time.time() - self.start > 1:
            print("FPS:", self.contFPS)
            self.start = time.time()
            self.contFPS = 0
        self.contFPS += 1
        return True

    # =============== Methods for Component Implements ==================
    # ===================================================================
    ### CameraRGBDSimple
    #
    # getAll
    #
    def CameraRGBDSimple_getAll(self, nombre):
        ret = RoboCompCameraRGBDSimple.TRGBD()
        ret.image.cameraID = self.cameraid
        ret.image.width = self.width
        ret.image.height = self.height
        ret.image.depth = self.depth
        ret.image.focalx = self.color_focal_x
        ret.image.focaly = self.color_focal_y
        ret.image.image = self.acolor
        ret.image.alivetime = (time.time() - self.capturetime) * 1000

        ret.depth.cameraID = self.cameraid
        ret.depth.width = self.width
        ret.depth.height = self.height
        ret.depth.depth = self.adepth
        ret.depth.focalx = self.depth_focal_x
        ret.depth.focaly = self.depth_focal_y
        ret.depth.alivetime = (time.time() - self.capturetime) * 1000

        return ret

    #
    # getDepth
    #
    def CameraRGBDSimple_getDepth(self, nombre):
        # locker = QMutexLocker(self.mutex)
        dep = TDepth()
        dep.cameraID = self.cameraid
        dep.width = self.width
        dep.height = self.height
        dep.depth = self.adepth
        dep.focalx = self.depth_focal_x
        dep.focaly = self.depth_focal_y
        dep.alivetime = (time.time() - self.capturetime) * 1000
        return dep

    #
    # getImage
    #
    def CameraRGBDSimple_getImage(self, nombre):

        im = RoboCompCameraRGBDSimple.TImage()
        im.cameraID = self.cameraid
        im.width = self.width
        im.height = self.height
        im.focalx = self.color_focal_x
        im.focaly = self.color_focal_y
        img_gray = cv2.cvtColor(self.acolor, cv2.COLOR_BGR2GRAY)
        im.depth = self.depth
        im.image = self.acolor
        im.alivetime = (time.time() - self.capturetime) * 1000
        return im

    def mousecallback(self, event, x, y, flags, param):
        # grab references to the global variables
        global refPt, cropping
        Z = self.depth[y, x]
        X = (x - 320) * Z / 617
        Y = (y - 240) * Z / 616
        print("image", x, y, "point", X, Y, Z)
        # if the left mouse button was clicked, record the starting
        # (x, y) coordinates and indicate that cropping is being
        # performed
