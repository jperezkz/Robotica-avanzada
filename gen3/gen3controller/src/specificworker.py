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

from PySide2.QtCore import QTimer
from PySide2.QtWidgets import QApplication
from genericworker import *
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from yolov4 import Detector
from PIL import Image

class SpecificWorker(GenericWorker):
    def __init__(self, proxy_map, startup_check=False):
        super(SpecificWorker, self).__init__(proxy_map)

        # image
        self.image = []
        self.depth = []
        self.camera_name = "camera_arm"
        self.d = Detector(gpu_id=0)
        self.Period = 100
        if startup_check:
            self.startup_check()
        else:
            self.timer.timeout.connect(self.compute)
            self.timer.start(self.Period)
        self.position = 0
        self.objetos = {"cup":[]}
        self.centros= False
    def __del__(self):
        print('SpecificWorker destructor')

    def setParams(self, params):
        #try:
        #	self.innermodel = InnerModel(params["InnerModelPath"])
        #except:
        #	traceback.print_exc()
        #	print("Error reading config params")
        return True


    @QtCore.Slot()
    def compute(self):
        #print('SpecificWorker.compute...')
        all = self.camerargbdsimple_proxy.getAll(self.camera_name)
        #self.draw_image(all.image)
        #procesar imagen
        centre = self.procesarImagen(all.image)
        detections = self.yolo(all.image)
        #self.coger(detections, all.image)
        return True

    # ===================================================================
    def draw_image(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        plt.figure(1)
        plt.clf()
        plt.imshow(color)
        plt.title('Front Camera ')
        plt.pause(.1)

    def startup_check(self):
        QTimer.singleShot(200, QApplication.instance().quit)

    def yolo(self, color_):

        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)

        img = Image.fromarray(color)
        img_arr = np.array(img.resize((self.d.network_width(), self.d.network_height())))
        detections = self.d.perform_detect(image_path_or_buf=img_arr, show_image=False)
        for detection in detections:
            if detection.class_confidence * 100 > 10:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')

            else:
                print("Objeto no detectado")
        print("-"*100)

        return detections

    def procesarImagen(self, color_):
        color = np.frombuffer(color_.image, np.uint8).reshape(color_.height, color_.width, color_.depth)
        color = cv.medianBlur(color, 5)
        color = cv.cvtColor(color, cv.COLOR_BGR2GRAY)
        cimg = cv.cvtColor(color, cv.COLOR_GRAY2BGR)
        circles = cv.HoughCircles(color, cv.HOUGH_GRADIENT, 1, 20, param1=60, param2=60, minRadius=0,maxRadius=0)
        #print(type(circles))
        centre = []
        if type(circles) is not type(None):
            circles = np.uint16(np.around(circles))
            for i in circles[0, :]:
                cv.circle(cimg, (i[0], i[1]), i[2], (0, 255, 0), 2)
                cv.circle(cimg, (i[0], i[1]), 2, (0, 0, 255), 3)
                centre.append(i[0])
                centre.append(i[1])
                centre.append(i[2])
            print(centre[0], centre[1])
        plt.figure(1)
        plt.clf()
        plt.imshow(cimg)
        plt.title('Front Camera ')
        plt.pause(.1)
        plt.imshow(cimg)
            #self.draw_image(cimg)
        return centre

    def coger(self, detections, image):
        for detection in detections:
            box = detection.width, detection.height
            if detection.class_name == 'cup':
                self.objetos[detection.class_name] = box

        centre = self.procesarImagen(image)
        if centre or self.centros==True:
            self.centros=True
            self.pinza(centre)
        elif self.centros==False:
            self.mover(self.objetos["cup"])

    def mover(self,box):
        #tip = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        pos = RoboCompKinovaArm.TPose()


        pos.x = pos.x - box[0] / 100
        pos.y = pos.y - box[1] / 100
        pos.z = 0
        self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)

    def pinza(self, centre):
        tip = self.kinovaarm_proxy.getCenterOfTool(RoboCompKinovaArm.ArmJoints.base)
        pos = RoboCompKinovaArm.TPose()
        print(tip.x)
        print(tip.y)
        if centre:
            print(centre[0] / 100)
            print(centre[1] / 100)
            print("*"*100)
            pos.x=tip.x - centre[0]/100
            pos.y=tip.y - centre[1]/100
            pos.z=0
            """print(tip.x,tip.y)
            print(pos.x,pos.y)
            print(centre)"""
            """elif pos.x-tip.x<centre[2]:
            pos.x=5
            pos.y=0
            pos.z=0"""
        elif tip.z > 0.08:
            pos.x=0
            pos.y=0
            pos.z=-3
        else:
            pos.x=0
            pos.y=0
            pos.z=0
            self.kinovaarm_proxy.closeGripper()

        self.kinovaarm_proxy.setCenterOfTool(pos, RoboCompKinovaArm.ArmJoints.base)

        #self.kinovaarm_proxy.openGripper()
        #print(tip)

    ######################
    # From the RoboCompCameraRGBDSimple you can call this methods:
    # self.camerargbdsimple_proxy.getAll(...)
    # self.camerargbdsimple_proxy.getDepth(...)
    # self.camerargbdsimple_proxy.getImage(...)

    ######################
    # From the RoboCompCameraRGBDSimple you can use this types:
    # RoboCompCameraRGBDSimple.TImage
    # RoboCompCameraRGBDSimple.TDepth
    # RoboCompCameraRGBDSimple.TRGBD

    ######################
    # From the RoboCompKinovaArm you can call this methods:
    # self.kinovaarm_proxy.closeGripper(...)
    # self.kinovaarm_proxy.getCenterOfTool(...)
    # self.kinovaarm_proxy.openGripper(...)
    # self.kinovaarm_proxy.setCenterOfTool(...)
    # self.kinovaarm_proxy.setGripper(...)