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

import sys, Ice, os

ROBOCOMP = ''
try:
    ROBOCOMP = os.environ['ROBOCOMP']
except KeyError:
    print('$ROBOCOMP environment variable not set, using the default value /opt/robocomp')
    ROBOCOMP = '/opt/robocomp'

Ice.loadSlice("-I ./src/ --all ./src/CommonBehavior.ice")
import RoboCompCommonBehavior

Ice.loadSlice("-I ./src/ --all ./src/CameraRGBDSimple.ice")
import RoboCompCameraRGBDSimple 
Ice.loadSlice("-I ./src/ --all ./src/CameraRGBDSimplePub.ice")
import RoboCompCameraRGBDSimplePub
Ice.loadSlice("-I ./src/ --all ./src/JoystickAdapter.ice")
import RoboCompJoystickAdapter
Ice.loadSlice("-I ./src/ --all ./src/CoppeliaUtils.ice")
import RoboCompCoppeliaUtils
Ice.loadSlice("-I ./src/ --all ./src/JointMotorPub.ice")
import RoboCompJointMotorPub
Ice.loadSlice("-I ./src/ --all ./src/JointMotor.ice")
import RoboCompJointMotor
Ice.loadSlice("-I ./src/ --all ./src/KinovaArmPub.ice")
import RoboCompKinovaArmPub

import camerargbdsimpleI
import joystickadapterI
import coppeliautilsI

class GenericWorker():

    #kill = QtCore.Signal()

    def __init__(self, mprx):
        super(GenericWorker, self).__init__()

        self.camerargbdsimplepub_proxy = mprx["CameraRGBDSimplePubPub"]
        self.jointmotorpub_proxy = mprx["JointMotorPubPub"]
        self.kinovaarmpub_proxy = mprx["KinovaArmPubPub"]