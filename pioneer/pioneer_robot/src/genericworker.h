/*
 *    Copyright (C) 2021 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef GENERICWORKER_H
#define GENERICWORKER_H

#include "config.h"
#include <stdint.h>
#include <qlog/qlog.h>
#include <CommonBehavior.h>

#include <BatteryStatus.h>
#include <DifferentialRobot.h>
#include <GenericBase.h>
#include <JoystickAdapter.h>
#include <RSSIStatus.h>
#include <Ultrasound.h>


#define CHECK_PERIOD 5000
#define BASIC_PERIOD 100


using TuplePrx = std::tuple<>;


class GenericWorker : public QObject
{
Q_OBJECT
public:
	GenericWorker(TuplePrx tprx);
	virtual ~GenericWorker();
	virtual void killYourSelf();
	virtual void setPeriod(int p);

	virtual bool setParams(RoboCompCommonBehavior::ParameterList params) = 0;
	QMutex *mutex;



	virtual RoboCompBatteryStatus::TBattery BatteryStatus_getBatteryState() = 0;
	virtual void DifferentialRobot_correctOdometer(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_getBasePose(int &x, int &z, float &alpha) = 0;
	virtual void DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state) = 0;
	virtual void DifferentialRobot_resetOdometer() = 0;
	virtual void DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state) = 0;
	virtual void DifferentialRobot_setOdometerPose(int x, int z, float alpha) = 0;
	virtual void DifferentialRobot_setSpeedBase(float adv, float rot) = 0;
	virtual void DifferentialRobot_stopBase() = 0;
	virtual RoboCompRSSIStatus::TRSSI RSSIStatus_getRSSIState() = 0;
	virtual RoboCompUltrasound::SensorsState Ultrasound_getAllSensorDistances() = 0;
	virtual RoboCompUltrasound::SensorParamsList Ultrasound_getAllSensorParams() = 0;
	virtual RoboCompUltrasound::SonarPoseList Ultrasound_getAllSonarPose() = 0;
	virtual RoboCompUltrasound::BusParams Ultrasound_getBusParams() = 0;
	virtual int Ultrasound_getSensorDistance(std::string sensor) = 0;
	virtual RoboCompUltrasound::SensorParams Ultrasound_getSensorParams(std::string sensor) = 0;
	virtual void JoystickAdapter_sendData (RoboCompJoystickAdapter::TData data) = 0;

protected:

	QTimer timer;
	int Period;

private:


public slots:
	virtual void compute() = 0;
	virtual void initialize(int period) = 0;
	
signals:
	void kill();
};

#endif
