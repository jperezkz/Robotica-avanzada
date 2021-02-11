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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
//	THE FOLLOWING IS JUST AN EXAMPLE
//	To use innerModelPath parameter you should uncomment specificmonitor.cpp readConfig method content
//	try
//	{
//		RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
//		std::string innermodel_path = par.value;
//		innerModel = std::make_shared(innermodel_path);
//	}
//	catch(const std::exception &e) { qFatal("Error reading config params"); }






	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;

	Aria::init();
    ArArgumentParser parser(&builder);
    parser.loadDefaultArguments();
    ArRobotConnector robotConnector(&parser, &robot);

    if(!robotConnector.connectRobot())
    {
        qInfo()<<"SimpleMotionCommands: Could not connect to the robot.";
        std::terminate();
    }

    if (!Aria::parseArgs())
    {
        Aria::logOptions();
        Aria::exit(1);
        std::terminate();
    }

    robot.runAsync(true);

	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
	}

}

void SpecificWorker::compute()
{
    //computeCODE
	//QMutexLocker locker(mutex);
	//try
	//{
	//  camera_proxy->getYImage(0,img, cState, bState);
	//  memcpy(image_gray.data, &img[0], m_width*m_height*sizeof(uchar));
	//  searchTags(image_gray);
	//}
	//catch(const Ice::Exception &e)
	//{
	//  std::cout << "Error reading from Camera" << e << std::endl;
	//}
	moveWheels();

}

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

void SpecificWorker::moveWheels(){

    robot.lock();
    ArLog::log(ArLog::Normal, "Robot: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
               robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
    robot.unlock();

    // Sleep for 3 seconds.
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Will start driving in 3 seconds...");
    ArUtil::sleep(3000);

    //Set forward velocity to 50 mm/s
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 250 mm/s for 5 sec...");
    robot.lock();
    robot.enableMotors();
    robot.setVel(250);
    robot.unlock();
    ArUtil::sleep(5000);

    ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
    robot.lock();
    robot.stop();
    robot.unlock();
    ArUtil::sleep(1000);

    ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at 10 deg/s for 5 sec...");
    robot.lock();
    robot.setRotVel(10);
    robot.unlock();
    ArUtil::sleep(5000);


    ArLog::log(ArLog::Normal, "simpleMotionCommands: Rotating at -10 deg/s for 10 sec...");
    robot.lock();
    robot.setRotVel(-10);
    robot.unlock();
    ArUtil::sleep(10000);


    ArLog::log(ArLog::Normal, "simpleMotionCommands: Driving forward at 150 mm/s for 5 sec...");
    robot.lock();
    robot.setRotVel(0);
    robot.setVel(150);
    robot.unlock();
    ArUtil::sleep(5000);


    ArLog::log(ArLog::Normal, "simpleMotionCommands: Stopping.");
    robot.lock();
    robot.stop();
    robot.unlock();
    ArUtil::sleep(1000);


    // Other motion command functions include move(), setHeading(),
    // setDeltaHeading().  You can also adjust acceleration and deceleration
    // values used by the robot with setAccel(), setDecel(), setRotAccel(),
    // setRotDecel().  See the ArRobot class documentation for more.


    robot.lock();
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f, Battery=%.2fV",
               robot.getX(), robot.getY(), robot.getTh(), robot.getVel(), robot.getRotVel(), robot.getBatteryVoltage());
    robot.unlock();


    ArLog::log(ArLog::Normal, "simpleMotionCommands: Ending robot thread...");
    robot.stopRunning();

    // wait for the thread to stop
    robot.waitForRunExit();

    // exit
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
    Aria::exit(0);
}


void SpecificWorker::DifferentialRobot_correctOdometer(int x, int z, float alpha)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_resetOdometer()
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setOdometerPose(int x, int z, float alpha)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_setSpeedBase(float adv, float rot)
{
//implementCODE

}

void SpecificWorker::DifferentialRobot_stopBase()
{
//implementCODE

}



/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

