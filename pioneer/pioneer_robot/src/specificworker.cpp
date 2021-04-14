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
#include <cppitertools/zip.hpp>
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
    robot->lock();
        robot->disableMotors();
    robot->unlock();
    ArLog::log(ArLog::Normal, "Ending robot thread...");
    robot->stopRunning();

    // wait for the thread to stop
    robot->waitForRunExit();

    // exit
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Exiting.");
    Aria::exit(0);
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	return true;
}

// int SpecificWorker::rate(){
//         instantiate dynamically to avoid stack unwinding before the process terminates
//         QProcess *iwconfig = new QProcess();
//         catch data output
//         QObject::connect(iwconfig, &QProcess::readyRead, [iwconfig] () {
//             QByteArray a = iwconfig->readAll();
//             qDebug() <<  a;
//         });
// 
//         delete process instance when done, and get the exit status to handle errors.
//         QObject::connect(iwconfig, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
//                      [=](int exitCode, QProcess::ExitStatus /*exitStatus*/){
//                          qDebug()<< "process exited with code " << exitCode;
//                          iwconfig->deleteLater();
//                      });
// 
//         start the process after making signal/slots connections
//     iwconfig->start("iwconfig");
//     return 0;
// }

void SpecificWorker::initialize(int period)
{

	std::cout << "Initialize worker" << std::endl;
	Aria::init();

    robot = new ArRobot();
    ArArgumentBuilder *args = new ArArgumentBuilder(); //  never freed
    ArArgumentParser *argparser = new ArArgumentParser(args); // Warning never freed
    argparser->loadDefaultArguments(); // adds any arguments given in /etc/Aria.args.  Useful on robots with unusual serial port or baud rate (e.g. pioneer lx)
    conn = new ArRobotConnector(argparser, robot); // warning never freed
    //ArRobotConnector robotConnector(argparser, robot);
    if(!conn->connectRobot())
    {
        qInfo()<< __FUNCTION__ << " SimpleMotionCommands: Could not connect to the robot->";
        std::terminate();
    }

    robot->runAsync(true);
    robot->lock();
        robot->enableMotors();
        robot->enableSonar();
        numSonars = robot->getNumSonar();
    robot->unlock();
    // Tamaño vectores sonars
    sonar.resize(numSonars);
    sonarPose.resize(numSonars);

    connect(&timerRSSI, &QTimer::timeout, this, &SpecificWorker::rate);
    timerRSSI.start(1000);

    reloj_seguridad.restart();
    new_command.store(false);
    connect(&timerWatchdog, &QTimer::timeout, this, &SpecificWorker::controlParadaBase);
    timerWatchdog.start(1000);

    this->Period = period;
	if(this->startup_check_flag)
		this->startup_check();
	else
		timer.start(Period);
}

void manejador(int Snum);

void SpecificWorker::compute()
{
   // if(!ejecucion) break;
    if(new_command.load())
        reloj_seguridad.restart();

    //signal(SIGUSR1,manejador);
    fillSonarDistances();
    fillSonarPose();
}

//void manejador(int Snum){
//    ejecucion=false;
//}

//Hay que quitar el void por el numero; debemos devolver el num en el controller
void SpecificWorker::rate()
{
    // instantiate dynamically to avoid stack unwinding before the process terminates
    QProcess *iwconfig = new QProcess();
    // catch data output

    QObject::connect(iwconfig, &QProcess::readyRead, [iwconfig, this] () {
        char c4 [3];
        QByteArray a = iwconfig->readAll();
        int indexRate = a.indexOf("=", 11);
        c4[0] = a.at(indexRate+1);;
        c4[1] = a.at(indexRate+2);
        c4[2] = a.at(indexRate+3);;
        this->quality_rssi.store(atoi(c4));
        // qInfo()<<"Link Quality: " << this->quality_rssi << "/100";

    });

    // delete process instance when done, and get the exit status to handle errors.
    QObject::connect(iwconfig, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                     [=](int exitCode, QProcess::ExitStatus /*exitStatus*/){
                         qDebug()<< "process exited with code " << exitCode;
                         iwconfig->deleteLater();
                     });

    // start the process after making signal/slots connections
    iwconfig->start("iwconfig");
}


/////////////////////////////////////////////////////////////////////////////////
/// SERVANTS
/////////////////////////////////////////////////////////////////////////////////


RoboCompBatteryStatus::TBattery SpecificWorker::BatteryStatus_getBatteryState()
{
    RoboCompBatteryStatus::TBattery battery;
    robot->lock();
        battery.percentage = robot->getBatteryVoltageNow();
    robot->unlock();
    return battery;
}

//////////////////////////////////////

void SpecificWorker::DifferentialRobot_correctOdometer(int x, int z, float alpha)
{}

void SpecificWorker::DifferentialRobot_getBasePose(int &x, int &z, float &alpha)
{
    robot->lock();
        x = robot->getX();
        z = robot->getY();
        alpha = robot->getTh();
    robot->unlock();
}

void SpecificWorker::DifferentialRobot_getBaseState(RoboCompGenericBase::TBaseState &state)
{
    robot->lock();
        state.x = robot->getX();
        state.z = robot->getY();
        state.alpha = robot->getTh();
        state.advVz = robot->getVel();
        state.rotV = robot->getRotVel();
    robot->unlock();
    ArLog::log(ArLog::Normal, "simpleMotionCommands: Pose=(%.2f,%.2f,%.2f), Trans. Vel=%.2f, Rot. Vel=%.2f",
               state.x, state.z, state.alpha, state.advVz, state.rotV);

}

void SpecificWorker::DifferentialRobot_resetOdometer()
{}

void SpecificWorker::DifferentialRobot_setOdometer(RoboCompGenericBase::TBaseState state)
{}

void SpecificWorker::DifferentialRobot_setOdometerPose(int x, int z, float alpha)
{}

void SpecificWorker::DifferentialRobot_setSpeedBase(float adv, float rot)
{
    if( adv < MAX_ADV and adv > -MAX_ADV and rot > -MAX_ROT and rot < MAX_ROT)
    {
        robot->lock();
            robot->setVel(adv);
            robot->setRotVel(rot);
        robot->unlock();
    }
    else
        std::cout << __FUNCTION__ << "Commanded velocity out of bounds " << adv << " mm/s " << rot << " rads/sg" << std::endl;
}

void SpecificWorker::DifferentialRobot_stopBase()
{
    robot->lock();
        robot->stop();
    robot->unlock();
}

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::Tstatic QTime reloj = QTime::currentTime();


int SpecificWorker::startup_check()
{
    std::cout << "Startup check" << std::endl;
    QTimer::singleShot(200, qApp, SLOT(quit()));
    return 0;
}

RoboCompUltrasound::SensorsState SpecificWorker::Ultrasound_getAllSensorDistances() 
{
//implementCODE
    return sonar;
}

void SpecificWorker::fillSonarDistances(){
    try{
        robot->lock();
        for (int i = 0; i < numSonars; i++){
            sonar[i] = robot->getSonarReading(i)->getRange();
        }
        robot->unlock();
    } catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
    
}

void SpecificWorker::fillSonarPose(){
    try{
        robot->lock();
        for(int i = 0; i < numSonars; i++){
            ArPose aux =  robot->getSonarReading(i)->getSensorPosition();
            sonarPose[i].x = aux.getX();
            sonarPose[i].y = aux.getY();
        }
    robot->unlock();
    } catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}

RoboCompUltrasound::SensorParamsList SpecificWorker::Ultrasound_getAllSensorParams()
{
  //implemetCODE

}

RoboCompUltrasound::SonarPoseList SpecificWorker::Ultrasound_getAllSonarPose()
{
//implementCODE
    return sonarPose;
}

RoboCompUltrasound::BusParams SpecificWorker::Ultrasound_getBusParams()
{
//implementCODE

}

int SpecificWorker::Ultrasound_getSensorDistance(std::string sensor)
{
//implementCODE

}

RoboCompUltrasound::SensorParams SpecificWorker::Ultrasound_getSensorParams(std::string sensor)
{
//implementCODE

}

int SpecificWorker::Ultrasound_getSonarsNumber()
{
    return numSonars;
}

RoboCompRSSIStatus::TRSSI SpecificWorker::RSSIStatus_getRSSIState()
{
    RoboCompRSSIStatus::TRSSI res;
    res.percentage = this->quality_rssi.load();
    return res;
}


/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

//SUBSCRIPTION to sendData method from JoystickAdapter interface
void SpecificWorker::JoystickAdapter_sendData(RoboCompJoystickAdapter::TData data)
{
    float adv_speed = 0;
    float rot_speed = 0;
    for(auto  &&[a, b] :  iter::zip(data.axes, data.buttons))
    {
        if(a.name == "advance" && a.value > 4){
            adv_speed = std::clamp(a.value, -1000.f, 1000.f);
            std::cout << "Advance" << std::endl;
        }
        if(a.name == "turn")
            rot_speed = std::clamp(a.value, -100.f, 100.f);
        
        if(b.name == "back") {
            adv_speed = -50.f; //(std::clamp(-50.f, -1000.f, 1000.f));
            std::cout << "BACK" << std::endl;
        }
    }
    
    if(fabs(rot_speed) < 1) rot_speed = 0;
    if(fabs(adv_speed) < 4) adv_speed = 0;
    
    if(adv_speed != 0 or rot_speed !=0)
        this->new_command.store(true);

        robot->lock();
    	qInfo() << adv_speed;
        robot->setVel(adv_speed);
        robot->setRotVel(rot_speed);
    robot->unlock();
}

void SpecificWorker::controlParadaBase()
{
    qInfo() << __FUNCTION__ << "entro en controlPAradaa";
    if(reloj_seguridad.elapsed() > 3000)
    {
        robot->lock();
            robot->stop();
        robot->unlock();
    }

}


/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

/**************************************/
// From the RoboCompRSSIStatus you can use this types:
// RoboCompRSSIStatus::TRSSI

/**************************************/
// From the RoboCompUltrasound you can use this types:
// RoboCompUltrasound::BusParams
// RoboCompUltrasound::SensorParams
// RoboCompUltrasound::SonarPose

/**************************************/
// From the RoboCompJoystickAdapter you can use this types:
// RoboCompJoystickAdapter::AxisParams
// RoboCompJoystickAdapter::ButtonParams
// RoboCompJoystickAdapter::TData

