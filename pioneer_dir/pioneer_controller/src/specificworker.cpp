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
    try
    {
        RoboCompCommonBehavior::Parameter par = params.at("InnerModelPath");
        std::string innermodel_path = par.value;
        innerModel = std::make_shared<InnerModel>(innermodel_path);
    }
    catch(const std::exception &e) { qFatal("Error reading config params"); }
    confParams = params;
	return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    // draw
    QFileInfo info1("../etc/escuela.simscene.xml");
    QFileInfo info2("../etc/escuela.json");
    qInfo() << info2.lastModified().toSecsSinceEpoch() - info1.lastModified().toSecsSinceEpoch();

    //    qInfo() << info.lastModified().toSecsSinceEpoch();
    //    if (QDateTime::currentDateTime().toSecsSinceEpoch() - info.lastModified().toSecsSinceEpoch() < 3000)
    //    {
    //        QProcess::execute("python3 /home/robocomp/robocomp/components/dsr-graph/scripts/vrep_to_json/vrep_to_qscene_json.py ../etc/escuela.simscene.xml ../etc/escuela.json");
    //        qInfo() << __FUNCTION__ << " JSON file created from XML";
    //    }

    // 2d scene initialization
    auto target_slot =  [this](QGraphicsSceneMouseEvent *e)
            {
                qDebug() << "Lambda SLOT: " << e->scenePos();
                target_buffer.put(std::move(e->scenePos()), [/*r = robot_polygon->pos()*/](auto &&t, auto &out)
                    {
                        out.pos = t;
                        //out.ang = -atan2(t.x() - r.x(), t.y() - r.y()); //target ang in the direction or line joining robot-target
                    });
            };
    robot_polygon = scene.initialize(graphicsView, target_slot, robot_polygon, ROBOT_WIDTH, ROBOT_LONG, FILE_NAME);
    robot = std::make_shared<Robot>(innerModel);

	this->Period = period;
	if(this->startup_check_flag)
	    this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    read_base();
    read_rgbd_camera();
    // compute_coastal_map();
    check_target(robot);
}

///////////////////////////////////////////////////////////////////////////////////////
float SpecificWorker::sigmoid(float t)
{
    return 2.f / (1.f + exp(-t * 1.4)) - 1.f;
}
float SpecificWorker::exponential(float value, float xValue, float yValue, float min)
{
    if (yValue <= 0)
        return 1.f;
    float landa = -fabs(xValue) / log(yValue);
    float res = exp(-fabs(value) / landa);
    return std::max(res, min);
}
RoboCompGenericBase::TBaseState SpecificWorker::read_base()
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        // bState.alpha is corrected to comply with InnerModel convention
        innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, -bState.alpha, 0);
        robot->update_state(Robot::State{bState.x, bState.z, bState.alpha});
        robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
        robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from DifferentialRobot" << e.what() << std::endl; }
    return bState;
}
void SpecificWorker::read_rgbd_camera()
{
    auto cdata = camerargbdsimple_proxy->getAll("pioneer_head_camera_sensor");
    cv::Mat img(cdata.image.height, cdata.image.width, CV_8UC3, const_cast<std::vector<uint8_t> &>(cdata.image.image).data());
    cv::imshow("rgb", img);
    const std::vector<uint8_t> &tmp = cdata.depth.depth;
    float *depth_array = (float *)cdata.depth.depth.data();
    const auto STEP = sizeof(float);
    std::vector<std::uint8_t> gray_image(tmp.size() / STEP);
    for (std::size_t i = 0; i < tmp.size() / STEP; i++)
        gray_image[i] = (int) (depth_array[i] * 15);  // ONLY VALID FOR SHORT RANGE, INDOOR SCENES
    cv::Mat depth(cdata.depth.height, cdata.depth.width, CV_8UC1, const_cast<std::vector<uint8_t> &>(gray_image).data());
    cv::imshow("depth", depth);
    //cv::waitKey(0);

}
void SpecificWorker::draw_target(QGraphicsScene *scene, std::shared_ptr<Robot> robot, const Target &target)
{
    static QGraphicsEllipseItem *target_draw = nullptr;

    if (target_draw) scene->removeItem(target_draw);
    target_draw = scene->addEllipse(target.pos.x() - 50, target.pos.y() - 50, 100, 100, QPen(QColor("green")), QBrush(QColor("green")));
    // angular reference obtained from line joinning robot an target when  clicking
    float tr_x = target.pos.x() - robot->state.x;
    float tr_y = target.pos.y() - robot->state.y;
    float ref_ang = -atan2(tr_x, tr_y);   // signo menos para tener ángulos respecto a Y CCW
    auto ex = target.pos.x() + 350 * sin(-ref_ang);
    auto ey = target.pos.y() + 350 * cos(-ref_ang);  //OJO signos porque el ang está respecto a Y CCW
    auto line = scene->addLine(target.pos.x(), target.pos.y(), ex, ey, QPen(QBrush(QColor("green")), 20));
    line->setParentItem(target_draw);
    auto ball = scene->addEllipse(ex - 25, ey - 25, 50, 50, QPen(QColor("green")), QBrush(QColor("green")));
    ball->setParentItem(target_draw);
}
void SpecificWorker::check_target(std::shared_ptr<Robot> robot)
{
    static Target target;
    if(auto t = target_buffer.try_get(); t.has_value())
    {
        target.set_new_value(t.value());
        draw_target(&scene, robot, target);
    }
    if(target.is_active())
    {
        try
        {
            if (not robot->at_target(target))
            {
                auto &&[dist_to_go, ang_to_go] = robot->to_go(target);
                float rot_speed = std::clamp(sigmoid(ang_to_go), -robot->MAX_ROT_SPEED, robot->MAX_ROT_SPEED);
                float adv_speed = std::min(robot->MAX_ADV_SPEED * exponential(rot_speed, 0.3, 0.1, 0), dist_to_go);
                adv_speed = std::clamp(adv_speed, 0.f, robot->MAX_ADV_SPEED);
                differentialrobot_proxy->setSpeedBase(adv_speed, rot_speed);
            } else
            {
                target.set_active(false);
                differentialrobot_proxy->setSpeedBase(0, 0);
            }
        }
        catch (const Ice::Exception &e)
        { std::cout << e.what() << std::endl; };
    }
}
//////////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

/**************************************/
// From the RoboCompCameraRGBDSimple you can call this methods:
// this->camerargbdsimple_proxy->getAll(...)
// this->camerargbdsimple_proxy->getDepth(...)
// this->camerargbdsimple_proxy->getImage(...)

/**************************************/
// From the RoboCompCameraRGBDSimple you can use this types:
// RoboCompCameraRGBDSimple::TImage
// RoboCompCameraRGBDSimple::TDepth
// RoboCompCameraRGBDSimple::TRGBD

/**************************************/
// From the RoboCompDifferentialRobot you can call this methods:
// this->differentialrobot_proxy->correctOdometer(...)
// this->differentialrobot_proxy->getBasePose(...)
// this->differentialrobot_proxy->getBaseState(...)
// this->differentialrobot_proxy->resetOdometer(...)
// this->differentialrobot_proxy->setOdometer(...)
// this->differentialrobot_proxy->setOdometerPose(...)
// this->differentialrobot_proxy->setSpeedBase(...)
// this->differentialrobot_proxy->stopBase(...)

/**************************************/
// From the RoboCompDifferentialRobot you can use this types:
// RoboCompDifferentialRobot::TMechParams

