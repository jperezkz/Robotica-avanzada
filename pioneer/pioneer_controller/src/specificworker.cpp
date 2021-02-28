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
#include <cppitertools/enumerate.hpp>
#include <cppitertools/sliding_window.hpp>
//#include <ranges>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>

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
    QFileInfo info1("../../etc/informatica.simscene.xml");
    QFileInfo info2("../../etc/informatica.json");
    //qInfo() << info2.lastModified().toSecsSinceEpoch() - info1.lastModified().toSecsSinceEpoch();

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
    scene.initialize(graphicsView, target_slot, ROBOT_WIDTH, ROBOT_LONG, FILE_NAME);
    robot = std::make_shared<Robot>(innerModel);

    // UI
    connect(&timer_alive, &QTimer::timeout, [this]()
            {
                try
                {
                    differentialrobot_proxy->ice_ping();
                    QPixmap mypix("../../etc/resources/green.png");
                    label_robot->setPixmap(mypix);
                }
                catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
                try
                {
                    fullposeestimation_proxy->ice_ping();
                    QPixmap mypix("../../etc/resources/green.png");
                    label_localization->setPixmap(mypix);
                }
                catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
                try
                {
                    camerargbdsimple_proxy->ice_ping();
                    QPixmap mypix("../../etc/resources/green.png");
                    label_camera->setPixmap(mypix);
                }
                catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;}
            });
    timer_alive.start(1000);

    // grid
    auto dim = scene.get_dimensions();
    grid.initialize(&scene, Grid<>::Dimensions{dim.TILE_SIZE, dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT });
    grid.fill_with_obstacles(scene. get_obstacles());

    // Viz
    //Displaying the Coordinate Origin (0,0,0)
    window.showWidget("coordinate", cv::viz::WCoordinateSystem(100));

    this->Period = period;
	if(this->startup_check_flag)
	    this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{

    read_base(&scene);
    read_robot_pose();

    // camara
    auto cdata = read_rgb_camera(true);

    // battery
    read_battery();

    // RSSI
    read_RSSI();

    //auto laser_data = get_laser_from_rgbd(cdata, &scene, true, 1);
    check_target(robot);

}

///////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::read_battery()
{
    try
    {
        auto battery = batterystatus_proxy->getBatteryState();
        bat_lcdnumber->display(battery.percentage);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}
void SpecificWorker::read_RSSI()
{
    try
    {
        auto rssi = rssistatus_proxy->getRSSIState();
        rssi_lcdnumber->display(rssi.percentage);
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}
void SpecificWorker::read_robot_pose()
{
    try
    {
        auto pose = fullposeestimation_proxy->getFullPose();
        //Info() << pose.x << pose.y << pose.z << pose.rx << pose.ry << pose.rz;
    }
    catch(const Ice::Exception &e){ std::cout << e.what() << std::endl;};
}
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
RoboCompGenericBase::TBaseState SpecificWorker::read_base(Robot2DScene *scene)
{
    RoboCompGenericBase::TBaseState bState;
    try
    {
        differentialrobot_proxy->getBaseState(bState);
        // bState.alpha is corrected to comply with InnerModel convention
        innerModel->updateTransformValues("robot", bState.x, 0, bState.z, 0, -bState.alpha, 0);
        robot->update_state(Robot::State{bState.x, bState.z, bState.alpha});
        scene->robot_polygon->setRotation(qRadiansToDegrees(bState.alpha));
        scene->robot_polygon->setPos(bState.x, bState.z);
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from DifferentialRobot" << e.what() << std::endl; }
    return bState;
}
RoboCompCameraRGBDSimple::TRGBD SpecificWorker::read_rgbd_camera(bool draw)
{
    auto cdata = camerargbdsimple_proxy->getAll("pioneer_head_camera_sensor");
    if(draw)
    {
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata.image.image).data();
        cv::Mat img(cdata.image.height, cdata.image.width, CV_8UC3, rgb_img_data);
        cv::flip(img, img, 0);
        cv::cvtColor(img ,img, cv::COLOR_RGB2BGR);
        //cv::imshow("rgb", img);
        //const std::vector<uint8_t> &tmp = cdata.depth.depth;
        //float *depth_array = (float *) cdata.depth.depth.data();
        //const auto STEP = sizeof(float);
        /*std::vector<std::uint8_t> gray_image(tmp.size() / STEP);
        for (std::size_t i = 0; i < tmp.size() / STEP; i++)
            gray_image[i] = (int) (depth_array[i] * 15);  // ONLY VALID FOR SHORT RANGE, INDOOR SCENES
        cv::Mat depth(cdata.depth.height, cdata.depth.width, CV_8UC1, const_cast<std::vector<uint8_t> &>(gray_image).data());*/
        //cv::imshow("depth", depth);
        //cv::waitKey(1);
        auto pix = QPixmap::fromImage(QImage(rgb_img_data, cdata.image.width, cdata.image.height, QImage::Format_RGB888));
        label_rgb->setPixmap(pix);
    }
    return cdata;
}
RoboCompCameraRGBDSimple::TImage SpecificWorker::read_rgb_camera(bool draw)
{
    auto cdata = camerargbdsimple_proxy->getImage("pioneer_head_camera_sensor");
    if(draw)
    {
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata.image).data();
        cv::Mat img(cdata.height, cdata.width, CV_8UC3, rgb_img_data);
        cv::flip(img, img, 0);
        cv::cvtColor(img ,img, cv::COLOR_RGB2BGR);
        auto pix = QPixmap::fromImage(QImage(rgb_img_data, cdata.width, cdata.height, QImage::Format_RGB888));
        label_rgb->setPixmap(pix);
    }
    return cdata;
}

void SpecificWorker::draw_target(Robot2DScene *scene, std::shared_ptr<Robot> robot, const Target &target)
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
        qInfo() << __FUNCTION__ << t.value().pos;
        target.set_new_value(t.value());
        draw_target(&scene, robot, target);
        auto path = grid.computePath(QPointF(robot->state.x, robot->state.y), target.pos);
        grid.draw_path(&scene, path, robot->WIDTH/3 );
    }
    if(target.is_active())
    {
//        try
//        {
//            if (not robot->at_target(target))
//            {
//                auto &&[dist_to_go, ang_to_go] = robot->to_go(target);
//                float rot_speed = std::clamp(sigmoid(ang_to_go), -robot->MAX_ROT_SPEED, robot->MAX_ROT_SPEED);
//                float adv_speed = std::min(robot->MAX_ADV_SPEED * exponential(rot_speed, 0.3, 0.1, 0), dist_to_go);
//                adv_speed = std::clamp(adv_speed, 0.f, robot->MAX_ADV_SPEED);
//                differentialrobot_proxy->setSpeedBase(adv_speed, rot_speed);
//            } else
//            {
//                target.set_active(false);
//                differentialrobot_proxy->setSpeedBase(0, 0);
//            }
//        }
//        catch (const Ice::Exception &e)
//        { std::cout << e.what() << std::endl; };
    }
}

std::vector<SpecificWorker::LaserPoint>  SpecificWorker::get_laser_from_rgbd( const RoboCompCameraRGBDSimple::TRGBD &cdata, Robot2DScene *scene,bool draw,unsigned short subsampling )
{
    const int MAX_LASER_BINS = 200;
    /*if (subsampling == 0 or subsampling > 10)
    {
        qWarning("SpecificWorker::get_laser_from_rgbd: subsampling parameter < 1 or > than 10");
        return std::vector<LaserPoint>();
    }
    const std::vector<uint8_t> &tmp = cdata.depth.depth;
    float *depth_array = (float *) cdata.depth.depth.data();  // cast to float
    const int WIDTH = cdata.depth.width;
    const int HEIGHT = cdata.depth.height;
    int FOCAL = cdata.depth.focalx;
    FOCAL = (int) ((WIDTH / 2) / atan(0.52));  // para angulo de 60º
    int STEP = subsampling;
    float X, Y, Z;
    int cols, rows;
    std::size_t SIZE = tmp.size() / sizeof(float);*/
    /*
    //const float TOTAL_HOR_ANGLE = atan2(WIDTH / 2.f, FOCAL) * 2.f;
    const float TOTAL_HOR_ANGLE = 1.0472;  // para 60º
    using Point = std::tuple< float, float, float>;
    auto cmp = [](Point a, Point b) { return true; };
    std::vector<std::set<Point, decltype(cmp)>> hor_bins(MAX_LASER_BINS);
    for (std::size_t i = 0; i < SIZE; i += STEP)
    {
        cols = (i % WIDTH) - (WIDTH / 2);
        rows = (HEIGHT / 2) - (i / WIDTH);
        // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
        Y = depth_array[i] * 1000; // we transform measurements to millimeters
        X = cols * Y / FOCAL;
        Z = rows * Y / FOCAL;
        // accumulate in bins of equal horizontal angle from optical axis
        float hor_angle = atan2(cols, FOCAL);
        // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
        int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
        hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
        // result[i] = std::make_tuple(X, Y, Z);
    }*/
    std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
    /*uint i=0;
    for(auto &bin : hor_bins)
    {
        const auto &[X,Y,Z] = *bin.cbegin();
        laser_data[i] = LaserPoint{sqrt(X*X+Y*Y+Z*Z), (i-MAX_LASER_BINS/2.f)/(MAX_LASER_BINS/TOTAL_HOR_ANGLE)};
        i++;
    }
    auto laser_poly = filter_laser(laser_data);
    if(draw)
        draw_laser(scene, laser_poly);*/
    return laser_data;
}
void SpecificWorker::draw_laser(Robot2DScene *scene, QPolygonF &laser_poly)
{
    static QGraphicsItem *laser_polygon = nullptr;
    if (laser_polygon != nullptr)
        scene->removeItem(laser_polygon);

    QColor color("LightGreen");
    color.setAlpha(40);
    laser_polygon = scene->addPolygon(scene->robot_polygon->mapToScene(laser_poly), QPen(QColor("DarkGreen"), 30), QBrush(color));
    laser_polygon->setZValue(3);
}
QPolygonF SpecificWorker::filter_laser(const std::vector<SpecificWorker::LaserPoint> &ldata)
{
    static const float MAX_RDP_DEVIATION_mm  =  70;
    static const float MAX_SPIKING_ANGLE_rads = 0.2;
    QPolygonF laser_poly;
    try
    {
        // Simplify laser contour with Ramer-Douglas-Peucker
        std::vector<Point> plist(ldata.size()+1);
        plist[0]=std::make_pair(0,0);
        std::generate(plist.begin()+1, plist.end(), [ldata, k=0]() mutable
            { auto &l = ldata[k++]; return std::make_pair(l.dist * sin(l.angle), l.dist * cos(l.angle));});
        std::vector<Point> pointListOut;
        ramer_douglas_peucker(plist, MAX_RDP_DEVIATION_mm, pointListOut);
        laser_poly.resize(pointListOut.size());
        std::generate(laser_poly.begin(), laser_poly.end(), [pointListOut, this, k=0]() mutable
            { auto &p = pointListOut[k++]; return QPointF(p.first, p.second);});
        laser_poly << QPointF(0,0);

        // Filter out spikes. If the angle between two line segments is less than to the specified maximum angle
        std::vector<QPointF> removed;
        for(auto &&[k, ps] : iter::sliding_window(laser_poly,3) | iter::enumerate)
            if( MAX_SPIKING_ANGLE_rads > acos(QVector2D::dotProduct( QVector2D(ps[0] - ps[1]).normalized(), QVector2D(ps[2] - ps[1]).normalized())))
                removed.push_back(ps[1]);
        for(auto &&r : removed)
            laser_poly.erase(std::remove_if(laser_poly.begin(), laser_poly.end(), [r](auto &p) { return p == r; }), laser_poly.end());
    }
    catch(const Ice::Exception &e)
    { std::cout << "Error reading from Laser" << e << std::endl;}
    laser_poly.pop_back();
    return laser_poly;  // robot coordinates
}
void SpecificWorker::ramer_douglas_peucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out)
{
    if(pointList.size()<2)
    {
        qWarning() << "Not enough points to simplify";
        return;
    }
    // Find the point with the maximum distance from line between start and end
    auto line = Eigen::ParametrizedLine<float, 2>::Through(Eigen::Vector2f(pointList.front().first, pointList.front().second),
                                                           Eigen::Vector2f(pointList.back().first, pointList.back().second));
    auto max = std::max_element(pointList.begin()+1, pointList.end(), [line](auto &a, auto &b)
            { return line.distance(Eigen::Vector2f(a.first, a.second)) < line.distance(Eigen::Vector2f(b.first, b.second));});
    float dmax =  line.distance(Eigen::Vector2f((*max).first, (*max).second));

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
        // Recursive call
        vector<Point> recResults1;
        vector<Point> recResults2;
        vector<Point> firstLine(pointList.begin(), max + 1);
        vector<Point> lastLine(max, pointList.end());

        ramer_douglas_peucker(firstLine, epsilon, recResults1);
        ramer_douglas_peucker(lastLine, epsilon, recResults2);

        // Build the result list
        out.assign(recResults1.begin(), recResults1.end() - 1);
        out.insert(out.end(), recResults2.begin(), recResults2.end());
        if (out.size() < 2)
        {
            qWarning() << "Problem assembling output";
            return;
        }
    }
    else
    {
        //Just return start and end points
        out.clear();
        out.push_back(pointList.front());
        out.push_back(pointList.back());
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
// From the RoboCompBatteryStatus you can call this methods:
// this->batterystatus_proxy->getBatteryState(...)

/**************************************/
// From the RoboCompBatteryStatus you can use this types:
// RoboCompBatteryStatus::TBattery

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

