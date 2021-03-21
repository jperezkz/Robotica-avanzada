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
#include <opencv2/imgcodecs.hpp>


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
//    try
//    {
//    }
//    catch(const std::exception &e) { qFatal("Error reading config params"); }
//    confParams = params;
	return true;
}

void SpecificWorker::initialize(int period)
{
    std::cout << "Initialize worker" << std::endl;

    // draw
    //QFileInfo info1("../../etc/informatica.simscene.xml");
    //QFileInfo info2("../../etc/informatica.json");
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
    robot = std::make_shared<Robot>(&scene);

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
    timer_alive.start(2000);

    // grid and planner
    auto dim = scene.get_dimensions();
    //grid.initialize(&scene, Grid<>::Dimensions{dim.TILE_SIZE, dim.HMIN, dim.VMIN, dim.WIDTH, dim.HEIGHT });
    //grid.fill_with_obstacles(scene. get_obstacles());

    // elastic band
    //elastic_band.initialize();

    // reset initial state
//    try
//    {
//        float x = 3305;
//        float y = 0;
//        float z = -21699;
//        float rx = 0;
//        float ry = 0;
//        float rz = 0;
//        fullposeestimation_proxy->setInitialPose(x, y, z, rx, ry, rz);
//    }
//    catch(const Ice::Exception &e){};


    this->Period = period;
	if(this->startup_check_flag)
	    this->startup_check();
	else
		timer.start(Period);
}

void SpecificWorker::compute()
{
    qInfo() << __FUNCTION__;
    read_robot_pose(&scene);
    // camara
    auto &&[cdata_left, cdata_right] = read_rgbd_camera(false);
    auto vframe = mosaic(cdata_left, cdata_right, 1);
    vframe = project_robot_on_image(robot, vframe, cdata_left.image.focalx);

    auto pix = QPixmap::fromImage(QImage(vframe.data, vframe.cols, vframe.rows, QImage::Format_RGB888));
    label_rgb->setPixmap(pix);

    // battery
    read_battery();
    // RSSI
    read_RSSI();
    //auto laser_data = get_laser_from_rgbd(cdata, &scene, true, 3);
    //check_target(robot);

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
void SpecificWorker::read_robot_pose(Robot2DScene *scene)
{
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    try
    {
        pose = fullposeestimation_proxy->getFullPoseEuler();  // en metros
    }
    catch(const Ice::Exception &e){ std::cout << e.what() <<  __FUNCTION__ << std::endl;};
    //qInfo() << __FUNCTION__ << pose.x << pose.y << pose.z << pose.rx << pose.ry << pose.rz << pose.vx << pose.vy << pose.vz << pose.vrx << pose.vry << pose.vrz;
    scene->robot_polygon->setRotation(qRadiansToDegrees(pose.rz));
    scene->robot_polygon->setPos(pose.x, pose.y);
    //qInfo() << "Robot in world" << scene->robot_polygon->mapToScene(scene->robot_polygon->pos()) << scene->robot_polygon->rotation();
    // projected robot
    int delta_time = 1;  // 1 sec
    // linear velocities are WRT world axes, so local speed has to be computed WRT to the robot's moving frame
    Eigen::Matrix<float, 2, 3> model;  // world velocity to robot velocity
    model << -sin(pose.rz), cos(pose.rz), 0.f,
             0.f,           0.f,          1.f;
    Eigen::Vector2f robot_velocity = model * Eigen::Vector3f(pose.vx, pose.vy, pose.vrz);
    //qInfo() << "robot speed " << robot_velocity.x() << robot_velocity.y();
    QPointF offset = scene->robot_polygon->mapToScene(0,robot_velocity[0] * delta_time );  //should be advance speed
    scene->robot_polygon_projected->setPos(offset);
    scene->robot_polygon_projected->setRotation(qRadiansToDegrees(pose.rz));
    robot->update_state(Robot::State{pose.x, pose.y, pose.z, pose.rx, pose.ry, pose.rz, pose.vx, pose.vy, pose.vz, pose.vrx, pose.vry, pose.vrz, robot_velocity[0]});
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
std::tuple<RoboCompCameraRGBDSimple::TRGBD, RoboCompCameraRGBDSimple::TRGBD> SpecificWorker::read_rgbd_camera(bool draw)
{
    RoboCompCameraRGBDSimple::TRGBD cdata_left, cdata_right;
    try
    {
        cdata_left = camerargbdsimple_proxy->getAll("pioneer_head_camera_0");
        cdata_right = camerargbdsimple_proxy->getAll("pioneer_head_camera_1");
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}
    auto &cdata = cdata_right;
    if(draw)
    {
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata.image.image).data();
        cv::Mat img(cdata.image.height, cdata.image.width, CV_8UC3, rgb_img_data);
        //cv::flip(img, img, -1);
        //cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        //cv::flip(img, img, 0);
        //cv::cvtColor(img ,img, cv::COLOR_RGB2BGR);
        //cv::imshow("rgb", img);
        //const std::vector<uint8_t> &tmp = cdata_left.depth.depth;
        //float *depth_array = (float *) cdata_left.depth.depth.data();
        //const auto STEP = sizeof(float);
        /*std::vector<std::uint8_t> gray_image(tmp.size() / STEP);
        for (std::size_t i = 0; i < tmp.size() / STEP; i++)
            gray_image[i] = (int) (depth_array[i] * 15);  // ONLY VALID FOR SHORT RANGE, INDOOR SCENES
        cv::Mat depth(cdata_left.depth.height, cdata_left.depth.width, CV_8UC1, const_cast<std::vector<uint8_t> &>(gray_image).data());*/
        //cv::imshow("depth", depth);
        //cv::waitKey(1);
        auto pix = QPixmap::fromImage(QImage(rgb_img_data, cdata.image.width, cdata.image.height, QImage::Format_RGB888));
        label_rgb->setPixmap(pix);
    }
    return std::make_tuple(cdata_left,cdata_right);
}
RoboCompCameraRGBDSimple::TImage SpecificWorker::read_rgb_camera(bool draw)
{
    auto cdata = camerargbdsimple_proxy->getImage("pioneer_head_camera_0");

    if(draw)
    {
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata.image).data();
        cv::Mat img(cdata.height, cdata.width, CV_8UC3, rgb_img_data);
        cv::flip(img, img, -1);
        cv::cvtColor(img, img, cv::COLOR_RGB2BGR);
        cv::Mat img_resized(640, 480, CV_8UC3);
        cv::resize(img, img_resized, cv::Size(640, 480));
        auto pix = QPixmap::fromImage(QImage(img_resized.data, img_resized.cols, img_resized.rows, QImage::Format_RGB888));
        label_rgb->setPixmap(pix);
    }
    else //coppelia
    {
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata.image).data();
        cv::Mat img(cdata.height, cdata.width, CV_8UC3, rgb_img_data);
        //cv::flip(img, img, 0);
        auto pix = QPixmap::fromImage(QImage(img.data, img.cols, img.rows, QImage::Format_RGB888));
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
        //auto path = grid.computePath(QPointF(robot->state.x, robot->state.y), target.pos);
        //grid.draw_path(&scene, path, robot->WIDTH/3 );
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
    const int MAX_LASER_BINS = 100;
    if (subsampling == 0 or subsampling > 10)
    {
        qWarning("SpecificWorker::get_laser_from_rgbd: subsampling parameter < 1 or > than 10");
        return std::vector<LaserPoint>();
    }
    const std::vector<uint8_t> &tmp = cdata.depth.depth;
    float *depth_array = (float *) cdata.depth.depth.data();  // cast to float
    const int WIDTH = cdata.depth.width;
    const int HEIGHT = cdata.depth.height;
    //int FOCAL_A = cdata.depth.focalx;  //554
    int FOCAL = (int) ((WIDTH / 2) / atan(0.52));  // para angulo de 60º  667
    int STEP = subsampling;
    float X, Y, Z;
    int cols, rows;
    std::size_t SIZE = tmp.size() / sizeof(float);

    //const float TOTAL_HOR_ANGLE = atan2(WIDTH / 2.f, FOCAL) * 2.f;
    const float TOTAL_HOR_ANGLE = 1.0472;  // para 60º
    using Point = std::tuple< float, float, float>;
    auto cmp = [](Point a, Point b) { auto &[ax,ay,az] = a; auto &[bx,by,bz] = b; return (ax*ax+ay*ay+az*az) < (bx*bx+by*by+bz*bz);};
    std::vector<std::set<Point, decltype(cmp)>> hor_bins(MAX_LASER_BINS);
    for (std::size_t i = 0; i < SIZE; i += STEP)
    {
        cols = (i % WIDTH) - (WIDTH / 2);
        rows = (HEIGHT / 2) - (i / WIDTH);
        // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
        Y = depth_array[i] * 1000; // we transform measurements to millimeters
        X = cols * Y / FOCAL;
        Z = rows * Y / FOCAL;
        if(Z>50)
            continue;
        // accumulate in bins of equal horizontal angle from optical axis
        float hor_angle = atan2(cols, FOCAL);
        // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
        int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
        hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
        // result[i] = std::make_tuple(X, Y, Z);
    }
    std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
    uint i=0;
    for(auto &bin : hor_bins)
    {
        if( bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            laser_data[i] = LaserPoint{sqrt(X * X + Y * Y + Z * Z), (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        }
        else
            laser_data[i] = LaserPoint{0.f,(i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        i++;
    }
    auto laser_poly = filter_laser(laser_data);
    if(draw)
        draw_laser(scene, laser_poly);
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
cv::Mat SpecificWorker::mosaic( const RoboCompCameraRGBDSimple::TRGBD &cdata_left,
                                const RoboCompCameraRGBDSimple::TRGBD &cdata_right,
                                unsigned short subsampling )
{
    // check that both cameras are equal
    // declare frame virtual
    cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(cdata_left.image.width*3, cdata_left.image.height), CV_8UC3);
    float center_virtual_i = frame_virtual.cols / 2.0;
    float center_virtual_j = frame_virtual.rows / 2.0;
    float frame_virtual_focalx = cdata_left.image.focalx;
    auto before = myclock::now();

    //project_image(cdata_left, frame_virtual);
    //project_image(cdata_right, frame_virtual);
    // filter
    // resize

    // laser stuff
    const int MAX_LASER_BINS = 100;
    const float TOTAL_HOR_ANGLE = 2.094;  // para 120º
    using Point = std::tuple< float, float, float>;
    auto cmp = [](Point a, Point b) { auto &[ax,ay,az] = a; auto &[bx,by,bz] = b; return (ax*ax+ay*ay+az*az) < (bx*bx+by*by+bz*bz);};
    std::vector<std::set<Point, decltype(cmp)>> hor_bins(MAX_LASER_BINS);

    // Left image check that rgb and depth are equal
    if(cdata_left.image.width == cdata_left.depth.width and cdata_left.image.height == cdata_left.depth.height)
    {
        // cast depth
        float *depth_array = (float *) cdata_left.depth.depth.data();
        // cast rgb
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata_left.image.image).data();
        // vars
        float X, Y, Z;
        int cols, rows;
        std::size_t num_pixels = cdata_left.depth.depth.size() / sizeof(float);
        float coseno = cos(-M_PI / 6.0);
        float seno = sin(-M_PI / 6.0);
        float h_offset = -100;
        for (std::size_t i = 0; i < num_pixels; i += subsampling)
        {
            cols = (i % cdata_left.depth.width) - (cdata_left.depth.width / 2);
            rows = (cdata_left.depth.height / 2) - (i / cdata_left.depth.height);
            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
            Y = depth_array[i] * 1000.f; // we transform measurements to millimeters
            if (Y < 100) continue;
            X = -cols * Y / cdata_left.depth.focalx;
            Z = rows * Y / cdata_left.depth.focalx;
            // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
            float XV = coseno * X - seno * Y + h_offset;
            float YV = seno * X + coseno * Y;
            // project on virtual camera
            auto col_virtual = frame_virtual_focalx * XV / YV + center_virtual_i;
            auto row_virtual = frame_virtual_focalx * Z / YV + center_virtual_j;
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            // laser computation
            if(Z<-100 or Z>100) continue;
            // accumulate in bins of equal horizontal angle from optical axis
            float hor_angle = atan2(cols, cdata_left.depth.focalx) - M_PI / 6.0 ;
            // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
            int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
            hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << " Depth and RGB sizes not equal";
        return cv::Mat();
    }
    // right image
    if(cdata_right.image.width == cdata_right.depth.width and cdata_right.image.height == cdata_right.depth.height)
    {
        // cast depth
        float *depth_array = (float *) cdata_right.depth.depth.data();
        // cast rgb
        const auto &rgb_img_data = const_cast<std::vector<uint8_t> &>(cdata_right.image.image).data();
        // vars
        float X, Y, Z;
        int cols, rows;
        std::size_t num_pixels = cdata_right.depth.depth.size() / sizeof(float);
        float coseno = cos(M_PI / 6.0);
        float seno = sin(M_PI / 6.0);
        float h_offset = 100;
        for (std::size_t i = 0; i < num_pixels; i += subsampling)
        {
            cols = (i % cdata_right.depth.width) - (cdata_right.depth.width / 2);
            rows = (cdata_right.depth.height / 2) - (i / cdata_right.depth.height);
            // compute axis coordinates according to the camera's coordinate system (Y outwards and Z up)
            Y = depth_array[i] * 1000.f; // we transform measurements to millimeters
            if (Y < 100) continue;
            X = -cols * Y / cdata_right.depth.focalx;
            Z = rows * Y / cdata_right.depth.focalx;
            // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
            float XV = coseno * X - seno * Y + h_offset;
            float YV = seno * X + coseno * Y;
            // project on virtual camera
            auto col_virtual = frame_virtual_focalx * XV / YV + center_virtual_i;
            auto row_virtual = frame_virtual_focalx * Z / YV + center_virtual_j;
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(floor(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(ceil(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(ceil(row_virtual), floor(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            if (is_in_bounds<float>(ceil(col_virtual), 0, frame_virtual.cols) and is_in_bounds<float>(floor(row_virtual), 0, frame_virtual.rows))
            {
                cv::Vec3b &color = frame_virtual.at<cv::Vec3b>(floor(row_virtual), ceil(col_virtual));
                color[0] = rgb_img_data[i * 3]; color[1] = rgb_img_data[i * 3 + 1]; color[2] = rgb_img_data[i * 3 + 2];
            }
            // laser computation
            if(Z<-100 or Z>100) continue;
            // accumulate in bins of equal horizontal angle from optical axis
            float hor_angle = atan2(cols, cdata_left.depth.focalx) + M_PI / 6.0;
            // map from +-MAX_ANGLE to 0-MAX_LASER_BINS
            int angle_index = (int)((MAX_LASER_BINS/TOTAL_HOR_ANGLE) * hor_angle + (MAX_LASER_BINS/2));
            hor_bins[angle_index].emplace(std::make_tuple(X,Y,Z));
        }
    }
    else
    {
        qWarning() << __FUNCTION__ << " Depth and RGB sizes not equal";
        return cv::Mat();
    }
    // Fill gaps
    //cv::inpaint(frame_virtual, frame_virtual_occupied, frame_virtual, 1.0, cv::INPAINT_TELEA);
    cv::medianBlur(frame_virtual, frame_virtual, 3);
    msec duration = myclock::now() - before;
    std::cout << "It took " << duration.count() << "ms" << std::endl;
    before = myclock::now();   // so it is remembered across QTimer calls to compute()
    //qInfo() << frame_virtual.step[0] * frame_virtual.rows;;
    //vector<int> compression_params;
    //compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    //compression_params.push_back(50);
    //vector<uchar> buffer;
    //cv::imencode(".jpg", frame_virtual, buffer, compression_params);
    //duration = myclock::now() - before;
    //std::cout << "Encode took " << duration.count() << "ms" << std::endl;
    //qInfo() << "comp " << buffer.size();

    cv::flip(frame_virtual, frame_virtual, -1);
    cv::Mat frame_virtual_final(label_rgb->width(),label_rgb->height(), CV_8UC3);
    cv::resize(frame_virtual, frame_virtual_final, cv::Size(label_rgb->width(),label_rgb->height()), 0, 0, cv::INTER_LANCZOS4);

    // laser computation
    std::vector<LaserPoint> laser_data(MAX_LASER_BINS);
    uint i=0;
    for(auto &bin : hor_bins)
    {
        if( bin.size() > 0)
        {
            const auto &[X, Y, Z] = *bin.cbegin();
            laser_data[i] = LaserPoint{sqrt(X * X + Y * Y + Z * Z), (i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        }
        else
            laser_data[i] = LaserPoint{0.f,(i - MAX_LASER_BINS / 2.f) / (MAX_LASER_BINS / TOTAL_HOR_ANGLE)};
        i++;
    }
    auto laser_poly = filter_laser(laser_data);
    draw_laser(&scene, laser_poly);

    return frame_virtual_final;
}
cv::Mat SpecificWorker::project_robot_on_image(std::shared_ptr<Robot> robot, cv::Mat virtual_frame, float focal)
{
    // only do if advance velocity is greater than 0
    //qInfo() << robot->state.adv;
    if(fabs(robot->state.adv) < 100) return virtual_frame;
    // get projected polygon in local coordinates
    QPolygonF poly = robot->scene->robot_polygon_projected->polygon();
    // transdorm to robot's coordinate frame
    QPolygonF poly_robot = robot->scene->robot_polygon->mapFromItem(robot->scene->robot_polygon_projected, poly);
    //qInfo() << poly_robot;
    float Z = -400.f;  // translation to virtual camera coordinate system  Y outwards
    float center_cols = virtual_frame.cols/2;
    float center_rows = virtual_frame.rows/2;
    // project on virtual camera
    std::vector<cv::Point> cv_poly;
    for(const auto &p : poly_robot)
    {
        auto col = focal * p.x() / p.y() + center_cols;
        auto row = focal * Z / p.y() + center_rows;
        //qInfo() << p.x() << p.y() << Z;
        if(virtual_frame.rows-row > virtual_frame.rows/2)
            cv_poly.emplace_back(cv::Point(col, virtual_frame.rows-row));
    }
    const cv::Point *pts = (const cv::Point*) cv::Mat(cv_poly).data;
    int npts = cv::Mat(cv_poly).rows;
    // draw the polygon
    cv::polylines(virtual_frame, &pts, &npts, 1, true, cv::Scalar(0, 255, 0), 12);
    return virtual_frame;
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

