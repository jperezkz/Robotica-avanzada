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

/**
	\brief
	@author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#include <genericworker.h>
#include <innermodel/innermodel.h>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsLineItem>
#include <myscene.h>
#include <doublebuffer/DoubleBuffer.h>
#include <grid2d/grid2d.h>
#include <grid2d/grid2d.cpp>  // due to templates populating grid2d.h
#include "elastic_band.h"
#include <chrono>
#include "opencv2/imgproc.hpp"
#include <opencv2/photo.hpp>

//#include <kindr/Core>
//#include <kindr/poses/HomogeneousTransformation.hpp>
//#include <kindr/poses/Pose.hpp>

class SpecificWorker : public GenericWorker
{
    using myclock = std::chrono::system_clock;
    using msec = std::chrono::duration<double, std::milli>;

    Q_OBJECT
    public:
        SpecificWorker(TuplePrx tprx, bool startup_check);
        ~SpecificWorker();
        bool setParams(RoboCompCommonBehavior::ParameterList params);


    public slots:
        void compute();
        int startup_check();
        void initialize(int period);

    protected:
        void resizeEvent(QResizeEvent * event)
        {
            graphicsView->fitInView(scene.sceneRect(), Qt::KeepAspectRatio);
        }
        void wheelEvent(QWheelEvent* event)
        {
            if(event->buttons() == Qt::RightButton )
            {
                qreal factor;
                if (event->angleDelta().y() > 0)
                    factor = 1.1;
                else
                    factor = 0.9;
                auto view_pos = event->pos();
                auto scene_pos = graphicsView->mapToScene(view_pos);
                graphicsView->centerOn(scene_pos);
                graphicsView->scale(factor, factor);
                auto delta = graphicsView->mapToScene(view_pos) - graphicsView->mapToScene(graphicsView->viewport()->rect().center());
                graphicsView->centerOn(scene_pos - delta);
            }
        }

    private:
        bool startup_check_flag;
        RoboCompCommonBehavior::ParameterList confParams;

        // Target
        struct Target
        {
            QPointF pos;
            float ang;
            void set_new_value(const Target &t) { pos = t.pos; ang = t.ang; active = true; };
            bool active = false;
            public:
                void set_active(bool a) { active = a;};
                bool is_active() const { return active;}
        };

        DoubleBuffer<QPointF, Target> target_buffer;

        //robot
        const float ROBOT_WIDTH = 400;
        const float ROBOT_LONG = 450;
        //const std::string FILE_NAME = "../../etc/escuela.json";
        const std::string FILE_NAME = "../../etc/informatica.json";

    struct Robot
        {
            Robot(Robot2DScene *scene_){ scene = scene_;}
            Robot2DScene *scene;
            const float WIDTH = 400;
            const float LENGTH = 450;
            float TARGET_THRESHOLD_DISTANCE = 100.f;
            float MAX_ROT_SPEED = 1.f; // rads/sg
            float MAX_ADV_SPEED = 1000.f;  // mm/sg
            struct State { float x; float y; float z;
                           float rx; float ry; float rz;
                           float vx; float vy; float vz;
                           float vrx; float vry; float vrz;
                           float adv; };
            State state;
            std::tuple<float, float> to_go(const Target &t) const
                {
                    //auto tr = innerModel->transform("robot", QVec::vec3(t.pos.x(), 0, t.pos.y()), "world");
                    auto tr = scene->robot_polygon->mapFromScene(t.pos);
                    float ang = atan2(tr.x(), tr.y());
                    float dist = QVector2D(tr).length();
                    return std::make_tuple(dist, -ang);
                };

        std::tuple<float, float> to_go(const QPointF p) const
        {
            //auto tr = innerModel->transform("robot", QVec::vec3(t.pos.x(), 0, t.pos.y()), "world");
            auto tr = scene->robot_polygon->mapFromScene(p);
            float ang = atan2(tr.x(), tr.y());
            float dist = QVector2D(tr).length();
            return std::make_tuple(dist, -ang);
        };
            bool at_target(Target &t)
            {
                return QVector2D(scene->robot_polygon->mapFromScene(t.pos)).length() < TARGET_THRESHOLD_DISTANCE;

            }
        bool at_target(QPointF p)
        {
            return QVector2D(scene->robot_polygon->mapFromScene(p)).length() < 400;
        }
            void update_state( const State &s)
            {
                state = s;
            }
        };
        std::shared_ptr<Robot> robot;
        RoboCompGenericBase::TBaseState read_base(Robot2DScene *scene);
        void read_robot_pose(Robot2DScene *scene);
        float sigmoid(float t);
        float exponential(float value, float xValue, float yValue, float min);
        void check_target( std::shared_ptr<Robot> robot);
        struct LaserPoint{ float dist; float angle;};
        std::vector<LaserPoint>  get_laser_from_rgbd( const RoboCompCameraRGBDSimple::TRGBD &cdata,
                                                      Robot2DScene *scene,
                                                      bool draw,
                                                      unsigned short subsampling);
        using Point = std::pair<float, float>;
        QPolygonF filter_laser(const std::vector<SpecificWorker::LaserPoint> &ldata);
        void ramer_douglas_peucker(const std::vector<Point> &pointList, double epsilon, std::vector<Point> &out);

        // 2d scene
        Robot2DScene scene;
        void draw_target(Robot2DScene *scene, std::shared_ptr<Robot> robot, const Target &target);
        void draw_laser(Robot2DScene *scene, QPolygonF &laser_poly); // robot coordinates

        // cameras
        std::tuple<RoboCompCameraRGBDSimple::TRGBD, RoboCompCameraRGBDSimple::TRGBD> read_rgbd_camera(bool draw);
        RoboCompCameraRGBDSimple::TImage read_rgb_camera(bool draw);
        cv::Mat mosaic( const RoboCompCameraRGBDSimple::TRGBD &cdata_left,
                        const RoboCompCameraRGBDSimple::TRGBD &cdata_right,
                        unsigned short subsampling);

        // battery
        void read_battery();

        // RSSI
        void read_RSSI();

        //Ultrasound
        struct sonarData{
            double x, y;
            int s;
        };

        void read_sonar();
        std::vector<sonarData> datosSonar;

        // Alive roobot
        QTimer timer_alive;

        // Grid
        Grid<> grid;
        list<QPointF> path;

        // Elastic band
        ElasticBand elastic_band;

        template <typename T>
        bool is_in_bounds(const T& value, const T& low, const T& high) { return !(value < low) && (value < high); }
        cv::Mat project_robot_on_image(std::shared_ptr<Robot> robot, cv::Mat virtual_frame, float focal);
        cv::Mat project_point_on_image(std::shared_ptr<Robot> robot, cv::Mat virtual_frame, float focal);
};

#endif
