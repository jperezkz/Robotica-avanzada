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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class SpecificWorker : public GenericWorker
{
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

    private:
        std::shared_ptr < InnerModel > innerModel;
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
        //const std::string FILE_NAME_XML = "../etc/escuela.simscene.json";
        const std::string FILE_NAME = "../etc/escuela.json";
        struct Robot
        {
            float TARGET_THRESHOLD_DISTANCE = 100.f;
            float MAX_ROT_SPEED = 1.f; // rads/sg
            float MAX_ADV_SPEED = 1000.f;  // mm/sg
            std::shared_ptr < InnerModel > innerModel;
            struct State { float x; float y; float ang;};
            State state;
            Robot(std::shared_ptr <InnerModel> innerModel_){ innerModel = innerModel_;}
            std::tuple<float, float> to_go(const Target &t) const
                {
                    auto tr = innerModel->transform("robot", QVec::vec3(t.pos.x(), 0, t.pos.y()), "world");
                    float ang = atan2(tr.x(), tr.z());
                    float dist = tr.norm2();
                    return std::make_tuple(dist, ang);
                };
            bool at_target(Target &t)
            {
                return innerModel->transform("robot", QVec::vec3(t.pos.x(), 0, t.pos.y()), "world").norm2() < TARGET_THRESHOLD_DISTANCE;
            }
            void update_state( const State &s)
            {
                state = s;
            }
        };
        std::shared_ptr<Robot> robot;
        RoboCompGenericBase::TBaseState read_base();
        float sigmoid(float t);
        float exponential(float value, float xValue, float yValue, float min);
        void check_target( std::shared_ptr<Robot> robot);

        // 2d scene
        MyScene scene;
        QGraphicsItem *robot_polygon = nullptr;
        void draw_target(QGraphicsScene *scene, std::shared_ptr<Robot> robot, const Target &target);

        // cameras
        void read_rgbd_camera();
};

#endif
