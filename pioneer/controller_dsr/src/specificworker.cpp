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
#include <cppitertools/zip.hpp>

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(TuplePrx tprx, bool startup_check) : GenericWorker(tprx)
{
	this->startup_check_flag = startup_check;
    QLoggingCategory::setFilterRules("*.debug=false\n");
}

/**
* \brief Default destructor
*/
SpecificWorker::~SpecificWorker()
{
	std::cout << "Destroying SpecificWorker" << std::endl;
	G->write_to_json_file("./"+agent_name+".json");
	G.reset();
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	agent_name = params["agent_name"].value;
	agent_id = stoi(params["agent_id"].value);
	tree_view = params["tree_view"].value == "true";
	graph_view = params["graph_view"].value == "true";
	qscene_2d_view = params["2d_view"].value == "true";
	osg_3d_view = params["3d_view"].value == "true";

	return true;
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	this->Period = period;
	if(this->startup_check_flag)
	{
		this->startup_check();
	}
	else
	{
		timer.start(Period);
		// create graph
		G = std::make_shared<DSR::DSRGraph>(0, agent_name, agent_id, ""); // Init nodes
		std::cout<< __FUNCTION__ << "Graph loaded" << std::endl;  

		//dsr update signals
		connect(G.get(), &DSR::DSRGraph::update_node_signal, this, &SpecificWorker::add_or_assign_node_slot);
		connect(G.get(), &DSR::DSRGraph::update_edge_signal, this, &SpecificWorker::add_or_assign_edge_slot);
		connect(G.get(), &DSR::DSRGraph::update_attrs_signal, this, &SpecificWorker::add_or_assign_attrs_slot);
		connect(G.get(), &DSR::DSRGraph::del_edge_signal, this, &SpecificWorker::del_edge_slot);
		connect(G.get(), &DSR::DSRGraph::del_node_signal, this, &SpecificWorker::del_node_slot);

		// Graph viewer
		using opts = DSR::DSRViewer::view;
		int current_opts = 0;
		opts main = opts::none;
		if(tree_view)
		    current_opts = current_opts | opts::tree;
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		    current_opts = current_opts | opts::scene;
		if(osg_3d_view)
		    current_opts = current_opts | opts::osg;

		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
        setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

        // custom widget
        graph_viewer->add_custom_widget_to_dock("Pioneer Controller", &custom_widget);

        // 2D widget
        widget_2d = qobject_cast<DSR::QScene2dViewer *>(graph_viewer->get_widget(opts::scene));
        widget_2d->set_draw_laser(true);
        connect(widget_2d, SIGNAL(mouse_right_click(int, int, std::uint64_t)), this, SLOT(new_target_from_mouse(int, int, std::uint64_t)));

        // get camera_api
        if(auto cam_node = G->get_node(pioneer_camera_virtual_name); cam_node.has_value())
            cam_api = G->get_camera_api(cam_node.value());
        else
        {
            std::cout << "Controller-DSR terminate: could not find a camera node named " << pioneer_camera_virtual_name << std::endl;
            std::terminate();
        }

        // Inner Api
        inner_eigen = G->get_inner_eigen_api();

        // Robot polygon
        if(auto robot_body = G->get_node(robot_body_name); robot_body.has_value())
        {
            auto width = G->get_attrib_by_name<width_att>(robot_body.value());
            auto height = G->get_attrib_by_name<depth_att>(robot_body.value());
            if (width.has_value() and height.has_value())
            {
                robot_polygon << QPointF(-width.value() / 2, -height.value() / 2)
                              << QPointF(-width.value() / 2, height.value() / 2)
                              << QPointF(width.value() / 2, height.value() / 2)
                              << QPointF(width.value() / 2, -height.value() / 2);
            } else
            {
                std::cout << __FUNCTION__ << " No robot body width or depth found. Terminating..." << std::endl;
                std::terminate();
            }
        }
        else
        {
            std::cout << __FUNCTION__ << " No robot body found. Terminating..." << std::endl;
            std::terminate();
        }
        OctaveFormat = Eigen::IOFormat(Eigen::StreamPrecision, 0, ", ", ";\n", "", "", "[", "]");
        CommaInitFmt = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");
        this->Period = 100;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    auto robot_node = get_robot_node();
    if(auto vframe = read_camera(robot_node); not vframe.empty())
    {
        if (auto laser_o = laser_buffer.try_get(); laser_o.has_value() and not vframe.empty())
        {
            const auto &[angles, dists, laser_poly_local, laser_cart_world] = laser_o.value();
            project_laser_on_image(robot_node, laser_poly_local, vframe, cam_api->get_focal_x());
        }

        auto pix = QPixmap::fromImage(QImage(vframe.data, vframe.cols, vframe.rows, QImage::Format_RGB888));
        custom_widget.label_rgb->setPixmap(pix);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////
cv::Mat SpecificWorker::read_camera(const DSR::Node &robot_node)
{
    if (auto vframe_t = virtual_camera_buffer.try_get(); vframe_t.has_value())
    {
        auto vframe = vframe_t.value().clone();
        project_robot_on_image(robot_node, robot_polygon, vframe, cam_api->get_focal_x());
        return vframe;
    }
    else
        return cv::Mat();
}

DSR::Node SpecificWorker::get_robot_node()
{
    if (auto robot_o = G->get_node(robot_name); robot_o.has_value())
        return robot_o.value();
    else
    {
        std::cout << __FUNCTION__ << " Terminating due to missing robot_node: " << robot_name << std::endl;
        std::terminate();
    }
}

void SpecificWorker::project_robot_on_image(const DSR::Node &robot_node, const QPolygonF &robot_polygon, cv::Mat virtual_frame, float focal)
{
    if (auto local_velocity = G->get_attrib_by_name<robot_local_linear_velocity_att>(robot_node); local_velocity.has_value())
    {
        float robot_adv_speed = local_velocity.value().get()[1];   // Y component of linear speed in robot's coordinate frame
        int delta_time = 1;  // 1 sec
        if (fabs(robot_adv_speed) < 50) return;    // only do if advance velocity is greater than 0
        // displace robot polygon by offset
        QPolygonF robot_polygon_projected(robot_polygon);
        robot_polygon_projected.translate(0, robot_adv_speed * delta_time);
        // transform projected polygon to virtal camera coordinate frame and  project into virtual camera
        std::vector<cv::Point> cv_poly;
        for (const auto &p : robot_polygon_projected)
        {
            if(auto projected_point = inner_eigen->transform(pioneer_camera_virtual_name, Eigen::Vector3d(p.x(), p.y(), 0.f), robot_name); projected_point.has_value())
            {
                auto point = cam_api->project(projected_point.value());
                if( point.y() > virtual_frame.rows / 2)
                    cv_poly.emplace_back(cv::Point(point.x(), point.y()));
            }
        }
        // paint on image
        const cv::Point *pts = (const cv::Point *) cv::Mat(cv_poly).data;
        int npts = cv::Mat(cv_poly).rows;
        cv::polylines(virtual_frame, &pts, &npts, 1, true, cv::Scalar(0, 255, 0), 8);
    }
    else{ std::cout << __FUNCTION__ << " No robot_local_linear_velocity attribute in robot: " << robot_name << std::endl; }
}

void SpecificWorker::project_laser_on_image(const DSR::Node &robot_node, const QPolygonF &laser_poly_local, cv::Mat virtual_frame, float focal)
{
        std::vector<cv::Point> cv_poly;
        // transform laser polygon to virtual camera coordinate frame
        for(const auto &p : laser_poly_local)
        {
            if(auto projected_point = inner_eigen->transform(pioneer_camera_virtual_name, Eigen::Vector3d(p.x(), p.y(), 0.f), robot_name); projected_point.has_value())
            {
                auto point = cam_api->project(projected_point.value());
                if( point.x() < virtual_frame.cols and point.y() < virtual_frame.rows and point.x() >= 0 and point.y()>=0 )
                    cv_poly.emplace_back(cv::Point(point.x(), point.y()));
            }
        }
        // add two points to close de polygon
        cv_poly.emplace_back(cv::Point(virtual_frame.cols/2,virtual_frame.rows));
        cv_poly.insert(cv_poly.begin(), cv::Point(virtual_frame.cols/2, virtual_frame.rows));
        // paint on image
        const cv::Point *pts = (const cv::Point*) cv::Mat(cv_poly).data;
        int npts = cv::Mat(cv_poly).rows;
        cv::polylines(virtual_frame, &pts, &npts, 1, true, cv::Scalar(0, 5, 200), 4);
}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    if(type == rgbd_type and id == cam_api->get_id())
    {
//        if(auto rgb_vector = cam_api->get_rgb_image(); rgb_vector.has_value())
//        {
//            virtual_camera_buffer.put(std::move(rgb_vector.value()),
//                                      [this](const std::vector<std::uint8_t> &in, cv::Mat &out)
//                                      {
//                                          out = cv::Mat(cam_api->get_height(), cam_api->get_width(), CV_8UC3, const_cast<std::vector<uint8_t> &>(in).data());
//                                      });
//        }
        if (auto cam_node = G->get_node(id); cam_node.has_value())
            if (const auto g_image = G->get_attrib_by_name<cam_rgb_att>(cam_node.value()); g_image.has_value())
            {
                //virtual_camera_buffer.put(std::move(g_image.value().get()), [this](std::vector<std::uint8_t> &&in, cv::Mat &out)
                virtual_camera_buffer.put(std::move(g_image.value().get()), [this](const std::vector<std::uint8_t> &in, cv::Mat &out) {
                    out = cv::Mat(cam_api->get_height(), cam_api->get_width(), CV_8UC3, const_cast<std::vector<uint8_t> &>(in).data());
                });
            }
    }
    else if (type == laser_type)    // Laser node updated
    {
        //qInfo() << __FUNCTION__ << " laser node change";
        if( auto node = G->get_node(id); node.has_value())
        {
            auto angles = G->get_attrib_by_name<laser_angles_att>(node.value());
            auto dists = G->get_attrib_by_name<laser_dists_att>(node.value());
            if(dists.has_value() and angles.has_value())
            {
                if(dists.value().get().empty() or angles.value().get().empty()) return;
                laser_buffer.put(std::make_tuple(angles.value().get(), dists.value().get()),
                                 [this](const LaserData &in, std::tuple<std::vector<float>, std::vector<float>, QPolygonF,std::vector<QPointF>> &out) {
                                     QPolygonF laser_poly_local;
                                     //laser_poly_local << QPointF(0.f, 200.f);
                                     std::vector<QPointF> laser_cart_world;
                                     const auto &[angles, dists] = in;
                                     for (const auto &[angle, dist] : iter::zip(angles, dists))
                                     {
                                         //convert laser polar coordinates to cartesian in the robot's coordinate frame
                                         float x = dist * sin(angle);
                                         float y = dist * cos(angle);
                                         Mat::Vector3d laserWorld = inner_eigen->transform(world_name,Mat::Vector3d(x, y, 0), pioneer_laser_name).value();
                                         laser_poly_local << QPointF(x, y);
                                         laser_cart_world.emplace_back(QPointF(laserWorld.x(), laserWorld.y()));
                                     }
                                     //laser_poly_local << QPointF(0.f, 200.f);
                                     out = std::make_tuple(angles, dists, laser_poly_local, laser_cart_world);
                                 });
            }
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

// connect to signal and show virtual image


