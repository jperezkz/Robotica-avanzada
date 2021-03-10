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

    dsr_input_file = params["dsr_input_file"].value;

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

		//APIS
        rt = G->get_rt_api();
        inner_eigen = G->get_inner_eigen_api();

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
		{
		    current_opts = current_opts | opts::tree;
		}
		if(graph_view)
		{
		    current_opts = current_opts | opts::graph;
		    main = opts::graph;
		}
		if(qscene_2d_view)
		{
		    current_opts = current_opts | opts::scene;
		}
		if(osg_3d_view)
		{
		    current_opts = current_opts | opts::osg;
		}
		graph_viewer = std::make_unique<DSR::DSRViewer>(this, G, current_opts, main);
		setWindowTitle(QString::fromStdString(agent_name + "-") + QString::number(agent_id));

		this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    update_robot_localization();
    update_rgbd();
    //auto cdata = read_rgb_camera(false);
    //read_battery();
    //read_RSSI();
}

//////////////////////////////////////////////////////////////////////////////////////////

void SpecificWorker::update_rgbd()
{
    RoboCompCameraRGBDSimple::TImage rgb;
    try
    {
        rgb = camerargbdsimple_proxy->getImage("pioneer_head_camera_0");
    }
    catch (const Ice::Exception &e){ std::cout << e.what() << std::endl;}

    if( auto node = G->get_node(pioneer_head_camera_left_name); node.has_value())
    {
        G->add_or_modify_attrib_local<cam_rgb_att>(node.value(), rgb.image);
        G->add_or_modify_attrib_local<cam_rgb_width_att>(node.value(), rgb.width);
        G->add_or_modify_attrib_local<cam_rgb_height_att>(node.value(), rgb.height);
        G->add_or_modify_attrib_local<cam_rgb_depth_att>(node.value(), rgb.depth);
        G->add_or_modify_attrib_local<cam_rgb_cameraID_att>(node.value(), rgb.cameraID);
        G->add_or_modify_attrib_local<cam_rgb_focalx_att>(node.value(), rgb.focalx);
        G->add_or_modify_attrib_local<cam_rgb_focaly_att>(node.value(), rgb.focaly);
        G->add_or_modify_attrib_local<cam_rgb_alivetime_att>(node.value(), rgb.alivetime);
        // depth
//        G->add_or_modify_attrib_local<cam_depth_att>(node.value(), depth.depth);
//        G->add_or_modify_attrib_local<cam_depth_width_att>(node.value(), depth.width);
//        G->add_or_modify_attrib_local<cam_depth_height_att>(node.value(), depth.height);
//        G->add_or_modify_attrib_local<cam_depth_focalx_att>(node.value(), depth.focalx);
//        G->add_or_modify_attrib_local<cam_depth_focaly_att>(node.value(), depth.focaly);
//        G->add_or_modify_attrib_local<cam_depth_cameraID_att>(node.value(), depth.cameraID);
//        G->add_or_modify_attrib_local<cam_depthFactor_att>(node.value(), depth.depthFactor);
//        G->add_or_modify_attrib_local<cam_depth_alivetime_att>(node.value(), depth.alivetime);
        G->update_node(node.value());
    }
    else
        qWarning() << __FUNCTION__ << "Node not found";
}

void SpecificWorker::read_battery()
{
    try
    {
        auto battery = batterystatus_proxy->getBatteryState();
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}
void SpecificWorker::read_RSSI()
{
    try
    {
        auto rssi = rssistatus_proxy->getRSSIState();
    }
    catch(const Ice::Exception &e) { std::cout << e.what() << std::endl;}
}
void SpecificWorker::update_robot_localization()
{
    static RoboCompFullPoseEstimation::FullPoseEuler last_state;
    RoboCompFullPoseEstimation::FullPoseEuler pose;
    try
    {
        pose = fullposeestimation_proxy->getFullPoseEuler();
        qInfo() << pose.x << pose.y << pose.z << pose.rx << pose.ry << pose.rz;
    }
    catch(const Ice::Exception &e){ std::cout << e.what() <<  __FUNCTION__ << std::endl;};

    if( auto robot = G->get_node(robot_name); robot.has_value())
    {
        if( auto parent = G->get_parent_node(robot.value()); parent.has_value())
        {
            if (are_different(std::vector < float > {pose.x, pose.y, pose.rz},
                              std::vector < float > {last_state.x, last_state.y, last_state.rz},
                              std::vector < float > {1, 1, 0.05}))
            {
                auto edge = rt->get_edge_RT(parent.value(), robot->id()).value();
                G->modify_attrib_local<rt_rotation_euler_xyz_att>(edge, std::vector < float > {0.0, 0.0, pose.rz});
                G->modify_attrib_local<rt_translation_att>(edge, std::vector < float > {pose.x, pose.y, 0.0});
                //G->modify_attrib_local<robot_current_linear_speed_att>(edge, std::vector<float>{pose.vel_x, pose.vel_y, pose.vel_z});
                //G->modify_attrib_local<robot_current_angular_speed_att>(edge, std::vector<float>{pose.rot.x, pos.rot.y, pose.rot_z});
                G->insert_or_assign_edge(edge);
                last_state = pose;
            }
        }
        else  qWarning() << __FUNCTION__ << " No parent found for node " << QString::fromStdString(robot_name);
    }
    else    qWarning() << __FUNCTION__ << " No node " << QString::fromStdString(robot_name);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

//////////////////// AUX ///////////////////////////////////////////////
bool SpecificWorker::are_different(const std::vector<float> &a, const std::vector<float> &b, const std::vector<float> &epsilon)
{
    for(auto &&[aa, bb, e] : iter::zip(a, b, epsilon))
        if (fabs(aa - bb) > e)
            return true;
    return false;
};
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

/**************************************/
// From the RoboCompFullPoseEstimation you can call this methods:
// this->fullposeestimation_proxy->getFullPose(...)
// this->fullposeestimation_proxy->setInitialPose(...)

/**************************************/
// From the RoboCompFullPoseEstimation you can use this types:
// RoboCompFullPoseEstimation::FullPose

/**************************************/
// From the RoboCompRSSIStatus you can call this methods:
// this->rssistatus_proxy->getRSSIState(...)

/**************************************/
// From the RoboCompRSSIStatus you can use this types:
// RoboCompRSSIStatus::TRSSI

/**************************************/
// From the RoboCompUltrasound you can call this methods:
// this->ultrasound_proxy->getAllSensorDistances(...)
// this->ultrasound_proxy->getAllSensorParams(...)
// this->ultrasound_proxy->getBusParams(...)
// this->ultrasound_proxy->getSensorDistance(...)
// this->ultrasound_proxy->getSensorParams(...)

/**************************************/
// From the RoboCompUltrasound you can use this types:
// RoboCompUltrasound::BusParams
// RoboCompUltrasound::SensorParams

