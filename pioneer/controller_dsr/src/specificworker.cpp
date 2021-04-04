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
        custom_widget.show();

        // get camera_api
        if(auto cam_node = G->get_node(pioneer_camera_virtual_name); cam_node.has_value())
            cam_api = G->get_camera_api(cam_node.value());
        else
        {
            std::cout << "Controller-DSR terminate: could not find a camera node named " << pioneer_camera_virtual_name << std::endl;
            std::terminate();
        }

        //Inner Api
        inner_eigen = G->get_inner_eigen_api();

        this->Period = period;
		timer.start(Period);
	}
}

void SpecificWorker::compute()
{
    if(auto vframe_t = virtual_camera_buffer.try_get(); vframe_t.has_value())
    {
        auto vframe = vframe_t.value();
        qInfo() << vframe.cols << vframe.rows << vframe.depth();
        auto pix = QPixmap::fromImage(QImage(vframe.data, vframe.cols, vframe.rows, QImage::Format_RGB888));
        custom_widget.label_rgb->setPixmap(pix);
    }

}

/////////////////////////////////////////////////////////////////////////////////////////////
/// Asynchronous changes on G nodes from G signals
////////////////////////////////////////////////////////////////////////////////////////////
void SpecificWorker::add_or_assign_node_slot(const std::uint64_t id, const std::string &type)
{
    if(type == rgbd_type and id == cam_api->get_id())
        if(auto cam_node = G->get_node(id); cam_node.has_value())
            if (const auto g_image = G->get_attrib_by_name<cam_rgb_att>(cam_node.value()); g_image.has_value())
            {
                virtual_camera_buffer.put(std::move(g_image.value().get()),
                           [this](const std::vector<std::uint8_t> &in, cv::Mat &out)
                           {
                               out = cv::Mat(cam_api->get_height(), cam_api->get_width(), CV_8UC3, const_cast<std::vector<uint8_t> &>(in).data());
                           });
            }

//    if (type == omnirobot_type)
//    {
//        float X,Y,A;
//        if(auto robot_node = G->get_node(id); robot_node.has_value())
//            if(auto tx = G->get_attrib_by_name<base_target_x_att>(robot_node.value()); tx.has_value())
//                X = tx.value();
//        if(auto robot_node = G->get_node(id); robot_node.has_value())
//            if(auto ty = G->get_attrib_by_name<base_target_x_att>(robot_node.value()); ty.has_value())
//                Y = ty.value();
//        if(auto robot_node = G->get_node(id); robot_node.has_value())
//            if(auto angle = G->get_attrib_by_name<base_target_angle_att>(robot_node.value()); angle.has_value())
//                A = angle.value();
//    }
}


//////////////////////////////////////////////////////////////////////////////////

int SpecificWorker::startup_check()
{
	std::cout << "Startup check" << std::endl;
	QTimer::singleShot(200, qApp, SLOT(quit()));
	return 0;
}

// connect to signal and show virtual image



