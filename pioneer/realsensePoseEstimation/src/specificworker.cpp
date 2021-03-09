/*
 *    Copyright (C)2019 by YOUR NAME HERE
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
	initialPose.set(0.f, 0.f, 0.f, 0.f, 0.f, 0.f);
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
	serial = params["serial"].value;
	print_output = (params["print"].value == "true") or (params["print"].value == "True");
	return true;
}

//workaround => using serial value not working on actual api version
rs2::device SpecificWorker::get_device(const std::string& serial_number) {
    rs2::context ctx;
    while (true)
    {
        for (auto&& dev : ctx.query_devices())
            if (std::string(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == serial_number)
                return dev;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void SpecificWorker::initialize(int period)
{
	std::cout << "Initialize worker" << std::endl;
	fullpose.source = "realsense";
	// Add pose stream
	try{
		cfg.enable_device(serial);
		cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		// Start pipeline with chosen configuration
		pipe.start(cfg);
	}catch(...)
	{
		qFatal("Unable to open device, please check config file");
	}
	this->Period = 50;
	std::cout << "Period: " << this->Period << std::endl;
	timer.start(Period);
}

void SpecificWorker::compute()
{
	auto frames = pipe.wait_for_frames();
    // Get a frame from the pose stream
    auto f = frames.first_or_default(RS2_STREAM_POSE);
	// Cast the frame to pose_frame and get its data
	auto pose_data = f.as<rs2::pose_frame>().get_pose_data();
	// Print the x, y, z values of the translation, relative to initial position
	if(print_output)
	          std::cout << "\r" << "Device Position: " << std::setprecision(3)
			  << std::fixed << pose_data.translation.x << " " 
			  << pose_data.translation.y << " " 
			  << pose_data.translation.z << " (meters)" << " " 
			  << pose_data.rotation.x << " "
			  << pose_data.rotation.y << " "
			  << pose_data.rotation.z << " "
			  << pose_data.rotation.w << " (quat)";

//	const auto &tr = pose_data.translation;
//	const auto &rot = pose_data.rotation;
//	RMat::Quaternion q(rot.x,rot.y,rot.z,rot.w);
//	QVec angles = q.toAngles();
//
//	RTMat cam(angles.x(), -angles.y(), angles.z(),tr.x*1000, tr.y*1000, -tr.z*1000);
//	//std::cout << "X "<<tr.x<<std::endl;
//	RTMat pose = initialPose * cam;
//	QVec angles2 = pose.extractAnglesR();

    const double &qx = pose_data.rotation.x;
    const double &qy = pose_data.rotation.y;
    const double &qz = pose_data.rotation.z;
    const double &qw = pose_data.rotation.w;

    Eigen::Vector3d res;
    res[1] = atan2(2*qy*qw-2*qx*qz , 1 - 2*qy*qy - 2*qz*qz);
    res[2] = asin(2*qx*qy + 2*qz*qw);
    res[0] = atan2(2*qx*qw-2*qy*qz , 1 - 2*qx*qx - 2*qz*qz);

    if(qFuzzyCompare((qx*qy + qz*qw), 0.5)) // north pole
    {
        res[1] = 2. * atan2(qx,qw);
        res[0] = 0.;
    }
    if(qFuzzyCompare((qx*qy + qz*qw), -0.5)) //south pole
    {
        res[1] = -2. * atan2(qx,qw);
        res[0] = 0.;
    }
	std::lock_guard<std::mutex> lock(bufferMutex);
	fullpose.x = pose_data.translation.x;
	fullpose.y = pose_data.translation.y;
	fullpose.z = pose_data.translation.z;
	fullpose.rx = res[0];
	fullpose.ry = res[1];
	fullpose.rz = res[2];

	//publish
	try
	{ fullposeestimationpub_pubproxy->newFullPose(fullpose); }
	catch(const Ice::Exception& ex)
	{ std::cout << "Exception publishing pose: "<<ex << std::endl;	}
}

RoboCompFullPoseEstimation::FullPose SpecificWorker::FullPoseEstimation_getFullPose()
{
	std::lock_guard<std::mutex> lock(bufferMutex);
	return fullpose;
}

void SpecificWorker::FullPoseEstimation_setInitialPose(float x, float y, float z, float rx, float ry, float rz)
{
	std::cout << "New initial pose received: " <<x<<" "<<y<<" "<<z<<" "<<rx<<" "<<ry<<" "<<rz<<std::endl;
	initialPose.set(rx, ry, rz, x, y, z);
}
