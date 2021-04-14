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
#include <librealsense2/rs.hpp>
//#include <librealsense2/rs_processing.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cppitertools/enumerate.hpp>
#include <opencv2/opencv.hpp>

//struct filter_options
//{
//public:
//    std::string filter_name;                                   //Friendly name of the filter
//    rs2::filter &filter;                                       //The filter in use
//    std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not
//
//    filter_options(const std::string name, rs2::filter &flt) :
//            filter_name(name),
//            filter(flt),
//            is_enabled(true)
//    {
//        const std::array<rs2_option, 5> possible_filter_options = {
//                RS2_OPTION_FILTER_MAGNITUDE,
//                RS2_OPTION_FILTER_SMOOTH_ALPHA,
//                RS2_OPTION_MIN_DISTANCE,
//                RS2_OPTION_MAX_DISTANCE,
//                RS2_OPTION_FILTER_SMOOTH_DELTA
//        };
//    }
//
//    filter_options(filter_options&& other) :
//            filter_name(std::move(other.filter_name)),
//            filter(other.filter),
//            is_enabled(other.is_enabled.load())
//    { }
//};
//

//class Filters : public QThread
//{
//    std::vector<rs2::pipeline>  pipes;
//    rs2::frame rgb_list[2];
//    rs2::frame depth_list[2];
//    rs2::pointcloud pointclouds[2];
//    rs2::points points[2];
//
//
//    std::vector<filter_options> filters;
//    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
//    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
//    rs2::temporal_filter temp_filter;
//    rs2::hole_filling_filter holef_filter;
//    rs2::disparity_transform depth_to_disparity;
//    rs2::disparity_transform disparity_to_depth;
//    rs2::frame_queue *mosaic_queue;
//
//public:
//    Filters(const std::vector<rs2::pipeline>  &pipelines, rs2::frame_queue *mosaic_queue_)
//    {
//        pipes=pipelines;
//        filters.emplace_back("Decimate", dec_filter);
//        filters.emplace_back("Spatial", spat_filter);
//        filters.emplace_back("Temporal", temp_filter);
//        filters.emplace_back("HFilling", holef_filter);
//        mosaic_queue = mosaic_queue_;
//    }
//    void run()
//    {
//        std::vector<rs2::frameset> frame_list(2);
//        rs2::frameset data;
//        while (true)
//        {
//            //process
//            for (auto &&[i, pipe] : iter::enumerate(pipes))
//            {
//                data = pipe.wait_for_frames();
//                frame_list[i] = data;
//                depth_list[i] = data.get_depth_frame(); // Find and colorize the depth data
//
//                bool revert_disparity = false;
//                for (auto &&filter : filters)
//                {
//                    if (filter.is_enabled) {
//                        depth_list[i] = filter.filter.process(depth_list[i]);
//                        if (filter.filter_name == "Disparity")
//                            revert_disparity = true;
//                    }
//                }
//                rgb_list[i] = data.get_color_frame(); // Find the color data
//                points[i] = pointclouds[i].calculate(depth_list[i]);
//                pointclouds[i].map_to(rgb_list[i]);
//                mosaic_queue->enqueue(frame_list[i]);
//                mosaic_queue->enqueue(points[i]);
//              //qInfo()<<frame_list[i].get_data_size();
//            }
//
//            //blocking write to buffer_mosaic
//        }
//    }
//};
//
//class Mosaic : public QThread
//{
//    rs2::points points[2];
//    rs2::frame_queue *mosaic_queue;
//    rs2_intrinsics left_cam_intr, right_cam_intr;
//    cv::Mat frame;
//    public:
//    Mosaic(rs2::frame_queue *mosaic_queue_, rs2_intrinsics left_cam_intr_, rs2_intrinsics right_cam_intr_)
//    {
//        mosaic_queue = mosaic_queue_;
//        left_cam_intr=left_cam_intr_;
//        right_cam_intr=right_cam_intr_;
//        frame.setTo(0);
//    };
//
//    cv::Mat getFrame(){
//        return frame;
//    }
//
//    void color(rs2::video_frame image, cv::Mat frame_v, int row_v, int col_v, int k, int l) {
//        auto ptr = (uint8_t *) image.get_data();
//        auto stride = image.get_stride_in_bytes();
//
//        cv::Vec3b &color = frame_v.at<cv::Vec3b>(floor(row_v), floor(col_v));
//        color[0] = int(ptr[k * stride + (3 * l)]);
//        color[1] = int(ptr[k * stride + (3 * l) + 1]);
//        color[2] = int(ptr[k * stride + (3 * l) + 2]);
//        color = frame_v.at<cv::Vec3b>(ceil(row_v), floor(col_v));
//        color[0] = int(ptr[k * stride + (3 * l)]);
//        color[1] = int(ptr[k * stride + (3 * l) + 1]);
//        color[2] = int(ptr[k * stride + (3 * l) + 2]);
//        color = frame_v.at<cv::Vec3b>(floor(row_v), ceil(col_v));
//        color[0] = int(ptr[k * stride + (3 * l)]);
//        color[1] = int(ptr[k * stride + (3 * l) + 1]);
//        color[2] = int(ptr[k * stride + (3 * l) + 2]);
//        color = frame_v.at<cv::Vec3b>(ceil(row_v), ceil(col_v));
//        color[0] = int(ptr[k * stride + (3 * l)]);
//        color[1] = int(ptr[k * stride + (3 * l) + 1]);
//        color[2] = int(ptr[k * stride + (3 * l) + 2]);
//    }
//
//    void run()
//    {
//        std::vector<rs2::frameset> frame_list(2);
//        while (true)
//        {
//            //block read from buffer_mosaic
//            qInfo() << "hola2";
//            frame_list[0] = mosaic_queue->wait_for_frame();
//            points[0] = mosaic_queue->wait_for_frame();
//            frame_list[1] = mosaic_queue->wait_for_frame();
//            points[1] = mosaic_queue->wait_for_frame();
//
//            qInfo()<<"0: "<<frame_list[0].get_data_size();
//            qInfo()<<"1: "<<frame_list[1].get_data_size();
//            //process
//            rs2::video_frame left_image = frame_list[0].get_color_frame();
//            rs2::video_frame right_image = frame_list[1].get_color_frame();
//            cv::Mat frame_virtual = cv::Mat::zeros(cv::Size(left_cam_intr.width * 2.5, left_cam_intr.height * 1.5),
//                                                   CV_8UC3);
//            float center_virtual_cols = frame_virtual.cols / 2.0;
//            float center_virtual_rows = frame_virtual.rows / 2.0;
//            float frame_virtual_lfocalx = left_cam_intr.fx;
//            float frame_virtual_rfocalx = right_cam_intr.fx;
//            {
//                auto left_ptr = (uint8_t *) left_image.get_data();
//                auto left_stride = left_image.get_stride_in_bytes();
//                float coseno = cos(-M_PI / 7.4);
//                float seno = sin(-M_PI / 7.4);
//                float h_offset = 0.1;
//                const rs2::vertex *vertices = points[0].get_vertices();
//                auto tex_coords = points[0].get_texture_coordinates(); // and texture coordinates, u v coor of rgb image
//                for (size_t i = 0; i < points[0].size(); i++) {
//                    if (vertices[i].z) {
//                        // Y downwards and Z outwards
//                        // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
//                        float XV = coseno * vertices[i].x - seno * vertices[i].z + h_offset;
//                        float ZV = seno * vertices[i].x + coseno * vertices[i].z;
//                        // project
//                        int col_virtual = static_cast<int>(fabs(frame_virtual_lfocalx * XV / ZV + center_virtual_cols));
//                        int row_virtual = static_cast<int>(fabs(
//                                frame_virtual_lfocalx * vertices[i].y / ZV + center_virtual_rows));
//
//                        if (col_virtual >= frame_virtual.cols or row_virtual >= frame_virtual.rows) continue;
//
//                        //col_virtual += center_virtual_j/2;
//                        //qInfo() << "coor " << vertices[i].x << vertices[i].y << vertices[i].z << col_virtual << row_virtual;
//
//                        int k = tex_coords[i].v * left_image.get_height();
//                        int l = tex_coords[i].u * left_image.get_width();
//
//                        if (k < 0 or k >= left_image.get_height() or l < 0 or l > left_image.get_width()) continue;
//
//                        color(left_image, frame_virtual, row_virtual, col_virtual, k, l);
//                    }
//                }
//            }
//            {
//                auto right_ptr = (uint8_t *) right_image.get_data();
//                auto right_stride = right_image.get_stride_in_bytes();
//                float coseno = cos(M_PI / 7.4);
//                float seno = sin(M_PI / 7.4);
//                float h_offset = -0.1;
//                const rs2::vertex *vertices = points[1].get_vertices();
//                auto tex_coords = points[1].get_texture_coordinates(); // and texture coordinates, u v coor of rgb image
//
//                for (size_t i = 0; i < points[1].size(); i++) {
//                    if (vertices[i].z) {
//                        // (Y outwards and Z up)
//                        // transform to virtual camera CS at center of both cameras. Assume equal height (Z). Needs angle and translation
//                        float XV = coseno * vertices[i].x - seno * vertices[i].z + h_offset;
//                        float ZV = seno * vertices[i].x + coseno * vertices[i].z;
//                        // project
//                        float col_virtual = fabs(frame_virtual_rfocalx * XV / ZV + center_virtual_cols);
//                        float row_virtual = fabs(frame_virtual_rfocalx * vertices[i].y / ZV + center_virtual_rows);
//                        //qInfo() << "coor " << vertices[i].x << vertices[i].y << vertices[i].z << col_virtual << row_virtual;
//                        if (col_virtual >= frame_virtual.cols or row_virtual >= frame_virtual.rows) continue;
//
//                        int k = tex_coords[i].v * right_image.get_height();
//                        int l = tex_coords[i].u * right_image.get_width();
//                        if (k < 0 or k >= right_image.get_height() or l < 0 or l > right_image.get_width()) continue;
//
//                        color(right_image, frame_virtual, row_virtual, col_virtual, k, l);
//                    }
//                }
//            }
//            //cv::medianBlur(frame_virtual, frame_virtual, 3);
//
//            cv::GaussianBlur(frame_virtual, frame_virtual, cv::Size(13, 13), 0, 0, 0);
//
//            //cv::RNG rng(cv::getTickCount());
//            //float min_alpha = 0.1;
//            //float max_alpha = 2.0;
//            float alpha = 4.0; //rng.uniform(min_alpha, max_alpha);
//            float beta = -1.0;
//            frame_virtual.convertTo(frame_virtual, -1, alpha, beta);
//            //cv::imshow("Virtual", frame_virtual);
//            //non-blocking write to buffer_frame
//        }
//    }
//};

class SpecificWorker : public GenericWorker
{
Q_OBJECT
public:
	SpecificWorker(TuplePrx tprx, bool startup_check);
	~SpecificWorker();
	bool setParams(RoboCompCommonBehavior::ParameterList params);

	RoboCompCameraRGBDSimple::TRGBD CameraRGBDSimple_getAll(std::string camera);
	RoboCompCameraRGBDSimple::TDepth CameraRGBDSimple_getDepth(std::string camera);
	RoboCompCameraRGBDSimple::TImage CameraRGBDSimple_getImage(std::string camera);
	RoboCompLaser::TLaserData Laser_getLaserAndBStateData(RoboCompGenericBase::TBaseState &bState);
	RoboCompLaser::LaserConfData Laser_getLaserConfData();
	RoboCompLaser::TLaserData Laser_getLaserData();

public slots:
	void compute();
	int startup_check();
	void initialize(int period);

private:
    vector<uchar> buffer;
    cv::Mat m;
    //
	std::shared_ptr < InnerModel > innerModel;
	bool startup_check_flag;
    mutable std::mutex bufferMutex;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    std::string serial_left, serial_right;
    bool print_output = false;
    std::vector<rs2::pipeline>  pipelines;

    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg_left, cfg_right;
    rs2::context ctx;
    rs2_intrinsics left_cam_intr, right_cam_intr, left_depth_intr, right_depth_intr;
    //cv::Mat mosaic( const rs2::frameset &cdata_left, const rs2::frameset &cdata_right, unsigned short subsampling );
    cv::Mat mosaic(  const rs2::points &points_left, const rs2::points &points_right, const rs2::frameset &cdata_left, const rs2::frameset &cdata_right);
    std::tuple<std::vector<rs2::points>, std::vector<rs2::frameset>> read_and_filter();
    template <typename T>
    bool is_in_bounds(const T& value, const T& low, const T& high) { return !(value < low) && (value < high); }
    struct LaserPoint{ float dist; float angle;};
    void color(rs2::video_frame image, cv::Mat frame_v, int row_v, int col_v, int k, int l);

    //hilos
    //Filters *filters_thread;
    //Mosaic *mosaic_thread;
    //rs2::frame_queue *mosaic_queue;

    struct filter_options
    {
    public:
        std::string filter_name;                                   //Friendly name of the filter
        rs2::filter &filter;                                       //The filter in use
        std::atomic_bool is_enabled;                               //A boolean controlled by the user that determines whether to apply the filter or not

        filter_options(const std::string name, rs2::filter &flt) :
                filter_name(name),
                filter(flt),
                is_enabled(true)
        {
            const std::array<rs2_option, 5> possible_filter_options = {
                    RS2_OPTION_FILTER_MAGNITUDE,
                    RS2_OPTION_FILTER_SMOOTH_ALPHA,
                    RS2_OPTION_MIN_DISTANCE,
                    RS2_OPTION_MAX_DISTANCE,
                    RS2_OPTION_FILTER_SMOOTH_DELTA
            };
        }

        filter_options(filter_options&& other) :
                filter_name(std::move(other.filter_name)),
                filter(other.filter),
                is_enabled(other.is_enabled.load())
        { }
    };

    std::vector<filter_options> filters;
    rs2::decimation_filter dec_filter;  // Decimation - reduces depth frame density
    rs2::spatial_filter spat_filter;    // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter temp_filter;
    rs2::hole_filling_filter holef_filter;
    rs2::disparity_transform depth_to_disparity;
    rs2::disparity_transform disparity_to_depth;
    const std::string disparity_filter_name = "Disparity";

    //rs2::frame_queue *mosaic_queue;

    std::vector<rs2::pipeline>  pipes;
    rs2::frame rgb_list[2];
    rs2::frame depth_list[2];
    rs2::pointcloud pointclouds[2];
    rs2::points points[2];
};





#endif
