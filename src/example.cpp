#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "flow_opencv.hpp"
#include "flow_px4.hpp"
// example: https://github.com/PX4/sitl_gazebo/blob/master/src/gazebo_opticalFlow_plugin.cpp

using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{
    int camera_id = 0;
    if (argc > 1) {
       camera_id = atoi(argv[1]);
    }

    cout << "Built with OpenCV " << CV_VERSION << endl;
    Mat image;
    VideoCapture capture;
    capture.open(camera_id);

    int cap_width = 320;
    int cap_height = 240;
    capture.set(CV_CAP_PROP_FRAME_WIDTH, cap_width);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT, cap_height);
    capture.set(CV_CAP_PROP_FPS, 125); // for sony camera

    if(capture.isOpened())
    {
        cout << "Capture is opened" << endl;

        int frame_i = 0;
        int num_frames_to_print = 100;
        double total_tick = 0;
        double process_tick = 0;
        float focal_length = 0.05; // TODO
        OpticalFlowOpenCV of(focal_length, focal_length);
        //OpticalFlowPX4 of(focal_length, focal_length);
        Rect roi = Rect((cap_width - cap_height) / 2, 0, cap_height, cap_height);
        
        double last_t_begin = 0;
        double first_frame_time = (double)getTickCount();
        float flow_x = 0;
        float flow_y = 0;
        float flow_x_draw = 0;
        float flow_y_draw = 0;

        Mat bgr, gray;
        for(;;)
        {
            double t_begin = (double)getTickCount();
            capture >> image;
            if(image.empty())
                break;
            //drawText(image);
            
            Mat image_roi = image(roi);
            resize(image_roi, bgr, Size(DEFAULT_IMAGE_WIDTH, DEFAULT_IMAGE_HEIGHT));
            cvtColor(bgr, gray, CV_BGR2GRAY);
            double t_p_begin = (double)getTickCount();
            int dt =  (t_begin - last_t_begin) * 1e6 / getTickFrequency();
            
            uint32_t frame_time_us = int((t_p_begin - first_frame_time) * 1e6 / getTickFrequency()); //since start
            
            int quality = of.calcFlow(gray.data, frame_time_us, dt, flow_x, flow_y);
            if (quality >= 0) { // calcFlow(...) returns -1 if data should not be published yet -> output_rate
              //cout << frame_time_us << "\t" << quality << "\t" << flow_x << "\t" << flow_y << endl;
              float k = 0.5;
              flow_x_draw = flow_x * k + (1 - k) * flow_x_draw;
              flow_y_draw = flow_y * k + (1 - k) * flow_x_draw;
            }
            
              CvPoint center;
              center.x = image.cols / 2;
              center.y = image.rows / 2;
              CvPoint centerTo;
              double scale = 30;
              int dx = int(flow_x_draw * scale);
              int dy = int(flow_y_draw * scale);
              centerTo.x = center.x + dx;
              centerTo.y = center.y + dy;
              circle(image, center, 5, Scalar(255,0,0,255), -1, 8);
              line(image, center, centerTo, Scalar(255,0,255,255), 1);
            
            double t_p_end = (double)getTickCount();
            process_tick += (t_p_end - t_p_begin);

//#ifndef NDEBUG
            //of.debugDraw(image);
            imshow("optical flow", image);
            imshow("gray", gray);
            waitKey(1);
            //if(waitKey(1) >= 0)
            //    break;
//#endif
            double t_end = (double)getTickCount();
            total_tick += (t_end - t_begin);
            frame_i++;
            if ((frame_i % num_frames_to_print) == 0) {
                double tickFrequency = getTickFrequency();
                cout << "framerate:" << int(tickFrequency/total_tick * num_frames_to_print)
                     << " processrate:" << int(tickFrequency/process_tick * num_frames_to_print) <<endl;
                total_tick = 0;
                process_tick = 0;
            }
            last_t_begin = t_begin;
        }
    }
    else
    {
        cout << "No capture" << endl;
    }
    return 0;
}

