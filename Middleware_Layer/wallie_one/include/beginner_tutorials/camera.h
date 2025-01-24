#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>



using namespace std;
using namespace cv;

class CameraDriver
{
public:
    int id_camera;
    int fps;
    bool show;
    VideoCapture input_video;
    Mat frame;
    cv_bridge::CvImage cvi;
    image_transport::ImageTransport *it;
    image_transport::Publisher pub;
    explicit CameraDriver();

};
