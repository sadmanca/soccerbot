#include <ros/ros.h>
#include <soccer_fieldline_detection/test.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <soccer_geometry/segment2.hpp>
#include <opencv2/highgui.hpp>

test_node::test_node() {

}

void test_node::imageCallback() {

    std::vector<cv::Vec4i> lines;
    std::vector<Point2> pts;


    try {

        int xMax = 0;
        int xMin = 100000;
        int yMax = 0;
        int yMin = 100000;
        //Get images of soccerball, goal post, bez in blue and red uniform
        //Imread
        cv::Mat image;
        image = cv::imread("/home/sk/catkin_ws/src/soccer_ws/soccer_fieldline_detection/media/pictures/bezRed.png");
        cv::namedWindow("test", cv::WINDOW_AUTOSIZE);
        //const cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; // change to non ros

        // Detect Field Lines (Copy from simulink)
        cv::Mat dst, cdst;
        cv::Mat hsv, mask,out;
        cvtColor(image, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv,cv::Scalar (0,150, 20), cv::Scalar(5, 250, 255), mask);

        // Detect rightmost leftmost topmost bottommost
        for(int i = 0; i < mask.size().width; i++) {
            for(int j = 0; j < mask.size().height; j++) {
                int sum = 0;
                int count = 0;
                for(int dx = -2; dx < 3; dx++) {
                    for(int dy = -2; dy < 3; dy++) {
                        if(mask.at<uchar>(j, i) ==255 && i + dx > 0 && i + dx < mask.size().width && j + dy > 0 && j + dy < mask.size().height) {
                            sum = sum + mask.at<uchar>(j + dy, i + dx);
                            count = count + 1;
                        }
                    }
                }
                if(count != 0) {
                    if(sum / count >= 122 && j < yMin) {
                        yMin = j;
                    } else if(sum / count >= 122 && j > yMax) {
                        yMax = j;
                    }
                    if(sum / count >= 122 && i < xMin) {
                        xMin = i;
                    } else if(sum / count >= 122 && i > xMax) {
                        xMax = i;
                    }
                }
            }
        }

        cv::rectangle(image, cv::Point((float)xMin, (float)yMin), cv::Point((float)xMax, (float)yMax), cv::Scalar(255, 0, 0), 3, cv::LINE_8, 0);

        //Imshow

        cv::imshow(" test", image);

        int k = cv::waitKey(0);
        cv::bitwise_and(image,image,out,mask);

        cvtColor(out, cdst, CV_BGR2GRAY);

        cv::threshold(cdst,dst, 127, 255,cv::THRESH_BINARY);


        cv::Canny(dst, cdst,50,150);



        HoughLinesP(cdst, lines, rho, theta,threshold,minLineLength,maxLineGap);
        cvtColor(cdst, dst, CV_GRAY2BGR);
        for (const auto& l : lines) {
            cv::line(dst, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 0, 255), 3, cv::LINE_AA);

            Point2 pt1(l[0],l[1]);
            Point2 pt2(l[2],l[3]);
            Segment2 s(pt1,pt2);
            float b = l[1] - s.slope()*l[0];

            for (size_t i = l[0]; i < l[2];i += 1) {
                float y = s.slope()*i + b;
                Point2 pt(i,y);
                pts.push_back(pt);
            }
        }


    } catch (const cv_bridge::Exception &e) {
        ROS_ERROR_STREAM("CV Exception" << e.what());
    }



}



int main(int argc, char **argv) {
    ros::init(argc, argv, "soccer_fieldline_detector");

    test_node detector;
    detector.imageCallback();
    ros::spin();

    return 0;
}
