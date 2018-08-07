#include <image_transport/image_transport.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <opencv2/highgui/highgui.hpp>  // Video write

#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/tf.h>

#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <fstream>
#include <iostream>
#include <math.h>
#include <deque>


using namespace sensor_msgs;
using namespace message_filters;
using namespace cv;
using namespace std;

string outputFolder = "/home/anurag/mother_ws/ORB_SLAM2_mathew/ORB_SLAM2/Examples/Monocular/output/framebyframe";
ofstream myfile;
char outputFolderstr[1024];
char txtfilename[1024];

char folder[1024];
char filepath[1024];
char bebop[] = "ardrone_hull";
Mat t_b2cb, R_b2cb;
int counter = 0;
Mat K_cam;
Mat D;

// KLT variables
const int MAX_COUNT = 500;
bool needToInit = true;
bool nightMode = false;

Point2f point;
vector<Point2f> points[2];
Mat gray, prevGray, image, frame;
int total_no_of_points;
//

void callback(const geometry_msgs::PoseStampedConstPtr &vrpn_pose,const geometry_msgs::PoseStampedConstPtr &ORB_pose,const sensor_msgs::ImageConstPtr &image_msg){
    bool DEBUG = 0;
    //if(1)cout << "Callback fired---------- counter" << counter << endl ;
    counter++;
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
    VideoCapture cap;

    //----------------------------
    TermCriteria termcrit(TermCriteria::COUNT|TermCriteria::EPS,20,0.03);
    Size subPixWinSize(10,10), winSize(31,31);


    namedWindow( "LK Demo", 1 );
    //setMouseCallback( "LK Demo", onMouse, 0 );





        //cout <<filepath <<  "\nK_cam\n" << K_cam << "\nD " << D << endl;
        Mat img_1_undist;undistort(cv_ptr->image, img_1_undist, K_cam, D);
        frame = img_1_undist;
        //Mat blank;blank = Mat::zeros(frame.rows,frame.cols,frame.type());

//    for(;;)
//    {
        if( frame.empty() )
            return;

        frame.copyTo(image);
        cvtColor(frame, gray, COLOR_BGR2GRAY);

        if( nightMode )
            image = Scalar::all(0);
        if( needToInit )
        {
            // automatic initialization
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);//cout << "1Here?" << endl;
            cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);//cout << "No" << endl;
            //cout << "no of points : " << points[1].size() << endl;
            total_no_of_points+=points[1].size();
        }
        else if( !points[0].empty() )
        {
            vector<uchar> status;
            vector<float> err;
            if(prevGray.empty())
                gray.copyTo(prevGray);//cout << " gray : " << gray.size() << "  prevGray : " << prevGray.size() << endl;
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                 3, termcrit, 0, 0.001);

         size_t i, k;
            for( i = k = 0; i < points[1].size(); i++ )
            {

                if( !status[i] )
                  {
                    //points[1].erase(points[1].begin()+i) ;k++;
                    continue;}

                points[1][k++] = points[1][i];
                circle( image, points[1][i], 3, Scalar(0,255,0), -1, 2);
            }
            points[1].resize(k);

        // opencv OPTICAL FLOW================================
        UMat  flowUmat, prevgrayU;
        prevGray.copyTo(prevgrayU);//cout << "here "<< endl;
        //cout << prevGray.size() << " " << gray.size() << " " <<  (prevGray.channels() == gray.channels()) << " " <<  (prevGray.channels() == 1) << endl;
        calcOpticalFlowFarneback(prevGray, gray, flowUmat, 0.4, 1, 12, 2, 8, 1.2, 0);
        //cout << "here "<< endl;
        Mat flow;
        flowUmat.copyTo(flow);
        //    for (int y = 0; y < image.rows; y += 2) {
        //     for (int x = 0; x < image.cols; x += 2)
        //     {
        //              // get the flow from y, x position * 10 for better visibility

        //              const Point2f flowatxy = flow.at<Point2f>(y, x) * 3;
        //              float fx = flowatxy.x*3.0;
        //              float fy = flowatxy.y*3.0;
        //              float mag = sqrt(fx*fx + fy*fy);

        //                     // draw line at flow direction
        //       line(blank, Point(x, y), Point(cvRound(x + 0.1*flowatxy.x), cvRound(y + 0.1*flowatxy.y)), Scalar((int)max(0.0,(255.0*fx/mag))  ,0,(int)  max(0.0,-(255.0*fx/mag))  ));//
        //                                                         // draw initial point
        //       circle(blank, Point(x, y), 1, Scalar(0, 0, 0), -1);


        //      }

        //     }
        // opencv OPTICAL FLOW================================

        // Dynamic features ==============================================
                int w = 50;
                int nd = 10;
                int sp = 5;
                int thres = 5;

                for(int i = 1; i <= nd ; i++)
                {
                    float xleft = 0, yup = 0, xrt = 0, ydown = 0;
                    int nxleft = 0, nyup = 0, nxrt = 0, nydown = 0;


                    for (int y = (i-1)*image.rows/nd; y < i*image.rows/nd; y += sp) {
                     for (int x = 0; x < w; x += sp)
                     {
                         const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
                         {xleft+=flowatxy.x;nxleft++;}
                     }
                    }

                    for (int y = (i-1)*image.rows/nd; y < i*image.rows/nd; y += sp) {
                     for (int x = image.cols - w; x < image.cols; x += sp)
                     {
                         const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
                         {xrt-=flowatxy.x;nxrt++;}
                     }
                    }

                    for (int y = 0; y < w; y += sp) {
                     for (int x = (i-1)*image.cols/nd; x < i*image.cols/nd; x += sp)
                     {
                         const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
                         {yup+=flowatxy.y;nyup++;}
                     }
                    }

                    for (int y = image.rows - w; y < image.rows; y += sp) {
                     for (int x = (i-1)*image.cols/nd; x < i*image.cols/nd; x += sp)
                     {
                         const Point2f flowatxy = flow.at<Point2f>(y, x) * 10;
                         {ydown-=flowatxy.y;nydown++;}
                     }
                    }

                    //cout << i << " " << ((nxleft)) << " " << ((nxrt)) << " " << ((nyup)) << " " << ((nydown)) << endl;

                    //w = 25;
                       if( (xleft/nxleft) > thres )
                       {
                           vector<Point2f> newpoints;
                           Mat mask(gray.size(),CV_8U,Scalar(0));
                           mask(Rect(0,(i-1)*image.rows/10,w,image.rows/10)) = 1;//cout << "2Here?" << endl;
                           goodFeaturesToTrack(gray, newpoints, MAX_COUNT/300, 0.01, 10, mask, 3, 3, 0, 0.04);
                           if(newpoints.size() > 0)cornerSubPix(gray, newpoints, subPixWinSize, Size(-1,-1), termcrit);//cout << "No" << endl;
                           for(int i =0 ; i < newpoints.size(); i++)
                           {
                               points[1].push_back(newpoints[i]);total_no_of_points++;
                           }

                       }

                       if( (xrt/nxrt) > thres )
                       {
                           vector<Point2f> newpoints;
                           Mat mask(gray.size(),CV_8U,Scalar(0));
                           mask(Rect(image.cols - w ,(i-1)*image.rows/10,w,image.rows/10)) = 1;
                           goodFeaturesToTrack(gray, newpoints, MAX_COUNT/300, 0.01, 10, mask, 3, 3, 0, 0.04);//cout << "3Here?" << endl;
                           if(newpoints.size() > 0)cornerSubPix(gray, newpoints, subPixWinSize, Size(-1,-1), termcrit);//cout << "No" << endl;
                           for(int i =0 ; i < newpoints.size(); i++)
                           {
                               points[1].push_back(newpoints[i]);total_no_of_points++;
                           }

                       }

                       if( (yup/nyup) > thres )
                       {
                           vector<Point2f> newpoints;
                           Mat mask(gray.size(),CV_8U,Scalar(0));
                           mask(Rect((i-1)*image.cols/10,0,image.cols/10,w)) = 1;
                           goodFeaturesToTrack(gray, newpoints, MAX_COUNT/300, 0.01, 10, mask, 3, 3, 0, 0.04);//cout << "4Here?" << endl;
                           if(newpoints.size() > 0)cornerSubPix(gray, newpoints, subPixWinSize, Size(-1,-1), termcrit);//cout << "No" << endl;
                           for(int i =0 ; i < newpoints.size(); i++)
                           {
                               points[1].push_back(newpoints[i]);total_no_of_points++;
                           }

                       }

                       if( (ydown/nydown) > thres )
                       {
                           vector<Point2f> newpoints;
                           Mat mask(gray.size(),CV_8U,Scalar(0));
                           mask(Rect((i-1)*image.cols/10,image.rows-w,image.cols/10,w)) = 1;
                           goodFeaturesToTrack(gray, newpoints, MAX_COUNT/300, 0.01, 10, mask, 3, 3, 0, 0.04);//cout << "Here?" << endl;
                           if(newpoints.size() > 0)cornerSubPix(gray, newpoints, subPixWinSize, Size(-1,-1), termcrit);
                           for(int i =0 ; i < newpoints.size(); i++)
                           {
                               points[1].push_back(newpoints[i]);total_no_of_points++;
                           }

                       }

                }
        // Dynamic features ==============================================




        }



        // Maintian no of points============================
        int np = MAX_COUNT;
        int k = points[1].size();
        if( k < np)
        {
            vector<Point2f> newpoints;
            goodFeaturesToTrack(gray, newpoints, MAX_COUNT, 0.01, 10, Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, newpoints, subPixWinSize, Size(-1,-1), termcrit);
            int newp = newpoints.size();
            for(size_t i = 0; i < min(newp,np-k); i++)
            {
                points[1].push_back(newpoints[i]);total_no_of_points++;
            }

        }
        // Maintian no of points============================


        cout << " Totla no of points : " << total_no_of_points << endl;
        needToInit = false;
//        cout << "im size : " << image.size() << endl;
        imshow("LK Demo", image);
//        cout << "blank size : " << blank.size() << endl;
//        imshow("Blank", blank);

        char c = (char)waitKey(10);
        //cout << " Keypress : " << c << endl;
        //cout << "bool  : " << (c == 'q') << endl;
        if( c == 'q' )
        {ros::shutdown();return;}
        switch( c )
        {
        case 'r':
            needToInit = true;
            break;
        case 'c':
            points[0].clear();
            points[1].clear();
            break;
        case 'n':
            nightMode = !nightMode;
            break;
        case ' ':
            waitKey();
        }

        std::swap(points[1], points[0]);
        //cout << " points[0].size : " << points[0].size() << endl;
        cv::swap(prevGray, gray);
//   }

//-------------------------------
return;
}



int main(int argc, char** argv)
{
    bool DEBUG = 0;
    strcpy(outputFolderstr, outputFolder.c_str());
    sprintf(txtfilename,"%s/pose.txt",outputFolderstr);
    //myfile.open (txtfilename);
    //myfile.close();
    

    sprintf(folder,"/home/anurag/mother_ws/ardrone_calibration_ws/src/solve_pnp_calib/dataset");
    sprintf(filepath,"%s/RT_b2co_%s.yml",folder,bebop);
    FileStorage fs_e(filepath, FileStorage::READ);
    fs_e["R_b2co"] >> R_b2cb;
    fs_e["T_b2co"] >> t_b2cb;
    if(DEBUG) cout << "R_b2cb" << endl << R_b2cb << endl << "t_b2cb" << t_b2cb << endl;

    sprintf(folder,"/home/anurag/orb/build");
    sprintf(filepath,"%s/internal_calib_ardrone.yml",folder);
    FileStorage fs_i(filepath, FileStorage::READ);
    fs_i["camera_matrix"] >> K_cam;
    fs_i["distortion_coefficients"] >> D;



  ros::init(argc, argv, "pose_for_dfusmc_node");
  ros::NodeHandle nh;


  // --- Time Synchronized Messages
  char quad_topic[1024] = ""; sprintf(quad_topic,"/vrpn_client_node/%s/pose",bebop);
	  message_filters::Subscriber<geometry_msgs::PoseStamped> vrpn_sub(nh, quad_topic, 1);
  sprintf(quad_topic,"/ORB_SLAM/pose");
	  message_filters::Subscriber<geometry_msgs::PoseStamped> ORB_sub(nh, quad_topic, 1);

  	  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/ardrone/image_raw", 1);
  
  //TimeSynchronizer<geometry_msgs::PoseStamped, nav_msgs::Odometry> sync(vrpn_sub, odom_sub, 10);
  typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped,geometry_msgs::PoseStamped,sensor_msgs::Image > MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), vrpn_sub,ORB_sub, image_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();
  return 0;
}
