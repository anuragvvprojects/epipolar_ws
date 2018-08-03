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
Mat K, D;
Mat t_b2cb, R_b2cb;
int counter = 0;


void from_b2co(float* msg_in , float* msg_out);
void angle_betwee_q(float* q1, float* q2, float*q );


void angle_betwee_q(float* q1, float* q2, float* ang )
{
   *ang = acos(   q1[0]*q2[0]
            -  q1[1]*q2[1]
            -  q1[2]*q2[2]
            -  q1[3]*q2[3]);
}

void from_b2co(float* msg_in , float* msg_out)
{
    bool DEBUG = 0;if(DEBUG)cout << "from_b2co" << endl;
    float r_x, r_y, r_z, r_qx, r_qy, r_qz, r_qw;
    r_x = msg_in[1];
    r_y = msg_in[2];
    r_z = msg_in[3];
    r_qw = msg_in[4];
    r_qx = msg_in[5];
    r_qy = msg_in[6];
    r_qz = msg_in[7];
		if(DEBUG)cout << " 1 " << endl;
    Eigen::Matrix3d mat3 = Eigen::Quaterniond(r_qw, r_qx, r_qy, r_qz).toRotationMatrix();
    Mat R_b2w;
    cv::eigen2cv(mat3,R_b2w);
    Mat t_b2w =(Mat_<double>(3,1) << r_x,r_y,r_z);
    if(DEBUG)cout << "R_b2w : \n" << R_b2w << endl << "t_b2w : \n" << t_b2w << endl;
    if(DEBUG)cout << "R_b2w ttype: \n" << R_b2w.type() << endl << "R_b2cb.type() : \n" << R_b2cb.type() << endl;
		if(DEBUG)cout << " 2 " << endl;
    Mat R_w2cb = R_b2cb * R_b2w.t();
    Mat t_w2cb = t_b2cb + R_b2cb*(-R_b2w.t()*t_b2w);
    Mat R_cb2w = R_w2cb.inv();
    Mat t_cb2w = -R_cb2w*t_w2cb;
		if(DEBUG)cout << " 3 " << endl;
    // Listening to T_co2cb
    tf::TransformListener listener;
    tf::StampedTransform tf_co2cb;
    geometry_msgs::TransformStamped tf_co2cb_gm;
    Eigen::Affine3d  T_co2cb;
		if(DEBUG)cout << " 4 " << endl;
/*    try{
        ros::Time now = ros::Time::now();
           tf_co2cb_gm = tfBuffer->lookupTransform( "camera_base_link","camera_optical", now );
        }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
    }
*/
		if(DEBUG)cout << " 5 " << endl;
    //tf::transformTFToEigen(tf_co2cb,T_co2cb);
    Eigen::Matrix3d mat3_co2cb = Eigen::Quaterniond(tf_co2cb_gm.transform.rotation.w,
                                                    tf_co2cb_gm.transform.rotation.x,
                                                    tf_co2cb_gm.transform.rotation.y,
                                                    tf_co2cb_gm.transform.rotation.z ).toRotationMatrix();
   // mat3 = T_co2cb.matrix().block(0,0,3,3);
		if(DEBUG)cout << " 5 " << endl;
    Mat R_co2cb;
    cv::eigen2cv(mat3_co2cb,R_co2cb);
    //Mat t_co2cb = (Mat_<double>(3,1) << (float)T_co2cb(0,3),(float)T_co2cb(1,3),(float)T_co2cb(2,3));
    Mat t_co2cb =  (Mat_<double>(3,1) << (double)tf_co2cb_gm.transform.translation.x,
                                         (double)tf_co2cb_gm.transform.translation.y,
                                         (double)tf_co2cb_gm.transform.translation.z );
		if(DEBUG)cout << " 6 " << endl;
    Mat R_co2w = R_cb2w * R_co2cb ;
    Mat t_co2w = R_co2cb * t_cb2w + t_co2cb ;
    if(DEBUG)cout << "R_co2cb : \n" << R_co2cb << endl << "t_co2cb : \n" << t_co2cb << endl;
		if(DEBUG)cout << " 7 " << endl;
    Eigen::Matrix3d mat3_out;
    cv::cv2eigen(R_cb2w,mat3_out);
		if(DEBUG)cout << " 8 " << endl;
    Eigen::Quaterniond Q_out(mat3_out);
    msg_out[0] = t_cb2w.at<double>(0,0);
    msg_out[1] = t_cb2w.at<double>(1,0);
    msg_out[2] = t_cb2w.at<double>(2,0);
    msg_out[3] = Q_out.w();
    msg_out[4] = Q_out.x();
    msg_out[5] = Q_out.y();
    msg_out[6] = Q_out.z();

}



void callback(const geometry_msgs::PoseStampedConstPtr &vrpn_pose,const geometry_msgs::PoseStampedConstPtr &ORB_pose,const sensor_msgs::ImageConstPtr &image_msg){
    bool DEBUG = 0;
    if(1)cout << "Callback fired---------- counter" << counter << endl ;
    counter++;
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

    char imgfilename[1024];
    sprintf(imgfilename,"%s/img%d.jpg",outputFolderstr,counter);
    cv::imwrite(imgfilename,cv_ptr->image);
	


    float q[7];
    q[0] = (*vrpn_pose).pose.position.x;
    q[1] = (*vrpn_pose).pose.position.y;
    q[2] = (*vrpn_pose).pose.position.z;
    q[3] = (*vrpn_pose).pose.orientation.w;
    q[4] = (*vrpn_pose).pose.orientation.x;
    q[5] = (*vrpn_pose).pose.orientation.y;
    q[6] = (*vrpn_pose).pose.orientation.z;	if(DEBUG)cout << "msg_in" << endl;

    float q2[7];
    q2[0] = (*ORB_pose).pose.position.x;
    q2[1] = (*ORB_pose).pose.position.y;
    q2[2] = (*ORB_pose).pose.position.z;
    q2[3] = (*ORB_pose).pose.orientation.w;
    q2[4] = (*ORB_pose).pose.orientation.x;
    q2[5] = (*ORB_pose).pose.orientation.y;
    q2[6] = (*ORB_pose).pose.orientation.z;	if(DEBUG)cout << "msg_ORB" << endl;

    strcpy(outputFolderstr, outputFolder.c_str());
    sprintf(txtfilename,"%s/pose.txt",outputFolderstr);
    myfile.open (txtfilename,std::ofstream::app);
    myfile  << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << " " << q[4] << " " << q[5] << " " << q[6] << " "
	    << q2[0] << " " << q2[1] << " " << q2[2] << " " << q2[3] << " " << q2[4] << " " << q2[5] << " " << q2[6] << "\n";  
    myfile.close();


}



int main(int argc, char** argv)
{
    bool DEBUG = 0;
    strcpy(outputFolderstr, outputFolder.c_str());
    sprintf(txtfilename,"%s/pose.txt",outputFolderstr);
    myfile.open (txtfilename);
    myfile.close();
    

    sprintf(folder,"/home/anurag/mother_ws/ardrone_calibration_ws/src/solve_pnp_calib/dataset");
    sprintf(filepath,"%s/RT_b2co_%s.yml",folder,bebop);
    FileStorage fs_e(filepath, FileStorage::READ);
    fs_e["R_b2co"] >> R_b2cb;
    fs_e["T_b2co"] >> t_b2cb;
    if(DEBUG) cout << "R_b2cb" << endl << R_b2cb << endl << "t_b2cb" << t_b2cb << endl;



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
