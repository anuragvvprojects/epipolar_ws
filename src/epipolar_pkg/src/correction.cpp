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

void split(const string &s, char delim, vector<string> &elems) {
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
}
vector<string> split(const string &s, char delim) {
    vector<string> elems;
    split(s, delim, elems);
    return elems;
}


int main(int argc, char** argv)
{
    bool DEBUG = 0;
        strcpy(outputFolderstr, outputFolder.c_str());
        sprintf(txtfilename,"%s/pose.txt",outputFolderstr);
	ifstream data;
	data.open(txtfilename);
	
	

	while(!data.eof())
	{
		string dummy;getline(data, dummy);
		vector<string> words = split(dummy,' ');
		//for(int i = 0 ; i < words.size() ; i++)
		//{

		//	cout << words[i] << endl;
		//}
		cout << words[6] << " \t" ;
		if(words[6][0] == '-')
			{cout << words[6][9] << " ";}
		else if(words[6][0] == '0')
			{cout << words[6][8] << " ";}
//		find ( words[6], 

         }       

}








