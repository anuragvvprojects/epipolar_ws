#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<fstream>
#include <eigen3/Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include "opencv2/imgproc.hpp"
#include <opencv2/plot.hpp>

using namespace std;
using namespace cv;


char folder[1024];
char filepath[1024];
Mat K, D;
Mat t_b2co, R_b2co;
char bebop[] = "ardrone_hull";

int main ( int argc, char** argv )
{

    // Read from file
    ifstream poses_file;
    poses_file.open("/home/anurag/mother_ws/ORB_SLAM2_mathew/ORB_SLAM2/Examples/Monocular/output/framebyframe/pose.txt");

    int no = 0 ;
    float poses[10000][14];
    while(!poses_file.eof())
    {
        no++;

        poses_file >> poses[no][0]
                 >> poses[no][1]
                 >> poses[no][2]

                 >> poses[no][3]
                 >> poses[no][4]
                 >> poses[no][5]
                 >> poses[no][6]
         >> poses[no][7]
                 >> poses[no][8]
                 >> poses[no][9]

                 >> poses[no][10]
                 >> poses[no][11]
                 >> poses[no][12]
                 >> poses[no][13] ;
        string dummy;
        getline(poses_file, dummy);
        cout << no
             << poses[no][7+0] << " "
                 << poses[no][7+1]<< " "
                 << poses[no][7+2]<< " "

                 << poses[no][7+3]<< " "
                 << poses[no][7+4]<< " "
                 << poses[no][7+5]<< " "
                 << poses[no][7+6]<< endl ;
    }

    cout << "no : " << no << endl;

   int a1 = std::atoi(argv[1]);
   int a2 = std::atoi(argv[2]);
   //cout << "a1 : " << a1 << "  a2 : " << a2 << endl;


  sprintf(folder,"/home/anurag/mother_ws/ardrone_calibration_ws/src/solve_pnp_calib/dataset");
  sprintf(filepath,"%s/RT_b2co_%s.yml",folder,bebop);
  FileStorage fs_e(filepath, FileStorage::READ);
  fs_e["R_b2co"] >> R_b2co; if(1)cout << " R_b2co\n " << R_b2co <<  endl;
  fs_e["T_b2co"] >> t_b2co;if(1)cout << " t_b2co\n " << t_b2co <<  endl;


   int np = 5;
for(int i = 2000 ; i < no-np-2 ; i++)
{   cout << i << endl;
    a1 = i+1;
    a2 = i+np;
    //return 0;
    // REad IMages
    string outputFolder = "/home/anurag/mother_ws/ORB_SLAM2_mathew/ORB_SLAM2/Examples/Monocular/output/framebyframe";
    char outputFolderstr[1024];
    strcpy(outputFolderstr, outputFolder.c_str());
    char imgfilename[1024];
    sprintf(imgfilename,"%s/img%d.jpg",outputFolderstr,a1);
    Mat img_1 = imread ( imgfilename, CV_LOAD_IMAGE_COLOR );
    sprintf(imgfilename,"%s/img%d.jpg",outputFolderstr,a2);
    Mat img_2 = imread ( imgfilename, CV_LOAD_IMAGE_COLOR );

    // Undistort
    char filepath[1024] = "_0";
    char folder[1024] = "_0";
    sprintf(folder,"/home/anurag/orb/build");
    sprintf(filepath,"%s/internal_calib_ardrone.yml",folder);
    FileStorage fs_i(filepath, FileStorage::READ);
    Mat K_cam;
    Mat D;
    fs_i["camera_matrix"] >> K_cam;
    fs_i["distortion_coefficients"] >> D;
    //cout <<filepath <<  "\nK_cam\n" << K_cam << "\nD " << D << endl;
    Mat img_1_undist;undistort(img_1, img_1_undist, K_cam, D);
    Mat img_2_undist;undistort(img_2, img_2_undist, K_cam, D);
    img_1 = img_1_undist;
    img_2 = img_2_undist;



    // Initialize vectors
    std::vector<KeyPoint> keypoints_1, keypoints_2;
    Mat descriptors_1, descriptors_2;
    Ptr<FeatureDetector> detector = ORB::create();
    Ptr<DescriptorExtractor> descriptor = ORB::create();

    // Matcher
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );

 ///  STAGE 1 -- Keypoints in both images
    // Detect keypoints
    detector->detect ( img_1,keypoints_1 );
    detector->detect ( img_2,keypoints_2 );

    // Compute descriptors of Keypoints
    descriptor->compute ( img_1, keypoints_1, descriptors_1 );
    descriptor->compute ( img_2, keypoints_2, descriptors_2 );

    // Show
    Mat outimg1;
    //drawKeypoints( img_1, keypoints_1, outimg1, Scalar::all(-1), DrawMatchesFlags::DEFAULT );
    //imshow("image_with_ORB",outimg1);


 ///  STAGE 2 -- match keypoints
    // Match keypoints
    vector<DMatch> matches;
    matcher->match ( descriptors_1, descriptors_2, matches );

 /// STAGE 4 -- find R,t, F
      Mat t1 = (Mat_<float>(3,1) << poses[a1][7+0], poses[a1][7+1], poses[a1][7+2]);
      Mat t2 = (Mat_<float>(3,1) << poses[a2][7+0], poses[a2][7+1], poses[a2][7+2]);
      Eigen::Matrix3f mat3 = Eigen::Quaternionf(poses[a1][7+3], poses[a1][7+4], poses[a1][7+5], poses[a1][7+6]).toRotationMatrix();
      Mat R1;
      cv::eigen2cv(mat3,R1);
      R_b2co.convertTo(R_b2co, R1.type());
      t_b2co.convertTo(t_b2co, t1.type());
      R1 = R1*R_b2co.inv();
      t1 = -R1*R_b2co.inv()*t_b2co + t1;

      mat3 = Eigen::Quaternionf(poses[a2][7+3], poses[a2][7+4], poses[a2][7+5], poses[a2][7+6]).toRotationMatrix();
      Mat R2;
      cv::eigen2cv(mat3,R2);
      R2 = R2*R_b2co.inv();
      t2 = -R2*R_b2co.inv()*t_b2co + t2;

      Mat t = R1.t()*(t2 - t1);
      Mat R = R1.t()*R2;

      float fx= 423.5067530620715;
      float fy= 424.2758201841426;
      float cx= 310.1344624982929;
      float cy= 249.0244080504391;

      Mat K = (Mat_<float>(3,3) << fx, 0, cx,
                                    0, fy, cy,
                                    0, 0, 1);

      Mat T = (Mat_<float>(3,3) << 0, -t.at<float>(2), t.at<float>(1),
                                   t.at<float>(2), 0, -t.at<float>(0),
                                  -t.at<float>(1), t.at<float>(0), 0);

      Mat F = (K.t()).inv() * T.t() * R * K.inv();
      //cout<<F<<endl<<endl<<K<<endl<<endl<<R<<endl<<endl<<T<<endl<<endl<<t<<endl;





 ///  STAGE 3 -- Find good matches, dynamic_matches, veryBad_matches
 ///

      // Find epipolar error of each match
      vector<float> e;
      vector<float> dist;
      vector<float> ORBdist;
      outimg1 = img_1.clone();
      float minm = 5000000;
      float maxm = 0;
      double minT, maxT;
      minMaxLoc(t, &minT, &maxT);
      for(int i =0; i< matches.size(); i++)
      {
          Mat P1 = (Mat_<float>(3,1) << keypoints_1[matches[i].queryIdx].pt.x, keypoints_1[matches[i].queryIdx].pt.y , 1.0);
          Mat P2 = (Mat_<float>(3,1) << keypoints_2[matches[i].trainIdx].pt.x, keypoints_2[matches[i].trainIdx].pt.y , 1.0);
          Mat er = P1.t()*F*P2;
          //cout<<er<<endl;
          float r = fabs(er.at<float>(0))/fabs(maxT);
          e.push_back(r);
          float x1 = keypoints_1[matches[i].queryIdx].pt.x;
          float y1 = keypoints_1[matches[i].queryIdx].pt.y;

          float x2 = keypoints_2[matches[i].trainIdx].pt.x;
          float y2 = keypoints_2[matches[i].trainIdx].pt.y;
          float x = x1-x2;
          float y = y1-y2;

          dist.push_back( std::sqrt(x*x + y*y)  );
          ORBdist.push_back( matches[i].distance  );

      }

      auto biggest = std::max_element(std::begin(e), std::end(e));
      auto smallest = std::min_element(std::begin(e), std::end(e));
      cout << "E : max : " << (*biggest) << " min : " << (*smallest) << " translation : " << t << "\n";
      biggest = std::max_element(std::begin(dist), std::end(dist));
      smallest = std::min_element(std::begin(dist), std::end(dist));
      //cout << "DIST : max : " << (*biggest)*1000.0 << " min : " << (*smallest)*1000.0 << " ";
      biggest = std::max_element(std::begin(ORBdist), std::end(ORBdist));
      smallest = std::min_element(std::begin(ORBdist), std::end(ORBdist));
      cout << "ORB max : " << (*biggest) << " min : " << (*smallest) << endl;
      maxm = *biggest;
      minm = *smallest;

//return 1;

    // Classify matches into good bad and ugly.
    float good_thres = 0.05;
    float dyn_thres = 0.8, orbthres = 40;
    std::vector< DMatch > good_matches;vector<int> good_ind;
    std::vector< DMatch > dyn_matches;vector<int> dyn_ind;
    std::vector< DMatch > ugly_matches;vector<int> ugly_ind;
    for(int i =0; i< matches.size(); i++)
    {
        if ( e[i] <= good_thres && ORBdist[i] <= orbthres)
        {
//            if(matches[i].distance < 20)
//            {}
            good_matches.push_back ( matches[i] );good_ind.push_back(i);

        }
        else {
            if(e[i] <= dyn_thres  && ORBdist[i] <= orbthres )
                {
                    dyn_matches.push_back(matches[i]);dyn_ind.push_back(i);
                }
                else
                {
                    ugly_matches.push_back(matches[i]);ugly_ind.push_back(i);
                }
        }
    }

    Mat img_goodmatches;
    vector<vector<char>> dumV;
    std::vector<char> matchesMask = vector<char>();
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatches ,Scalar::all(-1),Scalar(0,0,255),matchesMask,2);
    cout << " GOOD MATCHES : \n";
    for(int i =0; i< good_matches.size(); i++)
    {
        if(0)cout << keypoints_1[good_matches[i].queryIdx].pt.x << " " << keypoints_2[good_matches[i].trainIdx].pt.x << " "
             << keypoints_1[good_matches[i].queryIdx].pt.y << " " << keypoints_2[good_matches[i].trainIdx].pt.y << " "
             << keypoints_1[good_matches[i].queryIdx].pt.x - keypoints_2[good_matches[i].trainIdx].pt.x << " "
             << keypoints_1[good_matches[i].queryIdx].pt.y - keypoints_2[good_matches[i].trainIdx].pt.y << " \t\t\t\t"
             << dist[good_ind[i]] << endl;

        float r = e[good_ind[i]];
        float frac = r;
        //cout<<i<<" "<< r << " " << good_ind[i] << " " << e[good_ind[i]] << endl;
        circle(img_goodmatches,keypoints_1[good_matches[i].queryIdx].pt,(int)(5*frac)+1,Scalar(0,(int)((1-2*r)*225.0),(int)(2*r*225.0)), (int)(r*225.0/50)*0+3);
    }
    Mat img_dynmatches;
    cout << "dyn_matches.size() : " <<dyn_matches.size() << endl;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, dyn_matches, img_dynmatches ,Scalar::all(-1),Scalar(0,0,255),matchesMask,2);
    for(int i =0; i< dyn_matches.size(); i++)
    {
        float r = (float)ORBdist[i]/10000;//e[dyn_ind[i]];
        //cout<< i <<" "<<ORBdist[i]<<endl;
        float frac = r;
        circle(img_dynmatches,keypoints_1[dyn_matches[i].queryIdx].pt,(int)(5*frac)+1,Scalar(0,(int)((1-2*r)*225.0),(int)(2*r*225.0)), (int)(r*225.0/50)*0+3);
    }
    cout << "ugly_matches.size() : " <<ugly_matches.size() << endl;
    Mat img_uglymatches;
    drawMatches ( img_1, keypoints_1, img_2, keypoints_2, ugly_matches, img_uglymatches ,Scalar::all(-1),Scalar(0,0,255),matchesMask,2);
    for(int i =0; i< ugly_matches.size(); i++)
    {
        float r = e[ugly_ind[i]];
        float frac = r;
        circle(img_uglymatches,keypoints_1[ugly_matches[i].queryIdx].pt,(int)(5*frac)+1,Scalar(0,(int)((1-2*r)*225.0),(int)(2*r*225.0)), (int)(r*225.0/50)*0+3);
    }


    imshow ( "image_goodmatch", img_goodmatches );
    imshow ( "image_dynmatch", img_dynmatches );
    imshow ( "image_uglymatch", img_uglymatches );
    int k = waitKey(8);
    //cout << "k = " << k << endl;
    if(k == 32)
        {waitKey();}
    if(k == 27)
        {return 0;}



}


    return 0;
}
