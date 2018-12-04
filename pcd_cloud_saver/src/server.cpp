#include <iostream>
#include <ros/ros.h>
#include <pcd_cloud_saver/save.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gazebo_msgs/GetModelState.h>

#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


#include <sensor_msgs/PointCloud2.h>


#define SERVICE_NAME "pcd_cloud_saver_service"
#define TOPIC_NAME "/camera/depth/points"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

void callback(const sensor_msgs::PointCloud2 &msg)
{	
	cloud->clear();
	pcl::fromROSMsg(msg,*cloud);
}


void save_cloud(string cloud_name,Eigen::Matrix4f transform_matrix,geometry_msgs::Pose pose)
{
	pcl::PCDWriter writer;

	cout<<"Saving the cloud!"<<endl;

	const string pkg="pcd_cloud_saver";
	string path = ros::package::getPath(pkg);
  	
	//cout<<"Path :"<<path<<endl;

	stringstream ss;
	ss<<path<<"/clouds/"<<cloud_name<<".pcd";
	//ss<<path<<"/clouds/cloud.pcd";

	writer.write(ss.str(),*cloud);

	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud(*cloud,*transformed_cloud,transform_matrix);

	cout<<"Transformed cloud size :"<<transformed_cloud->points.size()<<endl;

	ss.str(string());  //clearing the contents of the stringstream variable
	ss<<path<<"/clouds/"<<cloud_name<<"_transformed.pcd";

	writer.write(ss.str(),*transformed_cloud);

	/*noise_cloud->width=transformed_cloud->width;
    noise_cloud->height=transformed_cloud->height;
    noise_cloud->points.resize(transformed_cloud->points.size());

    boost::mt19937 rng; rng.seed (static_cast<unsigned int> (time (0)));
  	boost::normal_distribution<> nd (0, 0.003);
  	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor (rng, nd);

  	for (size_t i = 0; i < transformed_cloud->points.size (); i++)
  	{
  		noise_cloud->points[i].x=transformed_cloud->points[i].x+static_cast<float>(var_nor());
  		noise_cloud->points[i].y=transformed_cloud->points[i].y+static_cast<float>(var_nor());
  		noise_cloud->points[i].z=transformed_cloud->points[i].z+static_cast<float>(var_nor());
  		//noise_cloud->points[i].z=filtered_model->points[i].z;
  	}


  	ss.str(string());
  	ss<<path<<"/clouds/"<<cloud_name<<"_noisy.pcd";
  	writer.write(ss.str(),*noise_cloud);


  	noise_cloud_2->width=transformed_cloud->width;
  	noise_cloud_2->height=transformed_cloud->height;
  	noise_cloud_2->points.resize(transformed_cloud->points.size());

  	boost::mt19937 rng2; rng2.seed (static_cast<unsigned int> (time (0)));
  	boost::normal_distribution<> nd2 (0, 0.005);
  	boost::variate_generator<boost::mt19937&, boost::normal_distribution<> > var_nor2 (rng2, nd2);

  	for (size_t i = 0; i < transformed_cloud->points.size (); i++)
  	{
  		noise_cloud_2->points[i].x=transformed_cloud->points[i].x+static_cast<float>(var_nor2());
  		noise_cloud_2->points[i].y=transformed_cloud->points[i].y+static_cast<float>(var_nor2());
  		noise_cloud_2->points[i].z=transformed_cloud->points[i].z+static_cast<float>(var_nor2());
  		//noise_cloud->points[i].z=filtered_model->points[i].z;
  	}

  	ss.str(string());
  	ss<<path<<"/clouds/"<<cloud_name<<"_noisy_2.pcd";
  	writer.write(ss.str(),*noise_cloud_2);*/


	/*Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

	transform_2.translation()<<pose.position.x,pose.position.y,pose.position.z;

	cout<<transform_2.matrix()<<endl;

	pcl::transformPointCloud(*transformed_cloud, *transformed_cloud, transform_2);

	ss.str(string());  //clearing the contents of the stringstream variable
	ss<<path<<"/clouds/"<<cloud_name<<"_translated.pcd";
	writer.write(ss.str(),*transformed_cloud);*/

	
  


	/*ros::NodeHandle nh;

  	ros::Publisher pub;
  	pub=nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);

           	sensor_msgs::PointCloud2 msg;
           	msg.header.frame_id="base";
            msg.header.stamp = ros::Time::now();
            pcl::toROSMsg(*transformed_cloud,msg);*/
         

            /*int nr=-1;
            while(nr!=0 && ros::ok())
            {
            	msg.header.stamp=ros::Time::now();
            	pub.publish(msg);
            	ros::spinOnce();
            	cout<<"Enter option!"<<endl;
            	cin>>nr;

            }*/

            /*while(ros::ok())
            {
          	 msg.header.stamp = ros::Time::now();
             pub.publish (msg);
             ros::spinOnce ();
            	   pub.publish(msg);
            }*/

}


bool saver_service(pcd_cloud_saver::save::Request &req,pcd_cloud_saver::save::Response &res)
{
	cout<<"A request has been made!"<<endl;

	cout<<"Get gazebo model state!"<<endl;

	ros::NodeHandle nh;
	
	/*ros::ServiceClient serviceClient=nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	gazebo_msgs::GetModelState srv;
	srv.request.model_name="bulb_0";


	if(serviceClient.call(srv))
	{
		cout<<"Gazebo model has been acquired!"<<endl;
	}else
	{
		cout<<"Error while getting model state!"<<endl;
	}

	cout<<srv.response.pose.position.x<<" "<<srv.response.pose.position.y<<" "<<srv.response.pose.position.z<<endl;*/


	tf::TransformListener listener;
	tf::StampedTransform transform;

	for(int i=0;i<5;i++)
	{
		try{
			listener.lookupTransform("/base","/camera_rgb_optical_frame",ros::Time(0), transform);
		 //listener.waitForTransform("/base", "/camera_rgb_optical_frame",ros::Time::now(),ros::Duration(10.0));
    	//listener.lookupTransform("/base", "/camera_rgb_optical_frame",ros::Time::now(), transform);
     	}catch (tf::TransformException &ex) {

         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
  	 	}
  	}

  	/*tf::Transform model,transformed;
  	model.setOrigin(tf::Vector3(srv.response.pose.position.x,srv.response.pose.position.y,srv.response.pose.position.z));
  	model.setRotation(tf::Quaternion(srv.response.pose.orientation.w,srv.response.pose.orientation.x,
  		srv.response.pose.orientation.y,srv.response.pose.orientation.z));*/

    /*geometry_msgs::Pose bulb_pose;

    bulb_pose.position.x=0.66;
    bulb_pose.position.y=-0.21;
    bulb_pose.position.z=0.05;

    bulb_pose.orientation.w=1;
    bulb_pose.orientation.x=0;
    bulb_pose.orientation.y=0;
    bulb_pose.orientation.z=0;

    model.setOrigin(tf::Vector3(bulb_pose.position.x,bulb_pose.position.y,bulb_pose.position.z));
    model.setRotation(tf::Quaternion(bulb_pose.orientation.w,bulb_pose.orientation.x,
      bulb_pose.orientation.y,bulb_pose.orientation.z));


  	cout<<transform.getOrigin().x()<<endl;
  	transformed=transform.inverseTimes(model);

  	cout<<transform.getOrigin().x()<<endl;
  	cout<<transformed.getOrigin().x()<<endl;*/

  	Eigen::Matrix4f out_mat;

  	double mv[12];
  	//transformed.getBasis ().getOpenGLSubMatrix (mv);
  	transform.getBasis().getOpenGLSubMatrix(mv);
  	tf::Vector3 origin=transform.getOrigin();

  	//tf::Vector3 origin = transformed.getOrigin ();

  	out_mat (0, 0) = mv[0]; out_mat (0, 1) = mv[4]; out_mat (0, 2) = mv[8];
  	out_mat (1, 0) = mv[1]; out_mat (1, 1) = mv[5]; out_mat (1, 2) = mv[9];
  	out_mat (2, 0) = mv[2]; out_mat (2, 1) = mv[6]; out_mat (2, 2) = mv[10];
                                                                     
  	out_mat (3, 0) = out_mat (3, 1) = out_mat (3, 2) = 0; out_mat (3, 3) = 1;
  	out_mat (0, 3) = origin.x ();
  	out_mat (1, 3) = origin.y ();
	  out_mat (2, 3) = origin.z ();

	cout<<"Transformation matrix"<<endl;

	cout<<out_mat<<endl;

	geometry_msgs::Pose pose;

	/*pose.position.x=transform.getOrigin().x()-srv.response.pose.position.x;
	pose.position.y=transform.getOrigin().y()-srv.response.pose.position.y;
	pose.position.z=transform.getOrigin().z()-srv.response.pose.position.z;*/

  pose.position.x=transform.getOrigin().x();
  pose.position.y=transform.getOrigin().y();
  pose.position.z=transform.getOrigin().z();

	ros::Subscriber sub=nh.subscribe(TOPIC_NAME,100,callback);


	/*do
	{	
		ros::spinOnce();

	}while(cloud->empty());*/

	for(int i=0;i<5;i++)
	{
		ros::spinOnce();
		sleep(1.0);
	}


	cout<<"The cloud has :"<<cloud->points.size()<<endl;
	cout<<"Cloud name :"<<req.cloud_name.data<<endl;

	save_cloud(req.cloud_name.data,out_mat,pose);

	sub.shutdown();


	return true;
};


int main(int argc,char**argv)
{
	ros::init(argc,argv,"pcd_cloud_saver");

	ros::NodeHandle nh;

	ros::ServiceServer server=nh.advertiseService(SERVICE_NAME,saver_service);

	ROS_INFO("Point Cloud saving service up . . .");
	ros::spin();

	return 0;
}