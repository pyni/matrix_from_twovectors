#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>
#include <boost/thread/thread.hpp>
#include <pcl/ModelCoefficients.h> 
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h> 

#include <pcl/common/transforms.h>    



void CrossProduct(double a[3], double b[3], double ret[3])
{
    ret[0] = a[1] * b[2] - a[2] * b[1];
    ret[1] = a[2] * b[0] - a[0] * b[2];
    ret[2] = a[0] * b[1] - a[1] * b[0];
}

double DotProduct(double a[3], double b[3])
{
    double result;
    result = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];

    return result;
}

double Normalize(double v[3])
{
    double result;

    result = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);

    return result;
}

void Rotate(double** rotateMatrix, double u[3], double ret[3]){
    ret[0]=rotateMatrix[0][0]*u[0]+rotateMatrix[0][1]*u[1]+rotateMatrix[0][2]*u[2];
    ret[1]=rotateMatrix[1][0]*u[0]+rotateMatrix[1][1]*u[1]+rotateMatrix[1][2]*u[2];
    ret[2]=rotateMatrix[2][0]*u[0]+rotateMatrix[2][1]*u[1]+rotateMatrix[2][2]*u[2];
}

void RotationMatrix(double angle, double u[3], double rotatinMatrix[3][3])
{
    double norm = Normalize(u);
    
    u[0] = u[0] / norm;
    u[1] = u[1] / norm;
    u[2] = u[2] / norm;

    rotatinMatrix[0][0] = cos(angle) + u[0] * u[0] * (1 - cos(angle));
    rotatinMatrix[0][1] = u[0] * u[1] * (1 - cos(angle) - u[2] * sin(angle));
    rotatinMatrix[0][2] = u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));

    rotatinMatrix[1][0] = u[2] * sin(angle) + u[0] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][1] = cos(angle) + u[1] * u[1] * (1 - cos(angle));
    rotatinMatrix[1][2] = -u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
      
    rotatinMatrix[2][0] = -u[1] * sin(angle) + u[0] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][1] = u[0] * sin(angle) + u[1] * u[2] * (1 - cos(angle));
    rotatinMatrix[2][2] = cos(angle) + u[2] * u[2] * (1 - cos(angle));

}

//cal transport
void Calculation3d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[3][3])
{
    double  rotationAxis[3];
    double rotationAngle;
    CrossProduct(vectorBefore, vectorAfter, rotationAxis);
    rotationAngle = acos(DotProduct(vectorBefore, vectorAfter) / Normalize(vectorBefore) / Normalize(vectorAfter));
    RotationMatrix(rotationAngle, rotationAxis, rotatinMatrix);
}

void Calculation4d(double vectorBefore[3], double vectorAfter[3], double rotatinMatrix[4][4])
{
    double rotate3d[3][3];
	Calculation3d(vectorBefore,vectorAfter, rotate3d);

	for(int i = 0; i < 3; i++)
		for(int j = 0; j < 3; j++)
			rotatinMatrix[i][j] = rotate3d[i][j];

	for(int i = 0; i < 3; i++)
	{
		rotatinMatrix[i][3] = 0;
		rotatinMatrix[3][i] = 0;
	}

	rotatinMatrix[3][3] = 1;
}













int
main (int argc, char** argv)
{
//superparameters:
int samplenum=200;
int suctionradius=0.015;





  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZ>);


  if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/yuan/doc/suction_ws/src/gqcnn-dev_jeff/output.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

    // 法线对象
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
     // 法线估计对象
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    normalEstimation.setInputCloud(cloud);
    // 法线估计方法: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT.
    
    normalEstimation.setNormalEstimationMethod(normalEstimation.AVERAGE_3D_GRADIENT);
    //设置深度变化的阀值
    normalEstimation.setMaxDepthChangeFactor(0.02f);
    // 设置计算法线的区域
    normalEstimation.setNormalSmoothingSize(20.0f);

    // 计算
    normalEstimation.compute(*normals);

    // 可视化
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Normals"));
   // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud3");



int Low=0;
int High=normals->size( );
int rannum;	
std::vector<int> indexnum   ;
std::vector<pcl::Normal> randpoints   ;
pcl::Normal nannormal;
for(int i=0;i<samplenum;i++)//samplenum    normals->size()
{
rannum=Low + rand() % (High - Low + 1) ;
while(isnan(normals->points[rannum].normal_x)==1)
{
//pcl::PointXYZ testcloud;
nannormal= normals->points[rannum] ;
rannum=Low + rand() % (High - Low + 1) ;//rando a num 
} 

indexnum.push_back(rannum); 
randpoints.push_back(normals->points[rannum]); 


std::cout << rannum <<normals->points[rannum] <<std::endl;
}


double vectorBefore[3]  = {normals->points[indexnum[0]].normal_x,normals->points[indexnum[0]].normal_y,normals->points[indexnum[0]].normal_z};
double vectorAfter[3] = {0,0,-1};
double rotatinMatrix[4][4];
Calculation4d(  vectorBefore , vectorAfter ,   rotatinMatrix );
std::cout << vectorBefore[0]<<"," << vectorBefore[1]<<"," << vectorBefore[2]<<std::endl;
std::cout << rotatinMatrix[0][0]<<"," << rotatinMatrix[0][1]<<"," << rotatinMatrix[0][2]<<"," << rotatinMatrix[0][3]<<std::endl;
std::cout << rotatinMatrix[1][0]<<"," << rotatinMatrix[1][1]<<"," << rotatinMatrix[1][2]<<"," << rotatinMatrix[1][3]<<std::endl;
std::cout << rotatinMatrix[2][0]<<"," << rotatinMatrix[2][1]<<"," << rotatinMatrix[2][2]<<"," << rotatinMatrix[2][3]<<std::endl;
std::cout << rotatinMatrix[3][0]<<"," << rotatinMatrix[3][1]<<"," << rotatinMatrix[3][2]<<"," << rotatinMatrix[3][3]<<std::endl;
//上面是为了采样，即对所有点进行随机采样
//下面是对每个采样点沿着法线进行8点投影







Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

// Define a rotation matrix (see https://en.wikipedia.org/wiki/Rotation_matrix)
// Here we defined a 45° (PI/4) rotation around the Z axis and a translation on the X axis. 
transform_1 (0,0) = rotatinMatrix[0][0];
transform_1 (0,1) = rotatinMatrix[0][1];
transform_1 (0,2) = rotatinMatrix[0][2];
transform_1 (0,3) =rotatinMatrix[0][3];


transform_1 (1,0) = rotatinMatrix[1][0];
transform_1 (1,1) = rotatinMatrix[1][1];
transform_1 (1,2) = rotatinMatrix[1][2];
transform_1 (1,3) =rotatinMatrix[1][3];


transform_1 (2,0) = rotatinMatrix[2][0];
transform_1 (2,1) = rotatinMatrix[2][1];
transform_1 (2,2) = rotatinMatrix[2][2];
transform_1 (2,3) =rotatinMatrix[2][3];

transform_1 (3,0) = rotatinMatrix[3][0];
transform_1 (3,1) = rotatinMatrix[3][1];
transform_1 (3,2) = rotatinMatrix[3][2];
transform_1 (3,3) =rotatinMatrix[3][3];

//    (row, column)

// Define a translation of 2.5 meters on the x axis.
 
// Print the transformation
printf ("Method #1: using a Matrix4f\n");
std::cout << transform_1 << std::endl;


// Executing the transformation
pcl::PointCloud<pcl::PointXYZ >::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ > ());
/*
void pcl::transformPointCloud(const pcl::PointCloud< PointT > & cloud_in, 
                            pcl::PointCloud< PointT > &  cloud_out,  
                            const Eigen::Matrix4f &  transform  ) 
*/
// Apply an affine transform defined by an Eigen Transform.
pcl::transformPointCloud (*cloud, *transformed_cloud, transform_1);


//这里需要重新求变化后点云的法线!！!！才能看出来


    // 法线对象
    pcl::PointCloud<pcl::Normal>::Ptr newnormals(new pcl::PointCloud<pcl::Normal>); 
    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation2;
    normalEstimation2.setInputCloud(transformed_cloud);
    // 法线估计方法: COVARIANCE_MATRIX, AVERAGE_DEPTH_CHANGE, SIMPLE_3D_GRADIENT.
    
    normalEstimation2.setNormalEstimationMethod(normalEstimation.AVERAGE_3D_GRADIENT);
    //设置深度变化的阀值
    normalEstimation2.setMaxDepthChangeFactor(0.02f);
    // 设置计算法线的区域
    normalEstimation2.setNormalSmoothingSize(20.0f);

    // 计算
    normalEstimation2.compute(*newnormals);
 

for(int i=0;i<newnormals->size();i++)//samplenum    normals->size()
{
if (  i ==indexnum[0]  ) //and i>indexnum[0]-520) 除了要看的vector,其余vector设为0
{ 
std::cout << newnormals->points[i].normal_x  << ","<< newnormals->points[i].normal_y<< ","  <<  newnormals->points[i].normal_z<<std::endl; 

}
else
{
newnormals->points[i].normal_x=0;//nannormal.normal_x;
newnormals->points[i].normal_y=0;//nannormal.normal_y;
newnormals->points[i].normal_z=0;//nannormal.normal_z;
}

}
//pcl::Normal testnormal;
//pcl::PointXYZ testcloud;
//testnormal=normals->points[rannum];
//normals->points.clear();
// normals->points.push_back(testnormal);

//testcloud=cloud->points[rannum];
 //cloud->points.clear();
 //cloud->points.push_back(testcloud);
 
//pcl::PointCloud<pcl::PointXYZ>::Ptr testnormal (new pcl::PointCloud<pcl::PointXYZ>);
//testnormal->points.push_back (normals->points[rannum]); 

viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud, "cloud");
viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(transformed_cloud, newnormals, 1, 0.83, "normals");//(这里，每 10 个点显示一个)及每个法线的长度 
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.1, 0.6,0.6, "normals");

viewer->addCoordinateSystem (0.1);
 
while (!viewer->wasStopped())
{
viewer->spinOnce(100);
boost::this_thread::sleep(boost::posix_time::microseconds(100000));
}



  // draw the samples
  //viewer->addPointCloud<pcl::PointXYZ>(filtered_cloud2, "samples cloud3");
 // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "samples cloud3");
//  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0,1.0, "samples cloud3");
//viewer->addCoordinateSystem (0.1);
//while (!viewer->wasStopped ())
 // {
 //   viewer->spinOnce (100);
 //   boost::this_thread::sleep (boost::posix_time::microseconds (100000));
 // }



}
