#define PCL_NO_PRECOMPILE

#include <boost/filesystem/operations.hpp>
#include <experimental/filesystem>
#include <iostream>
#include <fstream>

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

struct MyPointType
{
  float x;
  float y;
  float z;
  float occupied;
  float free;
  float unknown;
  float conflict;
  PCL_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (MyPointType,           
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, occupied, occupied)
				                   (float, free, free)
				                   (float, unknown, unknown)
				                   (float, conflict, conflict)
)

#include <cstring>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <ctime>

using namespace std;
using namespace pcl;
using namespace pcl::io;

#include <octomap/octomap.h>
#include <pcl/octree/octree_pointcloud.h>


// HELPERS =========================================================================================

int PclArrayToPointCloud(float* points, int nPoints, PointCloud<MyPointType>& cloud)
{

  for (int i = 0; i < nPoints; i++){
    MyPointType pt = {points[7*i+0], points[7*i+1], points[7*i+2], points[7*i+3], points[7*i+4], points[7*i+5], points[7*i+6]};
    cloud.push_back(pt);
  }
   // cloud.push_back(MyPointType(points[7*i+0], points[7*i+1], points[7*i+2], points[7*i+3], points[7*i+4], points[7*i+5], points[7*i+6]));

  return 0;
}

int PclPointCloudToNewArray(PointCloud<MyPointType>& cloud, float** ppoints, int* nPoints)
{
  *nPoints = cloud.size();
  float* points = new float[*nPoints * 7];
  *ppoints = points;

  if(cloud[0].occupied == NULL){
      for (int i = 0; i < *nPoints; i++)
        {
          points[7*i+0] = cloud[i].x;
          points[7*i+1] = cloud[i].y;
          points[7*i+2] = cloud[i].z;
          points[7*i+3] = 0.8;
          points[7*i+4] = 0.01;
          points[7*i+5] = 0.19;
          points[7*i+6] = 0;
        }
  }
  else{
      for (int i = 0; i < *nPoints; i++)
        {
            points[7*i+0] = cloud[i].x;
            points[7*i+1] = cloud[i].y;
            points[7*i+2] = cloud[i].z;
            points[7*i+3] = cloud[i].occupied;
            points[7*i+4] = cloud[i].free;
            points[7*i+5] = cloud[i].unknown;
            points[7*i+6] = cloud[i].conflict;
        }
  }

  return 0;
}

int PclPointCloudToNewArrayPtr(PointCloud<MyPointType>::Ptr& cloud, float** ppoints, int* nPoints)
{
  *nPoints = cloud->size();
  float* points = new float[*nPoints * 7];
  *ppoints = points;

  if((*cloud)[0].occupied == NULL){
      std::cout <<"i'm in null"<<std::endl;
      for (int i = 0; i < *nPoints; i++)
        {
          points[7*i+0] = (*cloud)[i].x;
          points[7*i+1] = (*cloud)[i].y;
          points[7*i+2] = (*cloud)[i].z;
          points[7*i+3] = 0.8;
          points[7*i+4] = 0.01;
          points[7*i+5] = 0.19;
          points[7*i+6] = 0;
        }
  }
  else{
      std::cout <<"i'm in NOTnull"<<std::endl;
      for (int i = 0; i < *nPoints; i++)
        {
            points[7*i+0] = (*cloud)[i].x;
            points[7*i+1] = (*cloud)[i].y;
            points[7*i+2] = (*cloud)[i].z;
            points[7*i+3] = (*cloud)[i].occupied;
            points[7*i+4] = (*cloud)[i].free;
            points[7*i+5] = (*cloud)[i].unknown;
            points[7*i+6] = (*cloud)[i].conflict;
        }
  }

  return 0;
}

int PclArrayToPointCloudPtr(float* points, int nPoints, PointCloud<MyPointType>::Ptr& cloud)
{
  for (int i = 0; i < nPoints; i++){
    MyPointType pt = {points[7*i+0], points[7*i+1], points[7*i+2], points[7*i+3], points[7*i+4], points[7*i+5], points[7*i+6]};
    cloud->push_back(pt);
  }

  return 0;
}

// EXTERN ==========================================================================================

extern "C" int CopyAndFree(float* in, float* out, int nPoints)
{
  std::cout << "copy here" << std::endl;
  memcpy(out, in, sizeof(float)*nPoints*7);
  delete[] in;
  return 0;
}

extern "C" int PclSavePcd(char* fileName, float* points, int nPoints)
{
  PointCloud<MyPointType> cloud;
  PclArrayToPointCloud(points, nPoints, cloud);

  if (savePCDFileASCII(fileName, cloud) < 0)
    return -1;

  return 0;
}

extern "C" int PclLoadPcd(char* fileName, float** ppoints, int* nPoints)
{
  PointCloud<MyPointType> cloud;
  if (loadPCDFile<MyPointType>(fileName, cloud) < 0)
    return -1;
  PclPointCloudToNewArray(cloud, ppoints, nPoints);
  return 0;
}

extern "C" int ConvertPcdToOctomap(char* input_file, char* output_file)
{
  PointCloud<PointXYZ> cloud;
  loadPCDFile<PointXYZ> ( input_file, cloud );

  cout<<"point cloud loaded, point size = "<<cloud.points.size()<<endl;

   
  cout<<"copy data into octomap..."<<endl;

  octomap::OcTree tree( 0.05 );

  for (auto p:cloud.points)
  {     
    tree.updateNode( octomap::point3d(p.x, p.y, p.z), true );
  }

    
  tree.updateInnerOccupancy();
    
  tree.writeBinary( output_file );
  cout<<"done."<<endl;

  return 0;
}

int PclArrayToPointCloudXYZ(float* points, int nPoints, PointCloud<pcl::PointXYZ>& cloud)
{

  for (int i = 0; i < nPoints; i++){
    pcl::PointXYZ pt = {points[3*i+0], points[3*i+1], points[3*i+2]};
    cloud.push_back(pt);
  }
   // cloud.push_back(MyPointType(points[7*i+0], points[7*i+1], points[7*i+2], points[7*i+3], points[7*i+4], points[7*i+5], points[7*i+6]));

  return 0;
}


int PclPointCloudXYZToNewArray(PointCloud<pcl::PointXYZ>& cloud, PointCloud<MyPointType>& final_cloud, float** ppoints, int* nPoints)
{
  std::cout << final_cloud.size() << std::endl;

  *nPoints = final_cloud.size();
  float* points = new float[*nPoints * 7];
  *ppoints = points;

  for (int i = 0; i < *nPoints; i++)
    {
      points[7*i+0] = cloud[i].x;
      points[7*i+1] = cloud[i].y;
      points[7*i+2] = cloud[i].z;
      points[7*i+3] = final_cloud[i].occupied;
      points[7*i+4] = final_cloud[i].free;
      points[7*i+5] = final_cloud[i].unknown;
      points[7*i+6] = final_cloud[i].conflict;
    }

  return 0;
}

MyPointType DS_comb(MyPointType pt_1, MyPointType pt_2){
        MyPointType pt_new;
        pt_new.x = pt_2.x;
        pt_new.y = pt_2.y;
        pt_new.z = pt_2.z;

        float new_occ = pt_1.occupied * pt_2.occupied + pt_1.occupied * pt_2.unknown
                        + pt_1.unknown * pt_2.occupied;
        float new_free = pt_1.free * pt_2.free + pt_1.free * pt_2.unknown + pt_1.unknown * pt_2.free;
        float new_unk = pt_1.unknown * pt_2.unknown;
        float new_conflict = 0;
        float empty = pt_1.occupied * pt_2.free + pt_1.free * pt_2.occupied;
        float koeff = 1 - empty;

        pt_new.occupied = new_occ/koeff;
        pt_new.free = new_free/koeff;
        pt_new.unknown = new_unk/koeff;
        pt_new.conflict = new_conflict/koeff;

        return pt_new;
}

void pairAlign(const PointCloud<PointXYZ>::Ptr cloud_src, const PointCloud<PointXYZ>::Ptr cloud_tgt, const PointCloud<PointXYZ>::Ptr output, Eigen::Matrix4f& final_transform, int iter){
    std::cout << iter << std::endl;
    PointCloud<PointXYZ>::Ptr src(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr tgt(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr reg_result(new PointCloud<PointXYZ>);

    pcl::VoxelGrid<PointXYZ> grid;
    grid.setLeafSize (1.5, 1.5, 1.5);
    grid.setInputCloud (cloud_src);
    grid.filter (*src);

    grid.setInputCloud (cloud_tgt);
    grid.filter (*tgt);

    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), targetToSource;

    pcl::IterativeClosestPoint<PointXYZ, PointXYZ> reg;
    reg.setInputSource (src);
    reg.setInputTarget (tgt);
    reg.align(*reg_result);
    Ti = reg.getFinalTransformation();

    targetToSource = Ti.inverse();
    pcl::transformPointCloud(*tgt, *output, targetToSource);
    *output = *reg_result;

    final_transform = targetToSource
}


extern "C" int ICP_main(char* oct_file, char* dirname, int res, float** ppoints, int* nPoints){
    PointCloud<PointXYZ>::Ptr result(new PointCloud<PointXYZ>), src(new PointCloud<PointXYZ>), tgt(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr temp(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ>::Ptr FinalCloud(new PointCloud<PointXYZ>);
    PointCloud<MyPointType>::Ptr tgt_full(new PointCloud<MyPointType>);

    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

    //initial cloud to octree
    octree::OctreePointCloudSearch<MyPointType> octree(1.5);
    PointCloud<MyPointType>::Ptr oct_cloud(new PointCloud<MyPointType>);

    loadPCDFile<MyPointType> (oct_file, *oct_cloud);
    octree.setInputCloud (oct_cloud);
    octree.addPointsFromInputCloud();

    int iter = 0;

    std::vector<boost::filesystem::path> files_in_directory;
    std::copy(boost::filesystem::directory_iterator(dirname), boost::filesystem::directory_iterator(), std::back_inserter(files_in_directory));
    std::sort(files_in_directory.begin(), files_in_directory.end());
    std::vector<boost::filesystem::path>::iterator it;
    boost::filesystem::directory_iterator end;

    int count = 0;

    for (it = files_in_directory.begin(); it != --files_in_directory.end();) {
        loadPCDFile<PointXYZ> (boost::filesystem::canonical(*it).string(), *src);
        ++it;
        loadPCDFile<PointXYZ> (boost::filesystem::canonical(*it).string(), *tgt);
        loadPCDFile<MyPointType> (boost::filesystem::canonical(*it).string(), *tgt_full);
        pairAlign(src, tgt, temp, pairTransform, iter);
        pcl::transformPointCloud(*temp, *result, GlobalTransform);
        GlobalTransform = pairTransform * GlobalTransform;
//        std::cout << iter++ << ", Final GlobalTrans: \n" << GlobalTransform <<std::endl;
        std::cout << "iter: " << iter++ <<std::endl;

        //add to octree
    for (auto p:tgt_full->points)
      {
        MyPointType searchPoint;
        searchPoint.x = p.x;
        searchPoint.y = p.y;
        searchPoint.z = p.z;
        searchPoint.occupied = p.occupied;
        searchPoint.free = p.free;
        searchPoint.unknown = p.unknown;
        searchPoint.conflict = p.conflict;

        if(!octree.isVoxelOccupiedAtPoint(searchPoint)){
          octree.addPointToCloud(searchPoint, oct_cloud);
        }
        else{
          std::vector<int> pointIdxVec;
          if(octree.voxelSearch(searchPoint, pointIdxVec)){
                std::cout << "Neighbors within voxel search at (" << searchPoint.x
                 << " " << searchPoint.y
                 << " " << searchPoint.z
                 << " " << searchPoint.occupied
                 << ")"
                 << std::endl;
                std::cout << "pointIdxVec size = " << pointIdxVec.size () << std::endl;
                  MyPointType octPoint;
                  octPoint.x = oct_cloud->points[pointIdxVec[i]].x;
                  octPoint.y = oct_cloud->points[pointIdxVec[i]].y;
                  octPoint.z = oct_cloud->points[pointIdxVec[i]].z;
                  octPoint.occupied  = oct_cloud->points[pointIdxVec[i]].occupied;
                  octPoint.free  = oct_cloud->points[pointIdxVec[i]].free;
                  octPoint.unknown  = oct_cloud->points[pointIdxVec[i]].unknown;
                  octPoint.conflict  = oct_cloud->points[pointIdxVec[i]].conflict;
    //                   std::cout << octPoint.x << " " << octPoint.y << " " << octPoint.z << " " <<
    //                   octPoint.occupied << " "  << octPoint.empty << std::endl;

                  MyPointType DSPoint = DS_comb(searchPoint, octPoint);
                  octree.deleteVoxelAtPoint(octPoint);
                  octree.addPointToCloud(DSPoint, oct_cloud);
            }
        }
      }
      int nPointOct = (*oct_cloud).size();
      std::cout << "octree size: " << nPointOct << std::endl;
      *FinalCloud += *result;
    }

    PclPointCloudToNewArrayPtr(oct_cloud, ppoints, nPoints);

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0,0,0);
    viewer->addPointCloud<pcl::PointXYZ> (FinalCloud, "cloud1");

    viewer->addCoordinateSystem(1.0);
       while(!viewer->wasStopped())
       {
          viewer->spinOnce();
       }
    return 0;
}