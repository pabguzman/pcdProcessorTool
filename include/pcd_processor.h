#ifndef PCD_PROCESSOR_H
#define PCD_PROCESSOR_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <boost/algorithm/string.hpp>

class PcdProcessor
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
    typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudXYZPtr;
    typedef pcl::PointCloud<pcl::PointNormal> PointCloudNormal;
    typedef pcl::PointCloud<pcl::PointNormal>::Ptr PointCloudNormalPtr;
    typedef pcl::PolygonMesh PolyMesh;
    typedef pcl::PolygonMesh::Ptr PolyMeshPtr;

    
public:
    explicit PcdProcessor(std::string file_to_open, 
                          std::string route_to_save);

    void addVoxelGrid(float l);
    void addOutliersFilter(int mean_k, float threshold);
    void addMlsFilter(int grade, float radius);
    void makeTriangulation(int npoints, float rad, int k_search_norm);

    void savePCD(std::string name);
    void saveSTL(std::string name);

    void setFileToOpen(std::string param);
    std::string getFileOpened(); 
    void setRouteToSave(std::string param);
    std::string getRouteToSave(); 
    void restartProcessor();
    bool checkValidation();

private:

    bool check_;
    std::string file_opened_;
    std::string route_to_save_;
    PointCloudXYZPtr original_cloud_;
    PointCloudXYZPtr cloud_processed_;
    PointCloudNormalPtr cloud_processed_with_normals_;
    PolyMeshPtr polygon_mesh_;

    void makeNormalEstimation(int k_search);

};

#endif // PCD_PROCESSOR_H