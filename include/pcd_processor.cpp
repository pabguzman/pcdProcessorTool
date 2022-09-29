#include "pcd_processor.h"


/**
 * The constructor of the class PcdProcessor. It initializes the class attributes.
 * 
 * @param file_to_open The path to the file you want to open.
 * @param route_to_save The route where the processed file will be saved.
 */
PcdProcessor::PcdProcessor(std::string file_to_open, std::string route_to_save) 
    : file_opened_(file_to_open), 
      route_to_save_(route_to_save),
      original_cloud_(new PointCloudXYZ),
      cloud_processed_(new PointCloudXYZ),
      cloud_processed_with_normals_(new PointCloudNormal),
      polygon_mesh_(new PolyMesh),
      check_(true)
      
{
    if(pcl::io::loadPCDFile(file_opened_, *original_cloud_))
    {
        std::cout << "[ERROR] File or file path is corrupted. Please, enter a valid file path." << std::endl;
        check_ = false;

    }
    else
    {
        std::cout << "[INFO] File loaded successfully!" << std::endl;
        cloud_processed_ = original_cloud_;
        check_ = true;
    }
}

/**
 * It takes a point cloud and downsamples it by taking the average of the points in each voxel
 *
 * @param l the size of the voxel grid.
 */
void PcdProcessor::addVoxelGrid(float l)
{
    if (checkValidation())
    {
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_processed_);
        sor.setLeafSize(l, l, l);
        sor.filter(*cloud_processed_);
    }
}

/**
 * It removes outliers from the point cloud
 *
 * @param mean_k The number of neighbors to analyze for each point.
 * @param threshold The number of standard deviations for a point to be considered an outlier.
 */
void PcdProcessor::addOutliersFilter(int mean_k, float threshold)
{   
    if (checkValidation())
    {
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_processed_);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(threshold);
    sor.filter(*cloud_processed_);
    }
}

/**
 * It takes a point cloud, and smooths it out by fitting a polynomial to the points in the neighborhood of each point
 *
 * @param grade The order of the polynomial to fit.
 * @param radius The size of the neighborhood around a point in which the local surface is approximated using a polynomial.
 */
void PcdProcessor::addMlsFilter(int grade, float radius)
{
    if (checkValidation())
    {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_processed_);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> mls;

    mls.setComputeNormals(true);

    // Set parameters
    mls.setInputCloud(cloud_processed_);
    mls.setPolynomialOrder(grade);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(radius);

    // Reconstruct
    mls.process(*cloud_processed_);
    }
}

/**
 * It takes a point cloud, estimates normals, and then uses the greedy projection triangulation algorithm to create a mesh
 *
 * @param npoints The number of points to use for the nearest neighbor search.
 * @param rad The maximum distance between connected points (maximum edge length)
 * @param k_search_norm The number of nearest neighbors to use for normal estimation.
 */
void PcdProcessor::makeTriangulation(int npoints, float rad, int k_search_norm)
{
    if (checkValidation())
    {
    makeNormalEstimation(k_search_norm);
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);

    tree->setInputCloud(cloud_processed_with_normals_);

    // Init object (second point type is for the normals, even if unused)
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius(rad);

    // Set typical values for the parameters
    gp3.setMu(2.5);
    gp3.setMaximumNearestNeighbors(npoints);
    gp3.setMaximumSurfaceAngle(M_PI / 4); // 45 degrees
    gp3.setMinimumAngle(M_PI / 18);       // 10 degrees
    gp3.setMaximumAngle(2 * M_PI / 3);    // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud(cloud_processed_with_normals_);
    gp3.setSearchMethod(tree);
    gp3.reconstruct(*polygon_mesh_);
    }
}

/**
 * It saves the processed cloud in the indicated route
 *
 * @param name Name of the file to be saved.
 */
void PcdProcessor::savePCD(std::string name)
{
    if (checkValidation())
    {
    if (pcl::io::savePCDFile(route_to_save_ + name + ".pcd", *cloud_processed_))
    {
        std::cout << "[ERROR] It is not possible to create the PCD file." << std::endl;
    }
    else
    {
        std::cout << "[INFO] PCD processed saved to the indicated path" << std::endl;
    }
    }
}

/**
 * It saves the STL file in the indicated path
 *
 * @param name Name of the file to be saved.
 */
void PcdProcessor::saveSTL(std::string name)
{
    if (checkValidation())
    {
    if (pcl::io::savePolygonFileSTL(route_to_save_ + name + ".ply", *polygon_mesh_))
    {
        std::cout << "[ERROR] It is not possible to create the STL file" << std::endl;
    }
    else
    {
        std::cout << "[INFO] STL processed saved to the indicated path" << std::endl;
    }
    }
}

std::string PcdProcessor::getFileOpened()
{
    return file_opened_;
}

std::string PcdProcessor::getRouteToSave()
{
    return route_to_save_;
    
}

bool PcdProcessor::checkValidation()
{
    return check_;
}

/**
 * It takes a point cloud, and for each point in the cloud, it finds the k nearest neighbors, and then computes the normal
 * vector for the point based on the k nearest neighbors
 *
 * @param k_search The number of nearest neighbors to use for the estimation.
 */
void PcdProcessor::makeNormalEstimation(int k_search)
{
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_processed_);

    // Init object (second point type is for the normals, even if unused)
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;

    // Set parameters
    n.setInputCloud(cloud_processed_);
    n.setSearchMethod(tree);
    n.setKSearch(k_search);

    n.compute(*normals);
    pcl::concatenateFields(*cloud_processed_, *normals, *cloud_processed_with_normals_);
}

/**
 * It sets the route to save the processed point cloud
 *
 * @param param the parameter to be set
 */

void PcdProcessor::setFileToOpen(std::string param)
{
    file_opened_ = param;
}

/**
 * It sets the route to save the processed point cloud
 *
 * @param param the parameter to be set
 */
void PcdProcessor::setRouteToSave(std::string param)
{
    route_to_save_ = param;
}

/**
 * It resets all the variables to their initial state
 */
void PcdProcessor::restartProcessor()
{
    check_ = false;
    original_cloud_ = PointCloudXYZPtr(new PointCloudXYZ);
    cloud_processed_ = PointCloudXYZPtr(new PointCloudXYZ);
    cloud_processed_with_normals_ = PointCloudNormalPtr(new PointCloudNormal);
    polygon_mesh_ = PolyMeshPtr(new PolyMesh);

    if(pcl::io::loadPCDFile(file_opened_, *original_cloud_))
    {
        std::cout << "[ERROR] File or file path is corrupted. Please, enter a valid file path." << std::endl;
        check_ = false;

    }
    else
    {
        std::cout << "[INFO] File loaded successfully!" << std::endl;
        cloud_processed_ = original_cloud_;
        check_ = true;
    }
}