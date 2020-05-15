// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered(new pcl::PointCloud<PointT>);
    
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes,filterRes,filterRes);
    sor.filter (*cloud_filtered);
    typename pcl::PointCloud<PointT>::Ptr cloud_region(new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud (cloud_filtered);
    region.filter (*cloud_region);
  
    typename pcl::PointCloud<PointT>::Ptr cloud_region_roof(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> roof(true);
    region.setMin(Eigen::Vector4f (-1.5,-1.7,-1,1));
    region.setMax(Eigen::Vector4f (2.6,1.7,-0.4,1));
    region.setNegative(true);
    region.setInputCloud (cloud_region);
    region.filter (*cloud_region_roof);

    /*pcl::PassThrough<PointT> pass;
    pass.setInputCloud (cloud_filtered);
    pass.setFilterFieldName ("region");
    pass.setFilterLimits (minPoint, maxPoint);
    //pass.setFilterLimitsNegative (true);
    pass.filter (*cloud_region);*/
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region_roof;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
  
    extract.setNegative (false);
    // Get the points associated with the planar surface
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());
    extract.filter (*planeCloud);
  
    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    extract.filter (*obstCloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // TODO:: Fill in this function to find inliers for the cloud.
    /*
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    // Create the segObjectmentation object
    pcl::SACSegmentation<PointT> segObject;
    // Optional
    segObject.setOptimizeCoefficients (true);
    // Mandatory
    segObject.setModelType (pcl::SACMODEL_PLANE);
    segObject.setMethodType (pcl::SAC_RANSAC);
    segObject.setDistanceThreshold (distanceThreshold);
    segObject.setMaxIterations (maxIterations);

    segObject.setInputCloud (cloud);
    segObject.segment (*inliers, *coefficients);
    std::cout << coefficients;*/
    auto ransac_pts = RansacPlane<PointT>(cloud, maxIterations, distanceThreshold);
    inliers->indices.insert(end(inliers->indices), begin(ransac_pts),end(ransac_pts));
    //std::cout << inliers->indices.size() << " copied out of " << ransac_pts.size() << std::endl;
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
/*
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance);
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract(cluster_indices);
    
    for (auto& indices : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr ccloud(new pcl::PointCloud<PointT>);
        ccloud->is_dense = true;
        ccloud->height = 1;
        std::transform ( std::begin(indices.indices), std::end(indices.indices), std::back_inserter(ccloud->points), [&](int v){return cloud->points[v];});
        clusters.push_back(ccloud);
    }*/
    
    KdTree* tree = new KdTree;
    for (int i=0; i<cloud->points.size(); i++) 
    {
        std::vector<float> point = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
        tree->insert(point,i);
    }
    //minSize and maxSize not supported yet
    std::vector<std::vector<int>> cluster_indices = euclideanCluster<PointT>(cloud, tree, clusterTolerance);
    std::cout << "euclideanCluster found " << cluster_indices.size() << " clusters" << std::endl;
    for (std::vector<int> indices : cluster_indices)
    {
        if(indices.size() > minSize)
        {
        typename pcl::PointCloud<PointT>::Ptr ccloud(new pcl::PointCloud<PointT>());
        ccloud->is_dense = true;
        ccloud->height = 1;
        std::transform ( std::begin(indices), std::end(indices), std::back_inserter(ccloud->points), [&](int v){return cloud->points[v];} );
        
        clusters.push_back(ccloud);
        }
    }
    delete [] tree;
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}