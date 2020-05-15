// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <unordered_set>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include "kdtree.h"

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};


//Quote kudos to https://stackoverflow.com/questions/6942273/how-to-get-a-random-element-from-a-c-container
#include  <random>
#include  <iterator>

template<typename Iter, typename RandomGenerator>
Iter select_randomly(Iter start, Iter end, RandomGenerator& g) {
    std::uniform_int_distribution<> dis(0, std::distance(start, end) - 1);
    std::advance(start, dis(g));
    return start;
}

template<typename Iter>
Iter select_randomly(Iter start, Iter end) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return select_randomly(start, end, gen);
}

//end Quote
template<typename PointT>
std::unordered_set<int> RansacPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult;
    srand(time(NULL));
    // TODO: Fill in this function
    
    // For max iterations 
    maxIterations = maxIterations;
    for(int i=0; i<maxIterations; i++)
    {
        std::unordered_set<int> inliers;
        float x1,x2,x3,y1,y2,y3,z1,z2,z3;
        float A,B,C,D,div;
        
        // Randomly sample subset and fit line
        while(inliers.size() < 3)
        {
            inliers.insert(std::distance(std::begin(cloud->points),select_randomly(std::begin(cloud->points), std::end(cloud->points))));
        }
      
        {auto itr = std::begin(inliers);

        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr ++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr ++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;
        }
        
        A = ((y2-y1)*(z3-z1))-((z2-z1)*(y3-y1));
        B = ((z2-z1)*(x3-x1))-((x2-x1)*(z3-z1));
        C = ((x2-x1)*(y3-y1))-((y2-y1)*(x3-x1));
        D = -1 * (A*x1+B*y1+C*z1);
        div = sqrt((A*A) + (B*B) + (C*C));
           
        // Measure distance between every point and fitted plane
        auto distance_plane
        {
            [=](PointT v) {
                return fabs((A*v.x) + (B*v.y) + (C*v.z) + D)/div <= distanceTol;
            }
        };
        
        // If distance is smaller than threshold count it as inlier
        // Kudos to https://stackoverflow.com/questions/12990148/get-all-positions-of-elements-in-stl-vector-that-are-greater-than-a-value
        auto iterator = std::find_if(std::begin(cloud->points), std::end(cloud->points), distance_plane);
        while (iterator != std::end(cloud->points)) {
            inliers.insert(std::distance(std::begin(cloud->points), iterator));
            iterator = std::find_if(std::next(iterator), std::end(cloud->points), distance_plane);
        }
        // Return indicies of inliers from fitted line with most inliers

        if(inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }
    return inliersResult;

}

template<typename PointT>
std::vector<std::vector<int>> euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol)
{
    // TODO: Fill out this function to return list of indices for each cluster
    std::vector<std::vector<int>> clusters;
    std::vector<int> clusterIdx(cloud->points.size(),0);
    
    for(int pId=0; pId<clusterIdx.size(); pId++)
    {
        if(0 == clusterIdx[pId])
        {
            std::vector<float> point = {cloud->points[pId].x,cloud->points[pId].y,cloud->points[pId].z};
            auto neighbors = tree->search(point,distanceTol);
            if(neighbors.size() > 0)
            {
                std::vector<int> cluster;
                cluster.push_back(pId);
                clusters.push_back(cluster);
                clusterIdx[pId] = clusters.size();
                //don't remove anything from neighbors vector
                for (int i = 0; i < neighbors.size(); ++i)
                {
                    int point_idx = neighbors[i];
                    if(0 == clusterIdx[point_idx])//unprocessed neighbor
                    {
                        clusters[clusterIdx[pId]-1].push_back(point_idx);
                        clusterIdx[point_idx] = clusterIdx[pId];
                        //find new neighbors and add them to the list
                        std::vector<float> neighbor_point = {cloud->points[point_idx].x,cloud->points[point_idx].y,cloud->points[point_idx].z};
                        auto new_neighbors = tree->search(neighbor_point,distanceTol);
                        for(int new_neighbor_idx: new_neighbors)
                        {
                            if(0 == clusterIdx[new_neighbor_idx])//unprocessed neighbor neighbor
                            {
                                neighbors.push_back(new_neighbor_idx);
                            }
                        }
                    }
                }
                /*
                //Any clusters nearby?
                auto neighbor_in_cluster = std::find_if(std::begin(neighbors), std::end(neighbors), [&](int id){return clusterIdx[id]>0;});

                //Add me to neighbor's cluster. Otherwise, create my own
                if(neighbor_in_cluster != std::end(neighbors))
                {
                    int clusterId = clusterIdx[*neighbor_in_cluster];
                    clusters[clusterId-1].push_back(pId);
                }
                else
                {
                    std::vector<int> cluster;
                    cluster.push_back(pId);
                    clusters.push_back(cluster);
                    clusterIdx[pId] = clusters.size();
                }*/
            }
        }
    }
    
    return clusters;
}
#endif /* PROCESSPOINTCLOUDS_H_ */