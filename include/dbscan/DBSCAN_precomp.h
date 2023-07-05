#ifndef DBSCAN_PRECOMP_H
#define DBSCAN_PRECOMP_H

#include <pcl/point_types.h>
#include "DBSCAN_simple.h"

template <typename PointT>
class DBSCANPrecompCluster : public DBSCANSimpleCluster<PointT>  {
public:
    virtual void setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
        this->input_cloud_ = cloud;
        int size = this->input_cloud_->points.size();
        adjoint_indexes_ = std::vector<std::vector<int>>(size, std::vector<int>());
        distances_ = std::vector<std::vector<float>>(size, std::vector<float>());
        precomp();
    }

protected:
    std::vector<std::vector<float>> distances_;
    std::vector<std::vector<int>> adjoint_indexes_;

    void precomp() {
        int size = this->input_cloud_->points.size();
        for (int i = 0; i < size; i++) {
            adjoint_indexes_[i].push_back(i);
            distances_[i].push_back(0.0);
        }
        double radius_square = this->eps_ * this->eps_;
        for (int i = 0; i < size; i++) {
            for (int j = i+1; j < size; j++) {
                double distance_x = this->input_cloud_->points[i].x - this->input_cloud_->points[j].x;
                double distance_y = this->input_cloud_->points[i].y - this->input_cloud_->points[j].y;
                double distance_z = this->input_cloud_->points[i].z - this->input_cloud_->points[j].z;
                double distance_square = distance_x * distance_x + distance_y * distance_y + distance_z * distance_z;
                if (distance_square <= radius_square) {
                    adjoint_indexes_[i].push_back(j);
                    adjoint_indexes_[j].push_back(i);
                    double distance = std::sqrt(distance_square);
                    distances_[i].push_back(distance);
                    distances_[j].push_back(distance);
                }
            }
        }
    }

    virtual int radiusSearch(
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        // radius = eps_
        k_indices = adjoint_indexes_[index];
        k_sqr_distances = distances_[index];
        return k_indices.size();
    }
}; // class DBSCANCluster

#endif // DBSCAN_PRECOMP_H