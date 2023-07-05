#include <stdio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/split.hpp>

#include <Eigen/Dense>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/edge_se3_priorxyz.hpp>

#include <radar_graph_slam/graph_slam.hpp>

using namespace std;
using namespace radar_graph_slam;

// Loop 3 matches: 0-5364 947-6168
// Loop 2 matches: 0-8240 583-8658 1350-9373 1573-9582
// Loop 1 matches: 0-10133(0-1654230309)  1961-11949(1654229491.74-1654230490.55)
int main()
{
    ifstream file_in("/home/zhuge/stamped_groundtruth_1.txt");
    
    if (!file_in.is_open()) {
        cout << "Can not open this file" << endl;
        return 0;
    }
    std::vector<std::string> vectorLines;
    std::string line;
    while (getline(file_in, line)) {
        vectorLines.push_back(line);
    }

    Eigen::MatrixXd pose_data;
    pose_data.resize(vectorLines.size()-1, 8);
    std::vector<Eigen::Isometry3d> odoms;

    for (size_t i = 1; i < vectorLines.size(); i++) {
        std::string line_ = vectorLines.at(i);
        double stamp,tx,ty,tz,qx,qy,qz,qw;
        stringstream data(line_);
        data >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        pose_data.block<1, 8>(i-1, 0) << stamp, tx, ty, tz, qx, qy, qz, qw;
        Eigen::Isometry3d odom(Eigen::Isometry3d::Identity());
        odom.pretranslate(Eigen::Vector3d(tx, ty, tz));
        odom.rotate(Eigen::Quaterniond(qw, qx, qy, qz));
        odoms.push_back(odom);
    }

    // Construct Pose Graph
    std::unique_ptr<GraphSLAM> graph_slam;
    graph_slam.reset(new GraphSLAM("lm_var"));
    std::vector<g2o::VertexSE3*> vertices;
    for (size_t i = 0; i < odoms.size(); i++) {
        g2o::VertexSE3* node = graph_slam->add_se3_node(odoms.at(i));
        if (i == 0)
            node->setFixed(true);
        vertices.push_back(node);
    }
    for (size_t i = 0; i < odoms.size()-1; i++) {
        Eigen::Isometry3d rel_odom = odoms.at(i).inverse() * odoms.at(i+1);
        Eigen::MatrixXd information_matrix_se3 = Eigen::MatrixXd::Identity(6,6) / 0.05;
        auto edge_odom = graph_slam->add_se3_edge(vertices.at(i), vertices.at(i+1), rel_odom, information_matrix_se3);
        graph_slam->add_robust_kernel(edge_odom, "Huber", 1.0);
    }
    // Add Loop Edges
    // Eigen::Matrix3d information_matrix_xyz = Eigen::Matrix3d::Identity() / 1.0;
    // graph_slam->add_se3_prior_xyz_edge(vertices.at(8240), Eigen::Vector3d(0,0,0), information_matrix_xyz);
    Eigen::MatrixXd information_matrix = Eigen::MatrixXd::Identity(6, 6);
    information_matrix.block<3,3>(0,0) /= 0.5;
    information_matrix.block<3,3>(3,3) /= 1;
    // graph_slam->add_se3_edge(vertices.at(1961), vertices.at(11949), Eigen::Isometry3d::Identity(), information_matrix);
    graph_slam->add_se3_edge(vertices.at(0), vertices.at(10133), Eigen::Isometry3d::Identity(), information_matrix);
    
    // optimize the pose graph
    int num_iterations = 1024;
    graph_slam->optimize(num_iterations);
    cout << "optimized !" << endl;

    ofstream file_out("/home/zhuge/stamped_groundtruth.txt", ios::trunc);
    file_out << "# timestamp tx ty tz qx qy qz qw" << endl;
    file_out.setf(ios::fixed, ios::floatfield);  // Set to fixed mode; floating-point numbers with decimal points
    file_out.precision(8);  // Set precision
    for(size_t i = 0; i < vertices.size(); i++) {
        Eigen::Vector3d pos_ = vertices.at(i)->estimate().translation();
        Eigen::Matrix3d rot_ = vertices.at(i)->estimate().rotation();
        Eigen::Quaterniond quat_(rot_);
        double timestamp = pose_data(i, 0);
        double tx = pos_(0), ty = pos_(1), tz = pos_(2);
        double qx = quat_.x(), qy = quat_.y(), qz = quat_.z(), qw = quat_.w();

        file_out << timestamp << " "
            << tx << " " << ty << " " << tz << " "
                << qx << " " << qy << " " << qz << " " << qw << endl;
    }

    file_in.close();
    file_out.close();
	return 0;
}




