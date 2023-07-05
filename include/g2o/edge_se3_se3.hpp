// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef KKL_G2O_EDGE_SE3_SE3_HPP
#define KKL_G2O_EDGE_SE3_SE3_HPP

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
// #include <g2o/types/sba/edge_se3_expmap.h>
// #include <g2o/types/sba/vertex_se3_expmap.h>

#include <Eigen/Dense>
#include <math.h>

namespace g2o {
class EdgeSE3SE3 : public g2o::BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3, g2o::VertexSE3> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  EdgeSE3SE3() : BaseBinaryEdge<6, g2o::SE3Quat, g2o::VertexSE3, g2o::VertexSE3>() {}

  void computeError() override {
    const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
    const g2o::VertexSE3* v2 = static_cast<const g2o::VertexSE3*>(_vertices[1]);

    g2o::SE3Quat estimate1 (v1->estimate().rotation(), v1->estimate().translation());
    g2o::SE3Quat estimate2 (v2->estimate().rotation(), v2->estimate().translation());

    g2o::SE3Quat C(_measurement);
    SE3Quat error= estimate2.inverse() * C * estimate1;
    _error = error.log();


    // const VertexSE3* v1 = dynamic_cast<const VertexSE3*>(_vertices[0]);
    // const VertexSE3* v2 = dynamic_cast<const VertexSE3*>(_vertices[1]);
    // SE3Quat delta = _measurement.inverse(). * (v1->estimate().inverse()*v2->estimate());
    // _error.head<3>() = delta.translation();
    // // The analytic Jacobians assume the error in this special form (w beeing positive)
    // if (delta.rotation().w() < 0.)
    //   _error.tail<3>() =  - delta.rotation().vec();
    // else
    //   _error.tail<3>() =  delta.rotation().vec();
  }

  // void linearizeOplus() override {
  //   const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
  //   const g2o::VertexSE3* v2 = static_cast<const g2o::VertexSE3*>(_vertices[1]);

  //   SE3Quat Ti (v1->estimate().rotation(), v1->estimate().translation());
  //   SE3Quat Tj (v2->estimate().rotation(), v2->estimate().translation());

  //   //注意这里把测量标记为Tij应该是标记错误了，应该是Tji，不然整个误差公式说不通了
  //   //这个可以看orbslam EdgeSim3里添加测量时就是用的Sji
  //   const SE3Quat & Tij = _measurement; // shoulb be Tji
  //   SE3Quat invTij = Tij.inverse();

  //   SE3Quat invTj_Tij = Tj.inverse()*Tij;
  //   SE3Quat infTi_invTij = Ti.inverse()*invTij;

  //   _jacobianOplusXi = invTj_Tij.adj();
  //   _jacobianOplusXj = - infTi_invTij.adj();
  // }


  void setMeasurement(const g2o::SE3Quat& m) override {
    _measurement = m;
  }

  virtual bool read(std::istream& is) override {
    g2o::Vector6 vec6;
    is >> vec6(0) >> vec6(1) >> vec6(2) >> vec6(3) >> vec6(4) >> vec6(5);
    double q = sqrt(1.0-vec6(0)*vec6(0)-vec6(1)*vec6(1)-vec6(2)*vec6(2));
    g2o::SE3Quat v(Eigen::Quaterniond(q,vec6(0),vec6(1),vec6(2)), Eigen::Vector3d(vec6(3),vec6(4),vec6(5)));
    setMeasurement(v);
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) {
        is >> information()(i, j);
        if(i != j) information()(j, i) = information()(i, j);
      }
    return true;
  }
  virtual bool write(std::ostream& os) const override {
    g2o::SE3Quat v = _measurement;
    g2o::Vector6 vec6 = v.log();
    os << vec6(0) << " " << vec6(1) << " " << vec6(2) << " " << vec6(3) << " " << vec6(4) << " " << vec6(5) << " ";
    for(int i = 0; i < information().rows(); ++i)
      for(int j = i; j < information().cols(); ++j) os << " " << information()(i, j);
    return os.good();
  }
};
}  // namespace g2o

#endif
