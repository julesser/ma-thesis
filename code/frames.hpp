
///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, University of Duisburg-Essen
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef CROCODDYL_MULTIBODY_FRAMES_HPP_
#define CROCODDYL_MULTIBODY_FRAMES_HPP_

#include "crocoddyl/multibody/fwd.hpp"
#include "crocoddyl/multibody/friction-cone.hpp"
#include "crocoddyl/core/mathbase.hpp"

#include <pinocchio/multibody/fwd.hpp>
#include <pinocchio/spatial/se3.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/force.hpp>

namespace crocoddyl {

typedef std::size_t FrameIndex;

template <typename _Scalar>
struct FrameCoPSupportTpl {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef _Scalar Scalar;
  typedef typename MathBaseTpl<Scalar>::Vector2s Vector2s;
  typedef typename MathBaseTpl<Scalar>::Vector3s Vector3s;
  typedef Eigen::Matrix<Scalar, 4, 6> Matrix46;

 public:
  explicit FrameCoPSupportTpl() : id_(0), support_region_(Vector2s::Zero()) { update_A(); }
  FrameCoPSupportTpl(const FrameCoPSupportTpl<Scalar>& value)
      : id_(value.get_id()), support_region_(value.get_support_region()), A_(value.get_A()) {}
  FrameCoPSupportTpl(const FrameIndex& id, const Vector2s& support_region) : id_(id), support_region_(support_region) {
    update_A();
  }
  friend std::ostream& operator<<(std::ostream& os, const FrameCoPSupportTpl<Scalar>& X) {
    os << "            id: " << X.get_id() << std::endl
       << "foot dimension: " << std::endl
       << X.get_support_region() << std::endl;
    return os;
  }

  // Define the inequality matrix A to implement A * f >= 0
  void update_A() {
    A_ << Scalar(0), Scalar(0), support_region_[0] / Scalar(2), Scalar(0), Scalar(-1), Scalar(0), Scalar(0), Scalar(0),
        support_region_[0] / Scalar(2), Scalar(0), Scalar(1), Scalar(0), Scalar(0), Scalar(0),
        support_region_[1] / Scalar(2), Scalar(1), Scalar(0), Scalar(0), Scalar(0), Scalar(0),
        support_region_[1] / Scalar(2), Scalar(-1), Scalar(0), Scalar(0);
  }

  void set_id(FrameIndex id) { id_ = id; }
  void set_support_region(const Vector2s& support_region) {
    support_region_ = support_region;
    update_A();
  }

  const FrameIndex& get_id() const { return id_; }
  const Vector2s& get_support_region() const { return support_region_; }
  const Matrix46& get_A() const { return A_; }

 private:
  FrameIndex id_;            //!< contact frame ID
  Vector2s support_region_;  //!< cop support region = (length, width)
  Matrix46 A_;               //!< inequality matrix
};

}  // namespace crocoddyl

#endif  // CROCODDYL_MULTIBODY_FRAMES_HPP_
