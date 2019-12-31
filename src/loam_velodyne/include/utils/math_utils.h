//
// Created by fansa on 2019/12/19.
//

#ifndef SRC_MATH_UTILS_H
#define SRC_MATH_UTILS_H

#include <cmath>
#include <limits>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
namespace mathutils {
	
	// 获取反对称矩阵
	template<typename Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 3, 3> SkewSymmetric(const Eigen::MatrixBase<Derived> &v3d) {
		Eigen::Matrix<typename Derived::Scalar, 3, 3> m;
		m << typename Derived::Scalar(0), -v3d.z(), v3d.y(),
				v3d.z(), typename Derived::Scalar(0), -v3d.x(),
				-v3d.y(), v3d.x(), typename Derived::Scalar(0);
		return m;
	}
	
	// 计算两点之间的距离
	template<typename PointT>
	inline float CalcSquaredDiff(const PointT &a, const PointT &b) {
		float diff_x = a.x - b.x;
		float diff_y = a.y - b.y;
		float diff_z = a.z - b.z;
		
		return diff_x * diff_x + diff_y * diff_y + diff_z * diff_z;
	}
	
	// 计算当前点距离激光雷达坐标系的距离
	template<typename PointT>
	inline float CalcPointDistance(const PointT &p) {
		return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
	}
	
	template<typename Derived>
	inline Eigen::Quaternion<typename Derived::Scalar> DeltaQ(const Eigen::MatrixBase<Derived> &theta) {
		typedef typename Derived::Scalar Scalar_t;
		
		Eigen::Quaternion<Scalar_t> dq;
		Eigen::Matrix<Scalar_t, 3, 1> half_theta = theta;
		half_theta /= static_cast<Scalar_t>(2.0);
		dq.w() = static_cast<Scalar_t>(1.0);
		dq.x() = half_theta.x();
		dq.y() = half_theta.y();
		dq.z() = half_theta.z();
		return dq;
	}
	
	static const Eigen::Matrix3d I3x3 = Eigen::Matrix3d::Identity();
	
	template<typename Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 4, 4> LeftQuatMatrix(const Eigen::QuaternionBase<Derived> &q) {
		Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
		Eigen::Matrix<typename Derived::Scalar, 3, 1> vq = q.vec();
		typename Derived::Scalar q4 = q.w();
		m.block(0, 0, 3, 3) << q4 * I3x3 + SkewSymmetric(vq);
		m.block(3, 0, 1, 3) << -vq.transpose();
		m.block(0, 3, 3, 1) << vq;
		m(3, 3) = q4;
		return m;
	}
	
	template<typename Derived>
	inline Eigen::Matrix<typename Derived::Scalar, 4, 4> RightQuatMatrix(const Eigen::QuaternionBase<Derived> &p) {
		Eigen::Matrix<typename Derived::Scalar, 4, 4> m;
		Eigen::Matrix<typename Derived::Scalar, 3, 1> vp = p.vec();
		typename Derived::Scalar p4 = p.w();
		m.block(0, 0, 3, 3) << p4 * I3x3 - SkewSymmetric(vp);
		m.block(3, 0, 1, 3) << -vp.transpose();
		m.block(0, 3, 3, 1) << vp;
		m(3, 3) = p4;
		return m;
	}
	
	template<typename T>
	inline T RadToDeg(T rad) {
		return rad * 180.0 / M_PI;
	}
	
}

#endif //SRC_MATH_UTILS_H
