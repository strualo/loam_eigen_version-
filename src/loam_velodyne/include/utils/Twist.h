//
// Created by fansa on 2019/12/19.
//

#ifndef SRC_TWIST_H
#define SRC_TWIST_H
#include <iostream>
#include <eigen3/Eigen/Eigen>


template<typename T>
struct Twist {
	Eigen::Quaternion<T> rot;
	Eigen::Matrix<T, 3, 1> pos;
	
	static Twist Identity() {
		return Twist();
	}
	
	Twist() {
		// 初始化旋转和平移
		rot.setIdentity();
		pos.setZero();
	}
	
	// 传进来的是四元数和平移向量
	Twist(Eigen::Quaternion<T> rot_in, Eigen::Matrix<T, 3, 1> pos_in) {
		this->rot = rot_in;
		this->pos = pos_in;
	}
	
	// 传进来的是Transform
	Twist(Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform) {
		this->rot = Eigen::Quaternion<T>{transform.linear()}.normalized();
		this->pos = transform.translation();
	}
	
	// 返回Transform
	Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform() const {
		Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform;
		transform.linear() = rot.normalized().toRotationMatrix();
		transform.translation() = pos;
		return transform;
	}
	
	// 返回一个Twist类型的逆变换
	Twist inverse() const {
		Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_inv = this->transform().inverse();
		Twist twist_inv;
		twist_inv.rot = transform_inv.linear();
		twist_inv.pos = transform_inv.translation();
		return twist_inv;
	}
	
	// 重载 *
	Twist operator*(const Twist &other) const {
		Eigen::Transform<T, 3, Eigen::TransformTraits::Affine> transform_out =
				this->transform() * other.transform();
		return Twist(transform_out);
	}
	
	template<typename NewType>
	Twist<NewType> cast() const {
		Twist<NewType> twist_new{this->rot.template cast<NewType>(), this->pos.template cast<NewType>()};
		return twist_new;
	}
	
	// 友元函数 重载运算符 <<
	friend std::ostream &operator<<(std::ostream &os, const Twist &twist) {
		os << twist.pos.x() << " " << twist.pos.y() << " " << twist.pos.z() << " " << twist.rot.w() << " "
		   << twist.rot.x()
		   << " " << twist.rot.y() << " " << twist.rot.z();
		return os;
	}

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
#endif //SRC_TWIST_H
