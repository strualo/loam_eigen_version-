//
// Created by fansa on 2019/12/19.
//

#ifndef SRC_GEOMETRY_UTILS_H
#define SRC_GEOMETRY_UTILS_H
namespace geometryutils {
	
	template<typename PointT>
	inline void RotatePoint(const Eigen::Quaternionf &q, PointT &p) {
		Eigen::Vector3f vec, vec_out;
		vec.x() = p.x;
		vec.y() = p.y;
		vec.z() = p.z;
		vec_out = q * vec;
		p.x = vec_out.x();
		p.y = vec_out.y();
		p.z = vec_out.z();
	}
}
#endif //SRC_GEOMETRY_UTILS_H
