#ifndef VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H
#define VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H

#include <Eigen/Core>
#include <glog/logging.h>

#include <voxblox/core/map.h>

namespace voxblox {

typedef Eigen::Matrix<FloatingPoint, 3, 1> Point;
typedef Eigen::Matrix<FloatingPoint, 3, 1> Ray;
typedef Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> Points;
typedef Eigen::Matrix<uint8_t, 3, Eigen::Dynamic> Colors;

class Integrator {
	public:

	struct IntegratorConfig {
		float truncation_distance = 0.1;
	};

	Integrator(const TsdfMap::Ptr& map, const IntegratorConfig& config) :
			map_(map), config_(config) {
		CHECK(map_);

		voxel_size_ = map_->getVoxelSize();
		block_size_ = map_->getBlockSize();
		voxels_per_side_ = map_->getVoxelsPerSide();

		voxel_size_inv_ = 1.0 / voxel_size_;
		block_size_inv_ = 1.0 / block_size_;
		voxels_per_side_inv_ = 1.0 / voxels_per_side_;
	}

	void integratePointCloud(const Transformation& T_G_C, const Points& points_C, const Colors& colors){


		const Points points_G = T_G_C * points_C;


		const Point& origin = T_G_C.getPosition();

		for(size_t pt_idx = 0; pt_idx < points_C.cols(); ++pt_idx){
			const Point& point_G = points_G.col(pt_idx);
			const Ray& ray = point_G - origin;



		}
	}


	protected:
		TsdfMap::Ptr map_;

		IntegratorConfig config_;

		// Cached map config.
		FloatingPoint voxel_size_;
		FloatingPoint voxels_per_side_;
		FloatingPoint block_size_;

		// Derived types.
		FloatingPoint voxel_size_inv_;
		FloatingPoint voxels_per_side_inv_;
		FloatingPoint block_size_inv_;
};


}  // namespace voxblox

#endif  // VOXBLOX_INTEGRATOR_RAY_INTEGRATOR_H
