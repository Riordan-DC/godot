#ifndef OZZGGUTILS_CLASS_H
#define OZZGDUTILS_CLASS_H

#include "ozz/animation/runtime/skeleton.h"
/*
#include "ozz/animation/offline/additive_animation_builder.h"
#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/animation_optimizer.h"
#include "ozz/animation/offline/motion_extractor.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/offline/track_builder.h"
#include "ozz/animation/offline/track_optimizer.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/blending_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/track_sampling_job.h"
#include "ozz/animation/runtime/track_triggering_job.h"
#include "ozz/samples/motion_utils.h"
*/

#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_float4x4.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/memory/unique_ptr.h"
#include "ozz/options/options.h"
#include "ozz/samples/utils.h"

#include "core/typedefs.h"
#include "core/variant/variant.h"


// Helper to convert from Ozz Mat4x4 to godot transform3d
inline Transform3D ozz_to_godot_xform(ozz::math::Float4x4 m) {
	return Transform3D(
			ozz::math::GetX(m.cols[0]), ozz::math::GetX(m.cols[1]), ozz::math::GetX(m.cols[2]),
			ozz::math::GetY(m.cols[0]), ozz::math::GetY(m.cols[1]), ozz::math::GetY(m.cols[2]),
			ozz::math::GetZ(m.cols[0]), ozz::math::GetZ(m.cols[1]), ozz::math::GetZ(m.cols[2]),
			ozz::math::GetX(m.cols[3]), ozz::math::GetY(m.cols[3]), ozz::math::GetZ(m.cols[3])

	);
}

inline ozz::math::Float4x4 godot_to_ozz(Transform3D m) {
	ozz::math::Float4x4 r;
	r.cols[0].x = m.basis.rows[0].x;
	r.cols[0].y = m.basis.rows[1].x;
	r.cols[0].z = m.basis.rows[2].x;
	r.cols[1].x = m.basis.rows[0].y;
	r.cols[1].y = m.basis.rows[1].y;
	r.cols[1].z = m.basis.rows[2].y;
	r.cols[2].x = m.basis.rows[0].z;
	r.cols[2].y = m.basis.rows[1].z;
	r.cols[2].z = m.basis.rows[2].z;

	r.cols[3].x = m.origin.x;
	r.cols[3].y = m.origin.y;
	r.cols[3].z = m.origin.z;
	r.cols[3].w = 1.f;
	return r;
}

// Helper functor used to set weights while traversing joints hierarchy.
struct WeightSetupIterator {
	WeightSetupIterator(ozz::vector<ozz::math::SimdFloat4> *_weights, float _weight_setting) : weights(_weights), weight_setting(_weight_setting) {}
	
	void operator()(int _joint, int) {
		ozz::math::SimdFloat4 &soa_weight = weights->at(_joint / 4);
		soa_weight = ozz::math::SetI(soa_weight, ozz::math::simd_float4::Load1(weight_setting), _joint % 4);
	}

	ozz::vector<ozz::math::SimdFloat4> *weights;
	float weight_setting;
};

#endif
