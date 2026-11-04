#ifndef OZZGGUTILS_CLASS_H
#define OZZGDUTILS_CLASS_H

#pragma once

#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_float4x4.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/memory/unique_ptr.h"
#include "ozz/base/maths/simd_quaternion.h"
#include "ozz/base/containers/vector.h"
#include "ozz/options/options.h"

#include "core/typedefs.h"
#include "core/variant/variant.h"

// Helper to convert from Ozz Mat4x4 to godot transform3d
Transform3D ozz_to_godot_xform(ozz::math::Float4x4 m);
ozz::math::Float4x4 godot_to_ozz(Transform3D m);

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
