#ifndef OZZGGUTILS_CLASS_H
#define OZZGDUTILS_CLASS_H

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
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/track_sampling_job.h"
#include "ozz/animation/runtime/track_triggering_job.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_float4x4.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/base/memory/unique_ptr.h"
#include "ozz/options/options.h"
#include "ozz/samples/motion_utils.h"
#include "ozz/samples/utils.h"

#include "core/io/resource.h"
#include "core/math/geometry_2d.h"
#include "core/object/class_db.h"
#include "core/object/ref_counted.h"
#include "core/os/os.h"
#include "core/string/print_string.h"
#include "core/string/ustring.h"
#include "core/templates/a_hash_map.h"
#include "core/typedefs.h"
#include "core/variant/variant.h"
#include "scene/3d/skeleton_3d.h"
#include "scene/main/node.h"
#include "scene/resources/animation.h"
#include "servers/physics_3d/physics_server_3d.h"
#include "servers/rendering/rendering_server.h"
//#include "scene/main/scene_tree.h

#include <stdalign.h>

#include <cmath>
#include <unordered_map>
#include <vector>

// Helper to convert from Ozz Mat4x4 to godot transform3d
inline Transform3D ozz_to_godot_xform(ozz::math::Float4x4 m) {
	return Transform3D(
			ozz::math::GetX(m.cols[0]), ozz::math::GetX(m.cols[1]), ozz::math::GetX(m.cols[2]),
			ozz::math::GetY(m.cols[0]), ozz::math::GetY(m.cols[1]), ozz::math::GetY(m.cols[2]),
			ozz::math::GetZ(m.cols[0]), ozz::math::GetZ(m.cols[1]), ozz::math::GetZ(m.cols[2]),
			ozz::math::GetX(m.cols[3]), ozz::math::GetY(m.cols[3]), ozz::math::GetZ(m.cols[3])

	);
}

// Helper functor used to set weights while traversing joints hierarchy.
struct WeightSetupIterator {
	WeightSetupIterator(ozz::vector<ozz::math::SimdFloat4> *_weights,
			float _weight_setting) : weights(_weights),
									 weight_setting(_weight_setting) {}
	void operator()(int _joint, int) {
		ozz::math::SimdFloat4 &soa_weight = weights->at(_joint / 4);
		soa_weight = ozz::math::SetI(soa_weight, ozz::math::simd_float4::Load1(weight_setting), _joint % 4);
	}
	ozz::vector<ozz::math::SimdFloat4> *weights;
	float weight_setting;
};

ozz::math::SoaTransform single_float4x4_to_soa_transform(const ozz::math::Float4x4 mats, ozz::math::SoaTransform soa_transform, int lane) {
    ozz::math::SimdFloat4 translation;
    ozz::math::SimdFloat4 rotation;
    ozz::math::SimdFloat4 scale;

	ozz::math::ToAffine(mats, &translation, &rotation, &scale);
  
	soa_transform.translation.x = ozz::math::SetI(soa_transform.translation.x, translation, lane);
	soa_transform.translation.y = ozz::math::SetI(soa_transform.translation.y, translation, lane);
	soa_transform.translation.z = ozz::math::SetI(soa_transform.translation.z, translation, lane);

	soa_transform.rotation.x = ozz::math::SetI(soa_transform.rotation.x, rotation, lane);
	soa_transform.rotation.y = ozz::math::SetI(soa_transform.rotation.y, rotation, lane);
	soa_transform.rotation.z = ozz::math::SetI(soa_transform.rotation.z, rotation, lane);
	soa_transform.rotation.w = ozz::math::SetI(soa_transform.rotation.w, rotation, lane);

	soa_transform.scale.x = ozz::math::SetI(soa_transform.scale.x, scale, lane);
	soa_transform.scale.y = ozz::math::SetI(soa_transform.scale.y, scale, lane);
	soa_transform.scale.z = ozz::math::SetI(soa_transform.scale.z, scale, lane);
    return soa_transform;
}

ozz::math::SoaTransform float4x4_to_soa_transform(const ozz::math::Float4x4 mats[4]) {
    ozz::math::SimdFloat4 translations[4];
    ozz::math::SimdFloat4 rotations[4];
    ozz::math::SimdFloat4 scales[4];

    // 1. Decompose each Float4x4 matrix
    for (int i = 0; i < 4; ++i) {
        ozz::math::ToAffine(mats[i], &translations[i], &rotations[i], &scales[i]);
    }

    // 2. Pack into a single SoaTransform
    ozz::math::SoaTransform soa_transform;
    
    // Pack translations (x, y, z)
	ozz::math::Transpose4x3(translations, &soa_transform.translation.x);
	ozz::math::Transpose4x4(rotations, &soa_transform.rotation.x); // or rotation.x/y/z/w loading
	ozz::math::Transpose4x3(scales, &soa_transform.scale.x);
    return soa_transform;
}

// We want to support blending, both regular and additive, in model (or mesh) space.
// This requires us to convert our local poses to model space. Perform the operation there.
// Then convert back to local space.
// This presents an issue. Ozz does not provide a job for model to local conversion.
// Even worse, ozz local to model converts our SoaTransforms to Float4x4.
// The process becomes this:
// 1. Convert base and blend pose to model space
// 2. Cast model space poses from Float4x4 to SoaTransform
// 3. Perform blend operation
// 4. Convert model space poses to local space poses 
// Operation 4 is the most difficult. The blend operation provides us SoaTransform
// to convert to model we need to convert a SoaTransform into a ozz::Transform, perform an inverse,
// and store the result back into a SoaTransform
void models_to_locals(ozz::animation::Skeleton& skeleton, ozz::vector<ozz::math::SoaTransform>& input,
	ozz::vector<ozz::math::SoaTransform>& output) {
		int joints = skeleton.num_joints();

		ozz::vector<ozz::math::Float4x4> transforms;
		transforms.resize(joints);

		ozz::span<const int16_t> parents = skeleton.joint_parents();
		int soa_joints = skeleton.num_soa_joints();

		for (int i = 0; i < soa_joints; i++) {
			ozz::math::SoaTransform transform = input[i];
			const ozz::math::SoaFloat4x4 local_soa_matrices = ozz::math::SoaFloat4x4::FromAffine(
				transform.translation, transform.rotation, transform.scale);

			// Converts to aos matrices.
			ozz::math::Float4x4 aos_matrices[4];
			ozz::math::Transpose16x16(&local_soa_matrices.cols[0].x, aos_matrices->cols);
			
			ozz::math::Float4x4 soa_bundle[4];

			for (int j = 0; j < 4; j++) {
				int joint = i * 4 + j;
				if (joint >= parents.size()) {
					soa_bundle[j] = ozz::math::Float4x4::identity();
					continue;
				}

				int parent = parents[joint];
				ozz::math::Float4x4 model_transform = aos_matrices[j];
				
				if (parent == ozz::animation::Skeleton::kNoParent) {
					transforms[joint] = model_transform;
				} else {
					ozz::math::Float4x4 model_parent_transform = transforms[parent];
					model_transform = ozz::math::Invert(model_parent_transform) * model_transform;
					transforms[joint] = model_transform;
				}
				soa_bundle[j] = model_transform;
			}
			// Bundle up the last 4 
			output[i] = float4x4_to_soa_transform(soa_bundle);
		}
}

#endif
