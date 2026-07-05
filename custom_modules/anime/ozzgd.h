#ifndef OZZGD_CLASS_H
#define OZZGD_CLASS_H

/*
PITFALLS:
1. Ozz Animation fallback. If there is no keyframe for a track it will default to the identity. For example: Missing position track, ok, its ZEROs now. This sucks.
	It should be the rest value.

*/


// TODO:
// 1. OzzAnimation resource in Godot. Convert and save them so they can be quickly loaded by the engine? Convert/optimise at runtime slow?
// 2. OzzAnimation seems to break the face (jaw, eyes), is this because there are no keyframes? If so, how can we stop it breaking? Use rest?
// 3. Support Godot method tracks by just using the godot animation and only sampling the method tracks

#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/blending_job.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/sampling_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/soa_float4x4.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/maths/vec_float.h"
#include "ozz/options/options.h"
#include "ozz/samples/utils.h"

// We don't need windows.h in this plugin but many others do and it throws up on itself all the time
// So best to include it and make sure CI warns us when we use something Microsoft took for their own goals....
#ifdef WIN32
#include <windows.h>
#endif

#define ADD_GETTER(class, name) ClassDB::bind_method(D_METHOD(#name), &class ::name);
#define ADD_SETTER(class, name, arg, defval) ClassDB::bind_method(D_METHOD(#name, #arg), &class ::name, DEFVAL(defval));

#include "core/object/class_db.h"
#include "core/object/ref_counted.h"
#include "core/string/print_string.h"
#include "scene/3d/skeleton_3d.h"
//#include "core/string.h"
#include "core/io/resource.h"
#include "core/math/geometry_2d.h"
#include "core/os/os.h"
#include "core/templates/a_hash_map.h"
#include "scene/animation/animation_player.h"
#include "scene/resources/animation.h"

#include <stdalign.h>

#include <cmath>
#include <vector>

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

/*
class OzzAnimationLibrary : public Resource {
	GDCLASS(OzzAnimationLibrary, Resource);

public:
ozz::unique_ptr<ozz::animation::Animation> load_animation(Ref<Animation> animation) {
    if (godot_skeleton == nullptr) {
			ERR_PRINT("Godot skeleton is null! Cannot load animation!");
			return nullptr;
		}
		// Use the Ozz animation builder to create an ozz animation.
		ozz::animation::offline::RawAnimation raw_animation;
		// All the animation keyframes times must be within range [0, duration].
		raw_animation.duration = animation->get_length();
        
		// Godot animations have separate tracks for rot/pos/scale, ozz has 1 track per joint
		// Warning: Assumption: All tracks are for bones/joints in a skeleton.
        
		// There should be as much tracks as there are joints in the skeleton that
		// this animation targets.
		raw_animation.tracks.resize(godot_skeleton->get_bone_count());
        
		// Fills each track with keyframes, in joint local-space.
		// Tracks should be ordered in the same order as joints in the
		// ozz::animation::Skeleton. Joint's names can be used to find joint's
		// index in the skeleton.
        
		{
			float motion_scale = godot_skeleton->get_motion_scale();
			// Ozz requires tracks to be ordered in the order of joints/bones in the skeleton. I.e. at bone_idx
			for (int i = 0; i < animation->get_track_count(); i++) {
				Animation::TrackType type = animation->track_get_type(i);
				NodePath path = animation->track_get_path(i);
				String bone_name = path.get_concatenated_subnames();
				int ozz_track = godot_skeleton->find_bone(bone_name); // bone idx
				if (ozz_track < 0) {
					continue;
				}
                
				switch (type) {
					case Animation::TYPE_POSITION_3D: {
						int keys = animation->track_get_key_count(i);
						for (int j = 0; j < keys; j++) {
							Variant val = animation->track_get_key_value(i, j);
							if (val.get_type() == Variant::VECTOR3) {
								Vector3 pos = (Vector3)val;
								float key_time = animation->track_get_key_time(i, j);
								const ozz::animation::offline::RawAnimation::TranslationKey key = { key_time, ozz::math::Float3(pos.x * motion_scale, pos.y * motion_scale, pos.z * motion_scale) };
								raw_animation.tracks[ozz_track].translations.push_back(key);
							}
						}
						break;
					}
					case Animation::TYPE_ROTATION_3D: {
						int keys = animation->track_get_key_count(i);
						for (int j = 0; j < keys; j++) {
							Variant val = animation->track_get_key_value(i, j);
							if (val.get_type() == Variant::QUATERNION) {
								Quaternion rot = (Quaternion)val;
								float key_time = animation->track_get_key_time(i, j);
								const ozz::animation::offline::RawAnimation::RotationKey key = { key_time, ozz::math::Quaternion(rot.x, rot.y, rot.z, rot.w) };
								raw_animation.tracks[ozz_track].rotations.push_back(key);
							}
						}
						break;
					}
					case Animation::TYPE_SCALE_3D: {
						break;
					}
				}
			}
		}
        
		// Test for animation validity. These are the errors that could invalidate
		// an animation:
		//  1. Animation duration is less than 0.
		//  2. Keyframes' are not sorted in a strict ascending order.
		//  3. Keyframes' are not within [0, duration] range.
		if (!raw_animation.Validate()) {
			ERR_PRINT("Ozz Animation produced invalid animation. Check this code for possible issues");
			return nullptr;
		}

		// converts the RawAnimation to a runtime Animation.
		// Creates a AnimationBuilder instance.
		ozz::animation::offline::AnimationBuilder builder;
		// Executes the builder on the previously prepared RawAnimation, which returns
		// a new runtime animation instance.
		// This operation will fail and return an empty unique_ptr if the RawAnimation
		// isn't valid.
		ozz::unique_ptr<ozz::animation::Animation> ozz_animation = builder(raw_animation);
		if (ozz_animation.get() == nullptr) {
			ERR_PRINT("Failed to convert animation to ozz animation");
			return nullptr;
		}
        
		return ozz_animation;
	}
}
*/

class OzzAnimationState : public RefCounted {
	GDCLASS(OzzAnimationState, RefCounted);

public:
	// Constructor, default initialization.
	OzzAnimationState() : weight(1.f), joint_weight_setting(1.f) {
	}

	// Playback animation controller. This is a utility class that helps with
	// controlling animation playback time.
	ozz::sample::PlaybackController controller;

	// Blending weight for the layer.
	float weight;

	// Blending weight_setting setting of the joints of this layer that are
	// affected
	// by the masking.
	float joint_weight_setting;

	// Runtime animation.
	ozz::unique_ptr<ozz::animation::Animation> animation;

	// Sampling context.
	ozz::animation::SamplingJob::Context context;

	// Buffer of local transforms as sampled from animation_.
	ozz::vector<ozz::math::SoaTransform> locals;

    // Buffer of model space matrices.
	// TODO: 1 array in OzzGD
    ozz::vector<ozz::math::Float4x4> models;

	// Per-joint weights used to define the partial animation mask. Allows to
	// select which joints are considered during blending, and their individual
	// weight_setting.
	ozz::vector<ozz::math::SimdFloat4> joint_weights;

	// Event tracks are inside the godot animation
	Ref<Animation> godot_animation;

	void set_joint_weights(PackedFloat32Array weights) {
		int num_soa_joints = joint_weights.size();
		int num_weights = weights.size();

		for (int i = 0; i < num_soa_joints; i++) {
			int offset = i * 4;
			float x, y, z, w = 0.f;
			if (offset < num_weights) {
				x = weights[offset];
			}
			if ((offset + 1) < num_weights) {
				y = weights[offset + 1];
			}
			if ((offset + 2) < num_weights) {
				z = weights[offset + 2];
			}
			if ((offset + 3) < num_weights) {
				w = weights[offset + 3];
			}
			joint_weights[i] = ozz::math::simd_float4::Load(x, y, z, w);
		}
	}

	void set_weight(float w) { weight = w; }
	float get_weight() { return weight; }

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("set_joint_weights", "weights"), &OzzAnimationState::set_joint_weights, DEFVAL(PackedFloat32Array()));
		// ClassDB::bind_method(D_METHOD("init", "animation", "anime"), &AnimationState::init, DEFVAL(nullptr));

		// ClassDB::bind_method(D_METHOD("find_thash_from_bone", "bone_idx"), &AnimationState::find_thash_from_bone);
		// ClassDB::bind_method(D_METHOD("get_track_rotation", "thash"), &AnimationState::get_track_rotation);
		// ClassDB::bind_method(D_METHOD("set_track_rotation", "thash", "rot"), &AnimationState::set_track_rotation);

		// ClassDB::bind_method(D_METHOD("copy"), &AnimationState::copy);

		// setgets
		ADD_SETTER(OzzAnimationState, set_weight, weight, 1.0)
		ADD_GETTER(OzzAnimationState, get_weight)
		ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "weight"), "set_weight", "get_weight");
		/*
		ADD_SETTER(AnimationState, set_time, time, 0.0f)
		ADD_GETTER(AnimationState, get_time)
		ADD_SETTER(AnimationState, set_never_loop, loop, true)
		ADD_GETTER(AnimationState, get_never_loop)

		ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time"), "set_time", "get_time");
		ADD_PROPERTY(PropertyInfo(Variant::BOOL, "never_loop"), "set_never_loop", "get_never_loop");
		*/
	}
};

class OzzGD : public RefCounted {
	GDCLASS(OzzGD, RefCounted);

public:
	// Runtime skeleton.
	ozz::unique_ptr<ozz::animation::Skeleton> skeleton;
	float time = 0.0;
	float threshold = ozz::animation::BlendingJob().threshold;

	AnimationPlayer *animation_player = nullptr;

    PackedStringArray names;

    PackedStringArray get_names() { return names; }
	void set_names(PackedStringArray n) { names = n; }

    Array rests;

    Array get_rests() { return rests; }
	void set_rests(Array r) { rests = r; }

    Array children;

    Array get_children() { return children; }
	void set_children(Array c) { children = c; }

	AnimationPlayer *get_animation_player() { return animation_player; }
	void set_animation_player(AnimationPlayer *player) { animation_player = player; } 

	OzzGD() {
	}

	bool init() {
		if (skeleton.get() == nullptr) {
			ERR_PRINT("Ozz skeleton is null");
			return false;
		}

		if (animation_player == nullptr) {
			ERR_PRINT("Godot Animation player is null!");
			return false;
		}

		return true;
	}

	Ref<OzzAnimationState> new_state(String animation_name) {
		if (skeleton.get() == nullptr) {
			ERR_PRINT("Ozz skeleton is null. Cannot create new animation state.");
			return nullptr;
		}

		const int num_joints = skeleton->num_joints();
		const int num_soa_joints = skeleton->num_soa_joints();

		// Load animation
		Ref<Animation> animation = animation_player->get_animation(animation_name);
		if (animation.is_null()) {
			ERR_PRINT("Animation does not exist in animation player!");
			return nullptr;
		}

		Ref<OzzAnimationState> as = Ref<OzzAnimationState>(memnew(OzzAnimationState));
		as->animation = load_animation(animation);

		// Allocates sampler runtime buffers.
		as->locals.resize(num_soa_joints);
        as->models.resize(num_joints);
		// Allocates a context that matches animation requirements.
		as->context.Resize(num_joints);

		// Allocates per-joint weights used for the partial animation. Note that
		// this is a Soa structure.
		as->joint_weights.resize(num_soa_joints);

		// Enable all joints
		for (int i = 0; i < skeleton->num_soa_joints(); ++i) {
			as->joint_weights[i] = ozz::math::simd_float4::one();
		}

		return as;
	}

	Array apply_animation_state(OzzAnimationState *state) {
        Array poses;
        
        // Converts from local space to model space matrices.
        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = skeleton.get();
        ltm_job.input = make_span(state->locals);
        ltm_job.output = make_span(state->models);
        if (!ltm_job.Run()) {
			return poses;
		}
        
        poses.resize(skeleton->num_joints());
        poses.fill(Transform3D());
        for (int i = 0; i < state->models.size(); i++) {
            const ozz::math::Float4x4& matrix = state->models[i];
            ozz::math::Float3 x_axis;
            ozz::math::Float3 y_axis;
            ozz::math::Float3 z_axis;
            ozz::math::Float3 position;
            ozz::math::Store3PtrU(matrix.cols[0], &x_axis.x);
            ozz::math::Store3PtrU(matrix.cols[1], &y_axis.x);
            ozz::math::Store3PtrU(matrix.cols[2], &z_axis.x);
            ozz::math::Store3PtrU(matrix.cols[3], &position.x);
            Basis basis = Basis(Vector3(x_axis.x, x_axis.y, x_axis.z), Vector3(y_axis.x, y_axis.y, y_axis.z), Vector3(z_axis.x, z_axis.y, z_axis.z));
            Transform3D pose = Transform3D(basis, Vector3(position.x, position.y, position.z));
            poses[i] = pose;
        }

        /*
		const int end = skeleton->num_joints();
		for (int i = 0; i < end;) {
			// transform is a soa transform. So its loc, rot, scale are actually arrays of 4
			const ozz::math::SoaTransform &soa_transform = state->locals[i / 4];
            
			// This is slower than it needs to be because I am doing it 4 times as many as needed
            
			// Transpose SoA data to AoS.
			ozz::math::SimdFloat4 translations[4];
			ozz::math::Transpose3x4(&soa_transform.translation.x, translations);
			ozz::math::SimdFloat4 rotations[4];
			ozz::math::Transpose4x4(&soa_transform.rotation.x, rotations);
			ozz::math::SimdFloat4 scales[4];
			ozz::math::Transpose3x4(&soa_transform.scale.x, scales);
            
			// Stores to the Transform object.
			for (int j = 0; j < 4 && i < end; j++, i++) {
				ozz::math::Transform pose;
				const int offset = j;
				ozz::math::Store3PtrU(translations[offset], &pose.translation.x);
				ozz::math::StorePtrU(rotations[offset], &pose.rotation.x);
				ozz::math::Store3PtrU(scales[offset], &pose.scale.x);
                
				Quaternion rot;
				Vector3 pos, scale;
				// Pos in Godot Skeletons is local to the skeleton, not to the parent
				// Ozz pos is local to parent.
				// So we must add the rest
                
				pos = Vector3(pose.translation.x, pose.translation.y, pose.translation.z);
				//Transform3D rest_pose = rests[i];
				//pos += rest_pose.origin;
                
				rot = Quaternion(pose.rotation.x, pose.rotation.y, pose.rotation.z, pose.rotation.w);
				scale = Vector3(pose.scale.x, pose.scale.y, pose.scale.z);
                Basis basis = Basis(rot).scaled(scale);
                
                poses.append(Transform3D(basis, pos));
			}
		}
        */
        return poses;
	}

	bool play_animation(Ref<OzzAnimationState> state, float delta, bool update_cache = true) {
		// Skeleton and animation needs to match.
		if (skeleton->num_joints() != state->animation->num_tracks()) {
			Array args;
			args.push_back(state->animation->num_tracks());
			args.push_back(skeleton->num_joints());
			ERR_PRINT(String("Ozz animation tracks ({0}) doesnt match ozz skeleton joints ({1})").format(args));
			return false;
		}

		state->controller.Update(*state->animation, delta);

		if (update_cache == false) {
			return false;
		}

		// Setup sampling job.
		ozz::animation::SamplingJob sampling_job;
		sampling_job.animation = state->animation.get();
		sampling_job.context = &state->context;
		sampling_job.ratio = state->controller.time_ratio();
		sampling_job.output = make_span(state->locals);

		// Samples animation.
		if (!sampling_job.Run()) {
			return false;
		}

		return true;
	}

	void blend_animation(Ref<OzzAnimationState> state, Ref<OzzAnimationState> blend_state, float blend_amount) {
		ozz::animation::BlendingJob::Layer layers[2];

		layers[0].transform = make_span(state->locals);
		layers[0].weight = state->weight;
		// Set per-joint weights for the partially blended layer.
		layers[0].joint_weights = make_span(state->joint_weights);

		layers[1].transform = make_span(blend_state->locals);
		layers[1].weight = blend_state->weight;
		// Set per-joint weights for the partially blended layer.
		layers[1].joint_weights = make_span(blend_state->joint_weights);

		// Setups blending job.
		ozz::animation::BlendingJob blend_job;
		blend_job.threshold = threshold;
		blend_job.layers = layers;
		blend_job.rest_pose = skeleton->joint_rest_poses();
		blend_job.output = make_span(state->locals);

		// Blends.
		if (!blend_job.Run()) {
			return;
		}
	}

	void add_animation(Ref<OzzAnimationState> state, Ref<OzzAnimationState> blend_state, float blend_amount) {
		ozz::animation::BlendingJob::Layer layers[2];

		layers[0].transform = make_span(state->locals);
		layers[0].weight = state->weight;
		// Set per-joint weights for the partially blended layer.
		layers[0].joint_weights = make_span(state->joint_weights);

		layers[1].transform = make_span(blend_state->locals);
		layers[1].weight = blend_state->weight;
		// Set per-joint weights for the partially blended layer.
		layers[1].joint_weights = make_span(blend_state->joint_weights);

		// Setups blending job.
		ozz::animation::BlendingJob blend_job;
		blend_job.threshold = threshold;
		blend_job.additive_layers = layers;
		blend_job.rest_pose = skeleton->joint_rest_poses();
		blend_job.output = make_span(state->locals);

		// Blends.
		if (!blend_job.Run()) {
			return;
		}
	}

	void _add_children_to_joint(int bone_idx, ozz::animation::offline::RawSkeleton::Joint *joint) {
		// Set name
		String bone_name = names[bone_idx];
		CharString char_bone_name = bone_name.utf8();
		joint->name = ozz::string(char_bone_name.get_data());
		// Set rest
		Transform3D rest = rests[bone_idx];
		Vector3 pos = rest.origin;
		Quaternion rot = rest.basis.get_rotation_quaternion();
		Vector3 scale = rest.basis.get_scale();
		joint->transform.translation = ozz::math::Float3(pos.x, pos.y, pos.z);
		joint->transform.rotation = ozz::math::Quaternion(rot.x, rot.y, rot.z, rot.w);
		joint->transform.scale = ozz::math::Float3(scale.x, scale.y, scale.z);

		PackedInt32Array bone_children = static_cast<PackedInt32Array>(children[bone_idx]);
		if (bone_children.size() > 0) {
			joint->children.resize(bone_children.size());

			for (int i = 0; i < bone_children.size(); i++) {
				ozz::animation::offline::RawSkeleton::Joint &new_joint = joint->children[i];
				int new_bone_idx = bone_children[i];
				_add_children_to_joint(new_bone_idx, &new_joint);
			}
		}
	}

	void load_skeleton() {
		// Creates a RawSkeleton.
		ozz::animation::offline::RawSkeleton raw_skeleton;

		raw_skeleton.roots.resize(1);
		ozz::animation::offline::RawSkeleton::Joint &root = raw_skeleton.roots[0];
		_add_children_to_joint(0, &root);

		// Test for skeleton validity.
		// The main invalidity reason is the number of joints, which must be lower
		// than ozz::animation::Skeleton::kMaxJoints.
		if (!raw_skeleton.Validate()) {
			ERR_FAIL_MSG("Ozz could not build a valid skeleton!");
		}

		// converts the RawSkeleton to a runtime Skeleton.
		// Creates a SkeletonBuilder instance.
		ozz::animation::offline::SkeletonBuilder builder;

		// Executes the builder on the previously prepared RawSkeleton, which returns
		// a new runtime skeleton instance.
		// This operation will fail and return an empty unique_ptr if the RawSkeleton
		// isn't valid.
		skeleton = builder(raw_skeleton);
		ERR_FAIL_NULL_MSG(skeleton.get(), "Skeleton failed to build");
	}

	int find_bone(String bone_name) {
		for (int i = 0; i < names.size(); i++) {
			if (names[i] == bone_name) {
				return i;
			}
		}
		return -1;
	}

	ozz::unique_ptr<ozz::animation::Animation> load_animation(Ref<Animation> animation) {
		// Use the Ozz animation builder to create an ozz animation.
		ozz::animation::offline::RawAnimation raw_animation;
		// All the animation keyframes times must be within range [0, duration].
		raw_animation.duration = animation->get_length();

		// Godot animations have separate tracks for rot/pos/scale, ozz has 1 track per joint
		// Warning: Assumption: All tracks are for bones/joints in a skeleton.

		// There should be as much tracks as there are joints in the skeleton that
		// this animation targets.
        int bones = names.size();
		raw_animation.tracks.resize(bones);

		// Fills each track with keyframes, in joint local-space.
		// Tracks should be ordered in the same order as joints in the
		// ozz::animation::Skeleton. Joint's names can be used to find joint's
		// index in the skeleton.

		{
			float motion_scale = 1.f; //godot_skeleton->get_motion_scale();
			// Ozz requires tracks to be ordered in the order of joints/bones in the skeleton. I.e. at bone_idx
			for (int i = 0; i < animation->get_track_count(); i++) {
				Animation::TrackType type = animation->track_get_type(i);
				NodePath path = animation->track_get_path(i);
				String bone_name = path.get_concatenated_subnames();
				int ozz_track = find_bone(bone_name); // bone idx
				if (ozz_track < 0) {
					continue;
				}

				switch (type) {
					case Animation::TYPE_POSITION_3D: {
						int keys = animation->track_get_key_count(i);
						for (int j = 0; j < keys; j++) {
							Variant val = animation->track_get_key_value(i, j);
							if (val.get_type() == Variant::VECTOR3) {
								Vector3 pos = (Vector3)val;
								float key_time = animation->track_get_key_time(i, j);
								const ozz::animation::offline::RawAnimation::TranslationKey key = { key_time, ozz::math::Float3(pos.x * motion_scale, pos.y * motion_scale, pos.z * motion_scale) };
								raw_animation.tracks[ozz_track].translations.push_back(key);
							}
						}
						break;
					}
					case Animation::TYPE_ROTATION_3D: {
						int keys = animation->track_get_key_count(i);
						for (int j = 0; j < keys; j++) {
							Variant val = animation->track_get_key_value(i, j);
							if (val.get_type() == Variant::QUATERNION) {
								Quaternion rot = (Quaternion)val;
								float key_time = animation->track_get_key_time(i, j);
								const ozz::animation::offline::RawAnimation::RotationKey key = { key_time, ozz::math::Quaternion(rot.x, rot.y, rot.z, rot.w) };
								raw_animation.tracks[ozz_track].rotations.push_back(key);
							}
						}
						break;
					}
					case Animation::TYPE_SCALE_3D: {
						int keys = animation->track_get_key_count(i);
						for (int j = 0; j < keys; j++) {
							Variant val = animation->track_get_key_value(i, j);
							if (val.get_type() == Variant::VECTOR3) {
								Vector3 scale = (Vector3)val;
								float key_time = animation->track_get_key_time(i, j);
								const ozz::animation::offline::RawAnimation::ScaleKey key = { key_time, ozz::math::Float3(scale.x, scale.y, scale.z) };
								raw_animation.tracks[ozz_track].scales.push_back(key);
							}
						}
						break;
					}
				}
			}
		}

		// Insert default rest poses
		for (int ozz_track = 0; ozz_track < raw_animation.tracks.size(); ozz_track++) {
			Transform3D rest = rests[ozz_track];
			if (raw_animation.tracks[ozz_track].translations.size() == 0) {
				Vector3 translation = rest.origin;
				const ozz::animation::offline::RawAnimation::TranslationKey key = { 0.f, ozz::math::Float3(translation.x, translation.y, translation.z) };
				raw_animation.tracks[ozz_track].translations.push_back(key);
			}
			if (raw_animation.tracks[ozz_track].rotations.size() == 0) {
				Quaternion rot = rest.basis.get_rotation_quaternion();
				const ozz::animation::offline::RawAnimation::RotationKey key = { 0.f, ozz::math::Quaternion(rot.x, rot.y, rot.z, rot.w) };
				raw_animation.tracks[ozz_track].rotations.push_back(key);
			}
			if (raw_animation.tracks[ozz_track].scales.size() == 0) {
				Vector3 scale = rest.basis.get_scale();
				const ozz::animation::offline::RawAnimation::ScaleKey key = { 0.f, ozz::math::Float3(scale.x, scale.y, scale.z) };
				raw_animation.tracks[ozz_track].scales.push_back(key);
			}
		}


		// Test for animation validity. These are the errors that could invalidate
		// an animation:
		//  1. Animation duration is less than 0.
		//  2. Keyframes' are not sorted in a strict ascending order.
		//  3. Keyframes' are not within [0, duration] range.
		if (!raw_animation.Validate()) {
			ERR_PRINT("Ozz Animation produced invalid animation. Check this code for possible issues");
			return nullptr;
		}

		// converts the RawAnimation to a runtime Animation.
		// Creates a AnimationBuilder instance.
		ozz::animation::offline::AnimationBuilder builder;
		// Executes the builder on the previously prepared RawAnimation, which returns
		// a new runtime animation instance.
		// This operation will fail and return an empty unique_ptr if the RawAnimation
		// isn't valid.
		ozz::unique_ptr<ozz::animation::Animation> ozz_animation = builder(raw_animation);
		if (ozz_animation.get() == nullptr) {
			ERR_PRINT("Failed to convert animation to ozz animation");
			return nullptr;
		}

		return ozz_animation;
	}

	static void _bind_methods() {
		ADD_SETTER(OzzGD, set_animation_player, animation_player, 0);
		ADD_GETTER(OzzGD, get_animation_player);
		ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "animation_player"), "set_animation_player", "get_animation_player");

		ADD_SETTER(OzzGD, set_names, names, PackedStringArray());
		ADD_GETTER(OzzGD, get_names);
		ADD_PROPERTY(PropertyInfo(Variant::PACKED_STRING_ARRAY, "names"), "set_names", "get_names");

		ADD_SETTER(OzzGD, set_rests, rests, Array());
		ADD_GETTER(OzzGD, get_rests);
		ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "rests"), "set_rests", "get_rests");

		ADD_SETTER(OzzGD, set_children, children, Array());
		ADD_GETTER(OzzGD, get_children);
		ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "children"), "set_children", "get_children");

		ClassDB::bind_method(D_METHOD("init"), &OzzGD::init);
		ClassDB::bind_method(D_METHOD("play_animation", "state", "delta", "update_state"), &OzzGD::play_animation);
		ClassDB::bind_method(D_METHOD("new_state", "animation_name"), &OzzGD::new_state);
		ClassDB::bind_method(D_METHOD("blend_animation", "state", "blend_state", "blend_amount"), &OzzGD::blend_animation);
		//ClassDB::bind_method(D_METHOD("load_animation", "skeleton", "animation"), &OzzGD::load_animation);
		ClassDB::bind_method(D_METHOD("load_skeleton"), &OzzGD::load_skeleton);
		ClassDB::bind_method(D_METHOD("apply_animation_state", "state"), &OzzGD::apply_animation_state);
	}
};

#endif