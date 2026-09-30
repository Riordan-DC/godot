#ifndef OZZGD_CLASS_H
#define OZZGD_CLASS_H

/*
PITFALLS:
1. Ozz Animation fallback. If there is no keyframe for a track it will default to the identity. For example: Missing position track, ok, its ZEROs now. This sucks.
	It should be the rest value.
2. Godot arrays copy on write. This makes them slow in situations where they need to be edited inside a tight loop.
*/

// TODO:
// 4. Inertia blending. Dont cross blend. Simply snap to next animation. When snap occurs trigger a temporary post processing
// 	which blends the last pose into the current. See the orange duck post on spring-roll-call
// 5. Aim offsets
// 6. mesh space additive
//	method: add a blend layer has a negative weight to subtract the current pose
//	normal add: pose A + pose B
//	mesh add: the delta between pose A and pose B
//  i..e lets say our regular delta adds 15 degrees. our parent rotates 45 so the final rotation = 45 + 15
// 	instead lets make our delta 15-45 = -30. then our final rotation is 45 + -30 = 15
// 7. build delta animation
//	1. create delta animation using builder either with skeleton reference or first frame
//	2. each frame of that animation becomes an additive pose in an array
// TODO: Remove first frame of delta animation??

#include "utils.h"

#include "ozz/animation/offline/additive_animation_builder.h"
#include "ozz/animation/offline/animation_builder.h"
#include "ozz/animation/offline/animation_optimizer.h"
#include "ozz/animation/offline/motion_extractor.h"
#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/offline/skeleton_builder.h"
#include "ozz/animation/offline/tools/gltf2ozz.h"
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

// We don't need windows.h in this plugin but many others do and it throws up on itself all the time
// So best to include it and make sure CI warns us when we use something Microsoft took for their own goals....
#ifdef WIN32
#include <windows.h>
#endif

#define ADD_GETTER(class, name) ClassDB::bind_method(D_METHOD(#name), &class ::name);
#define ADD_SETTER(class, name, arg, defval) ClassDB::bind_method(D_METHOD(#name, #arg), &class ::name, DEFVAL(defval));

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

// Turn off optimisation for debugging
#pragma optimize("", off)

class OzzAnimationState;
class OzzGD;

// Blend matrix
// setup(animations[], width: int)
// setup_offset(animation, frames: int)

class OzzAnimationState : public RefCounted {
	GDCLASS(OzzAnimationState, RefCounted);

public:
	// Constructor, default initialization.
	OzzAnimationState() : weight(1.f), joint_weight_setting(1.f), transform(ozz::math::Float4x4::identity()) {
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

	// motion tracks
	ozz::sample::MotionTrack motion_track;
	// Motion accumulator helper
	ozz::sample::MotionSampler motion_sampler;
	ozz::math::Float4x4 transform;

	// Event tracks
	ozz::vector<ozz::unique_ptr<ozz::animation::FloatTrack>> events;

	// Outputs
	std::vector<Vector3> positions;
	std::vector<Vector4> rotations;
	std::vector<Vector3> scales;

	// Settings
	bool has_motion = false;
	bool has_events = false;
	bool sample_events = true;

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

	void copy_pose(Ref<OzzAnimationState> as) {
		if (locals.size() != as->locals.size()) {
			locals.resize(as->locals.size());
		}
		locals.assign(as->locals.begin(), as->locals.end());
	}

	// duration * factor = true duration
	float get_duration() { return animation->duration(); }

	void set_weight(float w) { weight = w; }
	float get_weight() { return weight; }

	void set_playback_speed(float f) { controller.set_playback_speed(f); }
	float get_playback_speed() { return controller.playback_speed(); }

	// useful for seeking through an animation. After call play animation with delta = 0.0 to update cache
	void set_time_ratio(float f) { controller.set_time_ratio(f); }

	Transform3D get_transform() {
		return ozz_to_godot_xform(transform);
	}

	void set_time_stretch(float p_time_stretch) {
		set_playback_speed(animation->duration() * (1.f / p_time_stretch));
	}

	void set_reversed(bool p_reversed) {
		if (p_reversed) {
			set_playback_speed(abs(get_playback_speed()) * -1.f);
		} else {
			set_playback_speed(abs(get_playback_speed()));
		}
	}

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("set_joint_weights", "weights"), &OzzAnimationState::set_joint_weights, DEFVAL(PackedFloat32Array()));
		ClassDB::bind_method(D_METHOD("get_transform"), &OzzAnimationState::get_transform);
		ClassDB::bind_method(D_METHOD("get_duration"), &OzzAnimationState::get_duration);
		ClassDB::bind_method(D_METHOD("set_time_ratio", "time"), &OzzAnimationState::set_time_ratio);
		ClassDB::bind_method(D_METHOD("copy_pose", "animation_state"), &OzzAnimationState::copy_pose);
		ClassDB::bind_method(D_METHOD("set_time_stretch", "stretch"), &OzzAnimationState::set_time_stretch, DEFVAL(1.f));
		ClassDB::bind_method(D_METHOD("set_reversed", "reversed"), &OzzAnimationState::set_reversed, DEFVAL(false));
		// setgets
		ADD_SETTER(OzzAnimationState, set_weight, weight, 1.0)
		ADD_GETTER(OzzAnimationState, get_weight)
		ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "weight"), "set_weight", "get_weight");
		ADD_SETTER(OzzAnimationState, set_playback_speed, "speed", 1.0)
		ADD_GETTER(OzzAnimationState, get_playback_speed)
		ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "speed"), "set_playback_speed", "get_playback_speed");
	}
};

class OzzGD : public RefCounted {
	GDCLASS(OzzGD, RefCounted);

public:
	ozz::unique_ptr<ozz::animation::Skeleton> skeleton;
	float time = 0.0f;
	float threshold = ozz::animation::BlendingJob().threshold;

	int bones = 0;
	PackedStringArray names;
	PackedStringArray get_names() { return names; }
	TypedArray<Transform3D> rests;
	TypedArray<PackedInt32Array> children;
	PackedInt32Array parents;
	TypedArray<Transform3D> global_rests;
	TypedArray<Transform3D> bind_poses;
	PackedInt32Array binds;

	bool started = false;

	float dt = 0.f;
	float tick_time = 1.f / 30.f;
	float get_tick_time() { return tick_time; }
	void set_tick_time(float p_tick_time) { tick_time = p_tick_time; }

	ozz::vector<ozz::math::SoaTransform> from_locals;
	ozz::vector<ozz::math::SoaTransform> to_locals;
	ozz::vector<ozz::math::SoaTransform> interpolated_locals;
	ozz::vector<ozz::math::Float4x4> models;

	// For curving root motion animations
	float angular_velocity = 0.f;

	OzzGD() {
		// SceneTree* st = owner->get_tree();
		// st->
	}

	bool init(
			PackedStringArray p_names,
			TypedArray<PackedInt32Array> p_children,
			TypedArray<Transform3D> p_rests,
			TypedArray<Transform3D> p_global_rests,
			TypedArray<Transform3D> p_bind_poses,
			PackedInt32Array p_binds) {
		if (p_binds.is_empty() || p_names.is_empty() || p_children.is_empty() || p_rests.is_empty() || p_global_rests.is_empty() || p_bind_poses.is_empty()) {
			ERR_PRINT("OzzGD Init error: One input array is zero. Invalid configuration.");
			return false;
		};

		size_t bones = p_names.size();
		if (p_children.size() != bones || p_rests.size() != bones || p_global_rests.size() != bones || p_bind_poses.size() != bones) {
			ERR_PRINT("OzzGD Init error: One input array is not the same size. Invalid.");
			return false;
		}

		binds = p_binds;
		bind_poses = p_bind_poses;
		children = p_children;
		global_rests = p_global_rests;
		rests = p_rests;
		names = p_names;

		if (load_skeleton() == false) {
			return false;
		}

		if (skeleton.get() == nullptr) {
			ERR_PRINT("Ozz skeleton is null");
			return false;
		}

		from_locals.resize(skeleton->num_soa_joints());
		to_locals.resize(skeleton->num_soa_joints());
		interpolated_locals.resize(skeleton->num_soa_joints());

		models.resize(skeleton->num_joints());

		ozz::span<const ozz::math::SoaTransform> rests = skeleton->joint_rest_poses();
		from_locals.assign(rests.begin(), rests.end());
		to_locals.assign(rests.begin(), rests.end());
		interpolated_locals.assign(rests.begin(), rests.end());
		//std::fill(from_locals.begin(), from_locals.end(), ozz::math::SoaTransform::identity());
		//std::fill(to_locals.begin(), to_locals.end(), ozz::math::SoaTransform::identity());
		//std::fill(interpolated_locals.begin(), interpolated_locals.end(), ozz::math::SoaTransform::identity());

		std::fill(models.begin(), models.end(), ozz::math::Float4x4::identity());

		return true;
	}

	void update_hitboxes(Transform3D global_transform, Dictionary hitboxes) {
		// Updates the hitbox transforms
		for (int i = 0; i < hitboxes.size(); i++) {
			int bond_id = static_cast<int>(hitboxes.get_key_at_index(i));
			Object *hitbox = hitboxes[bond_id];
			if (hitbox == nullptr) {
				continue;
			}

			if (hitbox->has_method("set_body_transform")) {
				Transform3D xform = global_transform * ozz_to_godot_xform(models[bond_id]);
				hitbox->call("set_body_transform", xform);
			}
		}
	}

	// Compute rotation to apply for the given _duration
	ozz::math::Quaternion FrameRotation(float _duration) const {
		const float angle = angular_velocity * _duration;
		return ozz::math::Quaternion::FromEuler({ angle, 0, 0 });
	}

	Ref<OzzAnimationState> new_state() {
		if (skeleton.get() == nullptr) {
			ERR_PRINT("Ozz skeleton is null. Cannot create new animation state.");
			return nullptr;
		}

		// Test loading
		Ref<OzzAnimationState> as = Ref<OzzAnimationState>(memnew(OzzAnimationState));

		const int num_joints = skeleton->num_joints();
		const int num_soa_joints = skeleton->num_soa_joints();

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

		// State initially in rest pose
		as->locals.assign(skeleton->joint_rest_poses().begin(), skeleton->joint_rest_poses().end());

		return as;
	}

	Ref<OzzAnimationState> new_state_bin(PackedByteArray data) {
		if (skeleton.get() == nullptr) {
			ERR_PRINT("Ozz skeleton is null. Cannot create new animation state.");
			return nullptr;
		}

		// Test loading
		Ref<OzzAnimationState> as = new_state();

		ozz::io::MemoryStream inbuf;
		inbuf.Write((void *)data.ptr(), data.size()); // IMPORTANT: You must write to the buffer before assigning to the archive
		inbuf.Seek(0, ozz::io::MemoryStream::kSet);
		ozz::io::IArchive input(&inbuf); // Initialising the archive reads the first byte from the buffer! The endianess!

		if (!input.TestTag<ozz::animation::Animation>()) {
			return nullptr;
		}

		as->animation = ozz::make_unique<ozz::animation::Animation>();
		input >> *as->animation.get();

		// Load motion tracks
		if (input.TestTag<ozz::animation::Float3Track>()) {
			input >> as->motion_track.position;
			as->has_motion = true;
		}
		if (input.TestTag<ozz::animation::QuaternionTrack>()) {
			input >> as->motion_track.rotation;
			as->has_motion = true;
		}

		// Load event tracks
		while (input.TestTag<ozz::animation::FloatTrack>()) {
			ozz::unique_ptr<ozz::animation::FloatTrack> event_track = ozz::make_unique<ozz::animation::FloatTrack>();
			input >> *event_track.get();
			as->events.push_back(std::move(event_track));
			as->has_events = true;
		}

		return as;
	}

	void apply_animation_state(Ref<OzzAnimationState> state) {
		if (state.is_null() || skeleton == nullptr) {
			return;
		}
		if (state->locals.size() != skeleton->num_soa_joints()) {
			ERR_PRINT("OzzAnimationState tracks does not equal skeletons joints. Must be for a different rig.");
			return;
		}
		dt = 0.f;
		if (!started) {
			memcpy(to_locals.data(), state->locals.data(), state->locals.size() * sizeof(ozz::math::SoaTransform));
			started = true;
		}
		memcpy(from_locals.data(), to_locals.data(), to_locals.size() * sizeof(ozz::math::SoaTransform));
		memcpy(to_locals.data(), state->locals.data(), state->locals.size() * sizeof(ozz::math::SoaTransform));
	}

	bool play_animation(Ref<OzzAnimationState> state, float delta, bool update_cache = true, bool sample_motion = false, bool sample_events = true) {
		if (state.is_null()) {
			return false;
		}
		if (state->animation == nullptr) {
			return false;
		}

		int loops = state->controller.Update(*state->animation, delta);

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

		// Sample motion
		if (sample_motion && state->has_motion) {
			bool apply_motion_position = true;
			bool apply_motion_rotation = true;
			// Reset character transform
			state->transform = ozz::math::Float4x4::identity();

			// Get position from motion track
			if (apply_motion_position) {
				ozz::math::Float3 position;
				ozz::animation::Float3TrackSamplingJob position_sampler;
				position_sampler.track = &state->motion_track.position;
				position_sampler.result = &position;
				position_sampler.ratio = state->controller.time_ratio();
				if (!position_sampler.Run()) {
					return false;
				}

				state->transform = // Apply motion position to character transform
						state->transform * ozz::math::Float4x4::Translation(position);
			}

			// Get rotation from motion track
			if (apply_motion_rotation) {
				ozz::math::Quaternion rotation;
				ozz::animation::QuaternionTrackSamplingJob rotation_sampler;
				rotation_sampler.track = &state->motion_track.rotation;
				rotation_sampler.result = &rotation;
				rotation_sampler.ratio = state->controller.time_ratio();
				if (!rotation_sampler.Run()) {
					return false;
				}

				// Apply motion rotation to character transform
				state->transform = state->transform * ozz::math::Float4x4::FromQuaternion(ozz::math::simd_float4::LoadPtrU(&rotation.x));
			}
		}

		// Sample events
		if (sample_events && state->has_events) {
			ozz::animation::TrackTriggeringJob job;
			job.from = state->controller.previous_time_ratio();
			job.to = state->controller.time_ratio() + 0.0003f;
			job.threshold = 0.f;

			for (int i = 0; i < state->events.size(); i++) {
				ozz::unique_ptr<ozz::animation::FloatTrack> &track = state->events[i];
				job.track = track.get();
				ozz::animation::TrackTriggeringJob::Iterator iterator;
				job.iterator = &iterator;
				if (!job.Run()) {
					continue;
				}

				// Iteratively evaluates all edges.
				// Edges are lazily evaluated on iterator increments.
				for (const ozz::animation::TrackTriggeringJob::Iterator end = job.end();
						iterator != end; ++iterator) {
					const ozz::animation::TrackTriggeringJob::Edge &edge = *iterator;
					if (edge.rising) {
						String track_name = String(track->name());
						emit_signal("event_triggered", track_name);
					}
				}
			}
		}

		return true;
	}

	void blend_animations(Ref<OzzAnimationState> state, Ref<OzzAnimationState> blend_state, float blend_amount) {
		if (state.is_null() || blend_state.is_null()) {
			return;
		}
		ozz::animation::BlendingJob::Layer layers[2];

		layers[0].transform = make_span(state->locals);
		layers[0].weight = 1.0f - blend_amount; //state->weight;
		// Set per-joint weights for the partially blended layer.
		layers[0].joint_weights = make_span(state->joint_weights);

		layers[1].transform = make_span(blend_state->locals);
		layers[1].weight = blend_amount; //blend_state->weight;
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

	void blend2(Ref<OzzAnimationState> output, Ref<OzzAnimationState> base, Ref<OzzAnimationState> blend, float blend_amount) {
		if (output.is_null() || base.is_null() || blend.is_null()) {
			return;
		}
		ozz::animation::BlendingJob::Layer layers[2];

		layers[0].transform = make_span(base->locals);
		layers[0].weight = 1.0f - blend_amount;
		layers[0].joint_weights = make_span(base->joint_weights);

		layers[1].transform = make_span(blend->locals);
		layers[1].weight = blend_amount;
		layers[1].joint_weights = make_span(blend->joint_weights);

		// Setups blending job.
		ozz::animation::BlendingJob blend_job;
		blend_job.threshold = threshold;
		blend_job.layers = layers;
		blend_job.rest_pose = skeleton->joint_rest_poses();
		blend_job.output = make_span(output->locals);

		// Blends.
		if (!blend_job.Run()) {
			return;
		}
	}

	void add2(Ref<OzzAnimationState> output, Ref<OzzAnimationState> base, Ref<OzzAnimationState> add, float blend_amount, bool global = false) {
		if (output.is_null() || base.is_null() || add.is_null()) {
			return;
		}

		// Mesh space trick
		// local add: a = a + b * weight
		// this is all done on LOCALS.

		// global cant be done in local space because the
		// local to model conversion will break it
		// convert to model space
		// delta from model space
		// model delta = model pose - add pose

		// simply put: global ignores parent transforms in addition
		// to do this it counter acts the parents pose

		// mesh space rotation ONLY
		// mesh space is SUPER expensive because we need to convert to model space before blend/add

		if (global) {
			ozz::vector<ozz::math::Float4x4> base_models, models_b;
			base_models.resize(skeleton->num_joints());
			models_b.resize(skeleton->num_joints());

			// Converts from local space to model space
			ozz::animation::LocalToModelJob ltm_job;
			ltm_job.skeleton = skeleton.get();
			ltm_job.input = make_span(base->locals);
			ltm_job.output = make_span(base_models);
			if (!ltm_job.Run()) {
				return;
			}
			// ltm_job.input = make_span(add->locals);
			// ltm_job.output = make_span(models_b);
			// if (!ltm_job.Run()) {
			// 	return;
			// }

			// convert Float4x4 to SoaTransform
			// ozz::vector<ozz::math::SoaTransform> soa_base_models, models_b_soa;
			// soa_base_models.resize(skeleton->num_soa_joints());
			// models_b_soa.resize(skeleton->num_soa_joints());

			// Also zeros out translation
			// _convert_f4x4_to_soa(base_models, soa_base_models, true);
			// _convert_f4x4_to_soa(models_b, models_b_soa, true);

			// Compute model space delta
			// nullify model pose
			// delta = base - reference
			// testing if base pose is joint rests

			// I want the local pose of base to be such that when 
			// local to model is done, it becomes the rest pose.
			// The current pose of base is anything.
			// in model space we find the delta that will bring the
			// pose to its rest.
			// delta = pose * inv ref
			// where:
			// pose = identity (or rest?)
			// ref = current pose

			// ozz::vector<ozz::math::SoaTransform> identity;
			// identity.resize(skeleton->num_soa_joints());
			// std::fill(identity.begin(), identity.end(), ozz::math::SoaTransform::identity());
			//make_span(skeleton->joint_rest_poses());
			// for (int i = 0; i < skeleton->num_soa_joints(); i++) {
			// 	output->locals[i].rotation = ozz::math::Conjugate(ozz::math::Normalize(models_a_soa[i].rotation));
			// }

			// ozz::vector<ozz::math::SoaTransform> sub_locals = delta_b;
			// for (int i = 0; i < skeleton->num_soa_joints(); i++) {
			// 	sub_locals[i].translation = ozz::math::SoaFloat3::zero();
			// }
			
			// Subtract the rest poses from the model poses
			// the goal here is to isolate the model space pose
			// ozz::vector<ozz::math::SoaTransform> delta;
			// delta.resize(skeleton->num_soa_joints());
			// ozz::animation::BlendingJob::Layer layers2[1];
			// layers2[0].transform = make_span(base_models);
			// layers2[0].weight = 1.0f;
			// ozz::animation::BlendingJob::Layer additive2[1];
			// additive2[0].transform = make_span(skeleton->joint_rest_poses());
			// additive2[0].weight = -1.0f;
			// ozz::animation::BlendingJob delta_job;
			// delta_job.threshold = threshold;
			// delta_job.layers = layers2;
			// delta_job.additive_layers = additive2;
			// delta_job.rest_pose = skeleton->joint_rest_poses();
			// delta_job.output = make_span(delta);
			// if (!delta_job.Run()) return;

			// ozz::vector<ozz::math::SoaTransform> sub_locals = delta_b;
			// for (int i = 0; i < skeleton->num_soa_joints(); i++) {
			// 	sub_locals[i].translation = ozz::math::SoaFloat3::zero();
			// }

			// set joint 2 (spine) to the inverse of its parent in model space
			// lets keep it simple. replace a whole soaTransform
			ozz::span<const int16_t> parents = skeleton->joint_parents();
			for (int i = 0; i < skeleton->num_joints(); i++) {
				int block = i / 4;
				int lane = i % 4;

				int parent = parents[i];

				if (parent == ozz::animation::Skeleton::kNoParent) {
					continue;
				} else {
					ozz::math::Float4x4 parent_model_inv = ozz::math::Invert(base_models[parent]);
					output->locals[block] = single_float4x4_to_soa_transform(parent_model_inv, output->locals[block], lane);

				}
			}

			// ozz::animation::BlendingJob::Layer blend_layers[1];
			// blend_layers[0].transform = make_span(base->locals);
			// blend_layers[0].weight = 1.f;
			// blend_layers[0].joint_weights = make_span(base->joint_weights);
			// ozz::animation::BlendingJob::Layer additive[1];
			// additive[0].transform = make_span(base_models);
			// additive[0].weight = -blend_amount;
			// additive[0].joint_weights = make_span(add->joint_weights);
			// ozz::animation::BlendingJob add_job;
			// add_job.threshold = threshold;
			// add_job.layers = blend_layers;
			// add_job.additive_layers = additive;
			// add_job.rest_pose = skeleton->joint_rest_poses();
			// add_job.output = make_span(output->locals);
			// // output->locals.assign(skeleton->joint_rest_poses().begin(), skeleton->joint_rest_poses().end());
			// add_job.Run();
		} else {
			ozz::animation::BlendingJob::Layer layers[1];
			layers[0].transform = make_span(base->locals);
			layers[0].weight = 1.f; //state->weight;
			layers[0].joint_weights = make_span(base->joint_weights);

			ozz::vector<ozz::animation::BlendingJob::Layer> additive;
			additive.resize(1);

			additive[0].transform = make_span(add->locals);
			additive[0].weight = blend_amount;
			additive[0].joint_weights = make_span(add->joint_weights);

			ozz::animation::BlendingJob add_job;
			add_job.threshold = threshold;
			add_job.layers = layers;
			add_job.additive_layers = make_span(additive);
			add_job.rest_pose = skeleton->joint_rest_poses();
			add_job.output = make_span(output->locals);

			// Blends.
			if (!add_job.Run()) {
				return;
			}
		}
	}

	void _convert_f4x4_to_soa(ozz::vector<ozz::math::Float4x4> &in, ozz::vector<ozz::math::SoaTransform> &out, bool zero_translation = false) {
		const ozz::math::SimdFloat4 w_axis = ozz::math::simd_float4::w_axis();
		const ozz::math::SimdFloat4 zero = ozz::math::simd_float4::zero();
		const ozz::math::SimdFloat4 one = ozz::math::simd_float4::one();
		for (int i = 0; i < skeleton->num_soa_joints(); ++i) {
			ozz::math::SimdFloat4 translations[4];
			ozz::math::SimdFloat4 scales[4];
			ozz::math::SimdFloat4 rotations[4];

			for (int j = 0; j < 4; ++j) {
				int idx = i * 4 + j;
				if (idx < skeleton->num_joints()) {
					const ozz::math::Float4x4 m = in[idx];
					ozz::math::ToAffine(m, &translations[j], &rotations[j], &scales[j]);
				} else {
					translations[j] = zero;
					rotations[j] = w_axis;
					scales[j] = one;
				}
				if (zero_translation) {
					translations[j] = zero; // RIORDANS NOTE: zero translation. Used in mesh space blends we dont want to mess with translation.
				}
			}
			// Fills the SoaTransform structure.
			ozz::math::Transpose4x3(translations, &out[i].translation.x);
			ozz::math::Transpose4x4(rotations, &out[i].rotation.x);
			ozz::math::Transpose4x3(scales, &out[i].scale.x);
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

	bool load_skeleton() {
		// Creates a RawSkeleton.
		ozz::animation::offline::RawSkeleton raw_skeleton;

		raw_skeleton.roots.resize(1);
		ozz::animation::offline::RawSkeleton::Joint &root = raw_skeleton.roots[0];
		_add_children_to_joint(0, &root);

		// Test for skeleton validity.
		// The main invalidity reason is the number of joints, which must be lower
		// than ozz::animation::Skeleton::kMaxJoints.
		if (!raw_skeleton.Validate()) {
			ERR_PRINT("Ozz could not build a valid skeleton!");
			return false;
		}

		// converts the RawSkeleton to a runtime Skeleton.
		// Creates a SkeletonBuilder instance.
		ozz::animation::offline::SkeletonBuilder builder;

		// Executes the builder on the previously prepared RawSkeleton, which returns
		// a new runtime skeleton instance.
		// This operation will fail and return an empty unique_ptr if the RawSkeleton
		// isn't valid.
		skeleton = builder(raw_skeleton);
		if (skeleton.get() == nullptr) {
			ERR_PRINT("Skeleton failed to build");
			return false;
		}

		// Just load from resource
		if (bind_poses.size() == 0) {
			WARN_PRINT("No bind poses provided. Building skin. Consider passing precomputed binds to prevent this computation at runtime.");
			bind_poses = build_skin();
		}
		return true;
	}

	int find_bone(String bone_name) {
		for (int i = 0; i < names.size(); i++) {
			if (names[i] == bone_name) {
				return i;
			}
		}
		return -1;
	}

	// TODO: Add error logs and error checks
	PackedByteArray convert_animation_to_ozz(Ref<Animation> animation, bool extract_motion, int root_bone = 0, bool use_scale = false, bool delta = false) {
		PackedByteArray data;
		ozz::io::MemoryStream buf;
		ozz::io::OArchive output(&buf);

		ozz::animation::offline::RawAnimation raw_animation = _load_animation(animation, use_scale);

		// Test for animation validity. These are the errors that could invalidate
		// an animation:
		//  1. Animation duration is less than 0.
		//  2. Keyframes' are not sorted in a strict ascending order.
		//  3. Keyframes' are not within [0, duration] range.
		if (!raw_animation.Validate()) {
			ERR_PRINT("Ozz Animation produced invalid animation. Check this code for possible issues");
			return data;
		}

		if (delta) { // assumes first frame is reference pose to be delta against
			ozz::animation::offline::AdditiveAnimationBuilder aab;
			ozz::animation::offline::RawAnimation original = raw_animation; // assume copy?
			aab(original, &raw_animation);
		}

		// converts the RawAnimation to a runtime Animation.
		// Creates a AnimationBuilder instance.
		ozz::animation::offline::AnimationBuilder builder;

		if (extract_motion) {
			// Extraction motion tracks
			ozz::animation::offline::RawAnimation motion_animation;
			ozz::animation::offline::MotionExtractor motion_extractor;
			motion_extractor.root_joint = root_bone;
			// TODO: Add option to make extraction rotation/position loop-able
			// Raw motion tracks extraction
			ozz::animation::offline::RawFloat3Track raw_motion_position;
			ozz::animation::offline::RawQuaternionTrack raw_motion_rotation;
			if (!motion_extractor(raw_animation, *skeleton.get(), &raw_motion_position,
						&raw_motion_rotation, &motion_animation)) {
				return data;
			}

			{
				ozz::unique_ptr<ozz::animation::Animation> ozz_animation = builder(motion_animation);
				if (ozz_animation.get() == nullptr) {
					ERR_PRINT("Failed to convert animation to ozz animation");
					return data;
				}
				output << *ozz_animation.get();
			}

			{ // Track optimization and runtime building
				ozz::animation::offline::TrackOptimizer optimizer;
				ozz::animation::offline::RawFloat3Track raw_track_position_opt;
				if (!optimizer(raw_motion_position, &raw_track_position_opt)) {
					return data;
				}

				ozz::animation::offline::RawQuaternionTrack raw_track_rotation_opt;
				if (!optimizer(raw_motion_rotation, &raw_track_rotation_opt)) {
					return data;
				}

				// Build runtime tracks
				ozz::animation::offline::TrackBuilder track_builder;
				auto position_track = track_builder(raw_track_position_opt);
				auto rotation_track = track_builder(raw_track_rotation_opt);
				if (!position_track || !rotation_track) {
					return data;
				}
				//motion_track.position = std::move(*position_track);
				//motion_track.rotation = std::move(*rotation_track);

				output << *position_track.get();
				output << *rotation_track.get();
			}
		} else {
			ozz::unique_ptr<ozz::animation::Animation> ozz_animation = builder(raw_animation);
			if (ozz_animation.get() == nullptr) {
				ERR_PRINT("Failed to convert animation to ozz animation");
				return data;
			}
			output << *ozz_animation.get();
		}

		// Event tracks
		ozz::vector<ozz::animation::offline::RawFloatTrack> raw_event_tracks;
		for (int i = 0; i < animation->get_track_count(); i++) {
			Animation::TrackType type = animation->track_get_type(i);
			NodePath path = animation->track_get_path(i);
			if (type == Animation::TYPE_METHOD) {
				int keys = animation->track_get_key_count(i);
				for (int j = 0; j < keys; j++) {
					StringName method_name = animation->method_track_get_name(i, j);
					if (method_name == "EVENT") {
						Vector<Variant> args = animation->method_track_get_params(i, j);
						if (args.size() > 0) {
							Variant event_name = args[0];
							if (event_name.get_type() == Variant::Type::STRING) {
								String event_name_string = static_cast<String>(event_name);
								// Find/create event track
								ozz::animation::offline::RawFloatTrack raw_event_track;
								for (ozz::animation::offline::RawFloatTrack raw_track : raw_event_tracks) {
									if (strcmp(raw_track.name.c_str(), event_name_string.utf8().get_data())) {
										raw_event_track = raw_track;
									}
								}
								ozz::animation::offline::RawTrackKeyframe<float> frame;
								frame.value = 1.f;
								frame.ratio = CLAMP(animation->track_get_key_time(i, j) / animation->get_length(), 0.0002f, animation->get_length());
								ozz::animation::offline::RawTrackKeyframe<float> low;
								low.value = 0.f;
								low.ratio = frame.ratio - 0.0001f;
								raw_event_track.keyframes.push_back(low);
								raw_event_track.keyframes.push_back(frame);

								if (raw_event_track.name.empty()) {
									raw_event_track.name = ozz::string(event_name_string.utf8().get_data());
									raw_event_tracks.push_back(raw_event_track);
								}
							}
						}
					}
				}
			}
		}
		for (const ozz::animation::offline::RawFloatTrack track : raw_event_tracks) {
			ozz::animation::offline::TrackBuilder track_builder;
			auto event_track = track_builder(track);
			if (!event_track) {
				continue;
			}
			output << *event_track.get();
		}

		buf.Seek(0, ozz::io::MemoryStream::kSet);
		for (int i = 0; i < buf.Size(); i++) {
			unsigned char byte;
			buf.Read((void *)&byte, 1);
			data.push_back(byte);
		}

		return data;
	}

	ozz::animation::offline::RawAnimation _load_animation(Ref<Animation> animation, bool use_scale = false) {
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
						if (use_scale) {
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

		// Optimize
		ozz::animation::offline::AnimationOptimizer optimizer;
		ozz::animation::offline::AnimationOptimizer::Setting setting;

		// Setup global optimization settings.
		optimizer.setting = setting;

		// Setup joint specific optimization settings.
		/*
		if (joint_setting_enable_) {
			optimizer.joints_setting_override[joint_] = joint_setting_;
		}
		*/
		ozz::animation::offline::RawAnimation raw_optimized_animation;

		if (!optimizer(raw_animation, *skeleton, &raw_optimized_animation)) {
			return raw_animation;
		}

		return raw_optimized_animation;
	}

	void update_skeleton_interpolated(float delta, RID skeleton_rid, RID visibility_notifier_rid) {
		dt += delta;
		float tick_factor = CLAMP(dt / tick_time, 0.0, 1.0);

		if (from_locals.size() != skeleton->num_soa_joints()) {
			from_locals.resize(skeleton->num_soa_joints());
			to_locals.resize(skeleton->num_soa_joints());
			interpolated_locals.resize(skeleton->num_soa_joints());
			models.resize(skeleton->num_joints());
		}

		ozz::animation::BlendingJob::Layer layers[2];
		layers[0].transform = make_span(from_locals);
		layers[0].weight = 1.0 - tick_factor;
		layers[1].transform = make_span(to_locals);
		layers[1].weight = tick_factor;

		// Setups blending job.
		ozz::animation::BlendingJob blend_job;
		blend_job.threshold = threshold;
		blend_job.layers = layers;
		blend_job.rest_pose = skeleton->joint_rest_poses();
		blend_job.output = make_span(interpolated_locals);

		// Blends.
		if (!blend_job.Run()) {
			return;
		}

		// Converts from local space to model space matrices.
		ozz::animation::LocalToModelJob ltm_job;
		ltm_job.skeleton = skeleton.get();
		ltm_job.input = make_span(interpolated_locals);
		ltm_job.output = make_span(models);
		if (!ltm_job.Run()) {
			return;
		}

		AABB aabb = AABB();
		int bind_count = binds.size();
		for (int i = 0; i < bind_count; i++) {
			// Only update the bones that are bound
			int bone = binds[i];
			ozz::math::Float4x4 m = models[bone];
			Transform3D pose = Transform3D(
					ozz::math::GetX(m.cols[0]), ozz::math::GetX(m.cols[1]), ozz::math::GetX(m.cols[2]),
					ozz::math::GetY(m.cols[0]), ozz::math::GetY(m.cols[1]), ozz::math::GetY(m.cols[2]),
					ozz::math::GetZ(m.cols[0]), ozz::math::GetZ(m.cols[1]), ozz::math::GetZ(m.cols[2]),
					ozz::math::GetX(m.cols[3]), ozz::math::GetY(m.cols[3]), ozz::math::GetZ(m.cols[3])

			);
			aabb = aabb.expand(pose.origin);
			RenderingServer::get_singleton()->skeleton_bone_set_transform(skeleton_rid, i, pose * static_cast<Transform3D>(bind_poses[bone]));
		}
		RenderingServer::get_singleton()->visibility_notifier_set_aabb(visibility_notifier_rid, aabb);
	}

	void update_skeleton(RID skeleton_rid, RID visibility_notifier_rid) {
		// Converts from local space to model space matrices.
		ozz::animation::LocalToModelJob ltm_job;
		ltm_job.skeleton = skeleton.get();
		ltm_job.input = make_span(to_locals);
		ltm_job.output = make_span(models);
		if (!ltm_job.Run()) {
			return;
		}

		AABB aabb = AABB();
		int bind_count = binds.size();
		for (int i = 0; i < bind_count; i++) {
			// Only update the bones that are bound
			int bone = binds[i];
			ozz::math::Float4x4 m = models[bone];
			Transform3D pose = ozz_to_godot_xform(m);
			aabb = aabb.expand(pose.origin);
			RenderingServer::get_singleton()->skeleton_bone_set_transform(skeleton_rid, i, pose * static_cast<Transform3D>(bind_poses[bone]));
		}
		RenderingServer::get_singleton()->visibility_notifier_set_aabb(visibility_notifier_rid, aabb);
	}

	void update_skeleton_ragdoll(Transform3D global_transform, Dictionary bodies, RID skeleton_rid, RID visibility_notifier_rid) {
		// Updates the skeleton but uses the poses of the hitboxes
		Transform3D global_transform_inv = global_transform.inverse();
		TypedArray<int> q;
		q.append(0);

		HashMap<int, Transform3D> poses;
		for (int i = 0; i < rests.size(); i++) {
			poses[i] = static_cast<Transform3D>(rests[i]);
		}

		// We want to loop through all the binds
		// For every bound bone we want to check if
		while (q.size() > 0) {
			int bone = q.pop_front();
			Transform3D bone_pose = poses[bone];

			if (bodies.has(bone)) {
				RID body_rid = static_cast<RID>(bodies[bone]);
				Transform3D body_transform = PhysicsServer3D::get_singleton()->body_get_state(body_rid, PhysicsServer3D::BODY_STATE_TRANSFORM);
				bone_pose = global_transform_inv * body_transform;
				poses[bone] = bone_pose;
			}

			Array bone_children = static_cast<Array>(children[bone]);
			for (int i = 0; i < bone_children.size(); i++) {
				int child_bone = bone_children[i];
				if (!bodies.has(child_bone)) {
					poses[child_bone] = bone_pose * static_cast<Transform3D>(rests[child_bone]);
				}
			}
			q.append_array(bone_children);
		}

		AABB aabb = AABB();
		int bind_count = binds.size();
		for (int i = 0; i < bind_count; i++) {
			// Only update the bones that are bound
			int bone = binds[i];
			Transform3D pose = poses[bone];
			aabb = aabb.expand(pose.origin);
			RenderingServer::get_singleton()->skeleton_bone_set_transform(skeleton_rid, i, pose * static_cast<Transform3D>(bind_poses[bone]));
		}
		RenderingServer::get_singleton()->visibility_notifier_set_aabb(visibility_notifier_rid, aabb);
	}

	Array get_parentless_bones() {
		Array bones;
		for (int i = 0; i < parents.size(); i++) {
			if (parents[i] == -1) {
				bones.append(i);
			}
		}
		return bones;
	}

	// The point of the skin is to map bones to weight indices
	// Here we create the inverse bind matrix
	// In the skin shader we use the inverse bind matrix to compute local bone pose
	TypedArray<Transform3D> build_skin() {
		TypedArray<Transform3D> bind_poses;
		bind_poses.resize(bones);
		Array bones_to_process = get_parentless_bones();

		while (bones_to_process.size() > 0) {
			int current_bone_idx = bones_to_process.pop_front();
			Array child_bones = Array(children[current_bone_idx]);
			int parent = parents[current_bone_idx];
			if (parent < 0) {
				bind_poses[current_bone_idx] = rests[current_bone_idx];
			}

			for (int i = 0; i < child_bones.size(); i++) {
				int child_bone_idx = child_bones[i];
				Transform3D bind = bind_poses[current_bone_idx];
				Transform3D rest = rests[child_bone_idx];
				bind_poses[child_bone_idx] = bind * rest;
			}
			bones_to_process.append_array(child_bones);
		}

		for (int i = 0; i < bind_poses.size(); i++) {
			Transform3D pose = bind_poses[i];
			bind_poses[i] = pose.affine_inverse();
		}

		return bind_poses;
	}

	Transform3D get_bone_model_pose(int bone_id) {
		if (bone_id < 0 || bone_id > models.size()) {
			return Transform3D();
		}
		return ozz_to_godot_xform(models[bone_id]);
	}

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("init", "names", "children", "rests", "global_rests", "bind_poses", "binds"), &OzzGD::init);
		ClassDB::bind_method(D_METHOD("play_animation", "state", "delta", "update_state", "sample_motion", "sample_events"), &OzzGD::play_animation);
		ClassDB::bind_method(D_METHOD("get_names"), &OzzGD::get_names);
		ClassDB::bind_method(D_METHOD("new_state"), &OzzGD::new_state);
		ClassDB::bind_method(D_METHOD("new_state_bin", "data"), &OzzGD::new_state_bin);
		ClassDB::bind_method(D_METHOD("blend2", "output", "a", "b", "blend_amount"), &OzzGD::blend2);
		ClassDB::bind_method(D_METHOD("add2", "output", "a", "b", "blend_amount", "global"), &OzzGD::add2, DEFVAL(false));
		ClassDB::bind_method(D_METHOD("blend_animations", "state", "blend_state", "blend_amount"), &OzzGD::blend_animations);
		ClassDB::bind_method(D_METHOD("convert_animation_to_ozz", "animation", "extract_motion", "root_bone", "use_scale", "delta"), &OzzGD::convert_animation_to_ozz, DEFVAL(NULL), DEFVAL(false), DEFVAL(0), DEFVAL(false));
		ClassDB::bind_method(D_METHOD("apply_animation_state", "animation_state"), &OzzGD::apply_animation_state);
		ClassDB::bind_method(D_METHOD("update_skeleton", "skeleton_rid", "visibility_notifier_rid"), &OzzGD::update_skeleton);
		ClassDB::bind_method(D_METHOD("update_skeleton_interpolated", "delta", "skeleton_rid", "visibility_notifier_rid"), &OzzGD::update_skeleton_interpolated);
		ClassDB::bind_method(D_METHOD("get_tick_time"), &OzzGD::get_tick_time);
		ClassDB::bind_method(D_METHOD("set_tick_time", "p_tick_time"), &OzzGD::set_tick_time);
		ClassDB::bind_method(D_METHOD("get_bone_model_pose", "bone_id"), &OzzGD::get_bone_model_pose);
		ClassDB::add_property(
				"OzzGD",
				PropertyInfo(Variant::FLOAT, "tick_time", PROPERTY_HINT_NONE, "", PROPERTY_USAGE_DEFAULT),
				"set_tick_time",
				"get_tick_time");
		ClassDB::bind_method(D_METHOD("update_hitboxes", "global_transform", "hitboxes"), &OzzGD::update_hitboxes);
		ClassDB::bind_method(D_METHOD("update_skeleton_ragdoll", "global_transform", "bodies", "skeleton_rid", "visibility_rid"), &OzzGD::update_skeleton_ragdoll);
		ClassDB::add_signal("OzzGD", MethodInfo("event_triggered", PropertyInfo(Variant::STRING, "event_name")));
	}
};

struct OzzBlendPoint {
	Ref<OzzAnimationState> animation_state;
	Vector2 position = Vector2();
};

typedef struct OzzBlendPoint OzzBlendPoint;

class OzzBlendSpace2D : public RefCounted {
	GDCLASS(OzzBlendSpace2D, RefCounted);

public:
	std::vector<OzzBlendPoint> blend_points;
	Array triangles;

	Array get_triangles() { return triangles; }
	void set_triangles(Array p_triangles) { triangles = p_triangles; }

	OzzBlendSpace2D() {
		triangles = Array();
	}

	~OzzBlendSpace2D() {
	}

	int add_blend_point(Ref<OzzAnimationState> as, Vector2 p_position) {
		blend_points.push_back({ as, p_position });
		return blend_points.size() - 1;
	}

	void replace(int index, Ref<OzzAnimationState> as) {
		if (index >= blend_points.size() || index < 0) {
			return;
		}
		blend_points[index].animation_state = as;
	}

	void update(OzzGD *ozz, float delta) {
		// Update all animations in sync
		for (const auto &bp : blend_points) {
			// Advance time but dont update cache for all
			ozz->play_animation(bp.animation_state, delta, false, false, false);
		}
	}

	Ref<OzzAnimationState> get_animation_state(OzzGD *ozz, Vector2 p_position) {
		p_position.x = CLAMP(p_position.x, -1.0f, 1.0f);
		p_position.y = CLAMP(p_position.y, -1.0f, 1.0f);

		bool first = false;
		Vector2 best_point = Vector2(INFINITY, INFINITY);
		Array best_tri;
		Array res;
		float blend_weights[3] = { 0.f, 0.f, 0.f };

		// Find blend triangle
		for (int i = 0; i < triangles.size(); i++) {
			Array tri = triangles[i];
			// HANDLE CASE WHERE BLEND_POS IS ON POINT
			for (int x = 0; x < tri.size(); x++) {
				int index = (int)tri[x];
				if (p_position.distance_to(blend_points[index].position) <= CMP_EPSILON) {
					Ref<OzzAnimationState> as = blend_points[index].animation_state;
					ozz->play_animation(as, 0.0, true);
					return as;
				}
			}

			if (tri.size() != 3) {
				continue; // FUTURE: Handle case where we create a simple line?
			}

			// HANDLE CASE WHERE BLEND_POS IS INSIDE TRIANGLE
			// Vector2 centroid = (blend_points[(int)tri[0]].position + blend_points[(int)tri[1]].position + blend_points[(int)tri[2]].position) / 3.0;
			Vector2 dir0; // = centroid.direction_to(blend_points[(int)tri[0]].position) * CMP_EPSILON;
			Vector2 dir1; // = centroid.direction_to(blend_points[(int)tri[1]].position) * CMP_EPSILON;
			Vector2 dir2; // = centroid.direction_to(blend_points[(int)tri[2]].position) * CMP_EPSILON;
			if (Geometry2D::is_point_in_triangle(p_position, blend_points[(int)tri[0]].position + dir0, blend_points[(int)tri[1]].position + dir1, blend_points[(int)tri[2]].position + dir2)) {
				Ref<OzzAnimationState> states[3] = {
					blend_points[(int)tri[0]].animation_state, // You dont need to copy. This was done to remove the cached values but these are overriden when play_animation is called with update_cache = true
					blend_points[(int)tri[1]].animation_state,
					blend_points[(int)tri[2]].animation_state
				};

				Vector2 positions[3] = {
					blend_points[(int)tri[0]].position,
					blend_points[(int)tri[1]].position,
					blend_points[(int)tri[2]].position
				};

				if (p_position.distance_squared_to(positions[0]) <= CMP_EPSILON) {
					ozz->play_animation(states[0], 0.0, true);
					return states[0];
				}

				if (p_position.distance_squared_to(positions[1]) <= CMP_EPSILON) {
					ozz->play_animation(states[1], 0.0, true);
					return states[1];
				}

				if (p_position.distance_squared_to(positions[2]) <= CMP_EPSILON) {
					ozz->play_animation(states[2], 0.0, true);
					return states[2];
				}

				Vector2 v0 = positions[1] - positions[0];
				Vector2 v1 = positions[2] - positions[0];
				Vector2 v2 = p_position - positions[0];

				real_t d00 = v0.dot(v0);
				real_t d01 = v0.dot(v1);
				real_t d11 = v1.dot(v1);
				real_t d20 = v2.dot(v0);
				real_t d21 = v2.dot(v1);
				real_t denom = (d00 * d11 - d01 * d01);
				if (denom == 0) {
					ozz->play_animation(states[0], 0.0, true);
					return states[0];
				}

				real_t v = (d11 * d20 - d01 * d21) / denom;
				real_t w = (d00 * d21 - d01 * d20) / denom;
				real_t u = 1.0 - v - w;

				ozz->play_animation(states[0], 0.0, true);
				ozz->play_animation(states[1], 0.0, true);
				ozz->play_animation(states[2], 0.0, true);

				ozz->blend_animations(states[0], states[1], v);
				ozz->blend_animations(states[0], states[2], w);

				return states[0];
			}

			// HANDLE CASE WHERE BLEND_POS IS OUTSIDE OF ALL TRIANGLES
			// Get closest segment
			for (int j = 0; j < 3; j++) {
				Vector2 segment_a = blend_points[(int)tri[j]].position;
				Vector2 segment_b = blend_points[(int)tri[(j + 1) % 3]].position;
				Vector2 closest = Geometry2D::get_closest_point_to_segment(p_position, segment_a, segment_b);
				if (first || closest.distance_to(p_position) < best_point.distance_to(p_position)) {
					best_point = closest;
					first = false;
					best_tri = tri;
					float d = segment_a.distance_to(segment_b);
					if (d == 0.0) {
						blend_weights[j] = 1.0;
						blend_weights[(j + 1) % 3] = 0.0;
						blend_weights[(j + 2) % 3] = 0.0;
					} else {
						float c = segment_a.distance_to(closest) / d;

						blend_weights[j] = 1.0 - c;
						blend_weights[(j + 1) % 3] = c;
						blend_weights[(j + 2) % 3] = 0.0;
					}
				}
			}
		}

		// If here, must be outside triangle case
		Ref<OzzAnimationState> states[3] = {
			blend_points[(int)best_tri[0]].animation_state, // You dont need to copy. This was done to remove the cached values but these are overriden when play_animation is called with update_cache = true
			blend_points[(int)best_tri[1]].animation_state,
			blend_points[(int)best_tri[2]].animation_state
		};

		if (Math::is_equal_approx(blend_weights[0], 1.0f)) {
			ozz->play_animation(states[0], 0.0, true);
			return states[0];
		}

		if (Math::is_equal_approx(blend_weights[1], 1.0f)) {
			ozz->play_animation(states[1], 0.0, true);
			return states[1];
		}

		if (Math::is_equal_approx(blend_weights[2], 1.0f)) {
			ozz->play_animation(states[2], 0.0, true);
			return states[2];
		}

		Ref<OzzAnimationState> state;

		if (blend_weights[0] > 0.0 && blend_weights[1] > 0.0) {
			state = states[0];
			ozz->play_animation(state, 0.0, true);
			ozz->play_animation(states[1], 0.0, true);
			ozz->blend_animations(state, states[1], blend_weights[1]);
		}

		if (blend_weights[0] > 0.0 && blend_weights[2] > 0.0) {
			state = states[0];
			ozz->play_animation(state, 0.0, true);
			ozz->play_animation(states[2], 0.0, true);
			ozz->blend_animations(state, states[2], blend_weights[2]);
		}

		if (blend_weights[1] > 0.0 && blend_weights[2] > 0.0) {
			state = states[1];
			ozz->play_animation(state, 0.0, true);
			ozz->play_animation(states[2], 0.0, true);
			ozz->blend_animations(state, states[2], blend_weights[2]);
		}

		if (state.is_null()) {
			OS::get_singleton()->printerr("AnimationState is null. I thought all cases were covered? This should never happen!");
			return states[0];
		}

		return state;
	}

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("replace", "index", "animation_state"), &OzzBlendSpace2D::replace);
		ClassDB::bind_method(D_METHOD("add_blend_point", "animation", "point"), &OzzBlendSpace2D::add_blend_point);
		ClassDB::bind_method(D_METHOD("get_animation_state", "ozz", "position"), &OzzBlendSpace2D::get_animation_state);
		ClassDB::bind_method(D_METHOD("update", "ozz", "delta"), &OzzBlendSpace2D::update, DEFVAL(0.f));
		ClassDB::bind_method(D_METHOD("set_triangles", "triangles"), &OzzBlendSpace2D::set_triangles, DEFVAL(Array()));
		ClassDB::bind_method(D_METHOD("get_triangles"), &OzzBlendSpace2D::get_triangles);
		ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "triangles"), "set_triangles", "get_triangles");
	}
};

#endif
