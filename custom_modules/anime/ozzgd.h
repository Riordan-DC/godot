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

#include <stdalign.h>

#include <cmath>
#include <unordered_map>
#include <vector>

#ifdef DEBUG_ENABLED
// Turn off optimisation for debugging
#pragma optimize("", off)
#endif

class OzzAnimationState;
class OzzGD;

class OzzAnimationState : public RefCounted {
	GDCLASS(OzzAnimationState, RefCounted);

public:
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

	OzzGD() {}

	bool init(
			PackedStringArray p_names,
			TypedArray<PackedInt32Array> p_children,
			TypedArray<Transform3D> p_rests,
			TypedArray<Transform3D> p_global_rests,
			TypedArray<Transform3D> p_bind_poses,
			PackedInt32Array p_binds);

	void update_hitboxes(Transform3D global_transform, Dictionary hitboxes);

	// Compute rotation to apply for the given _duration
	ozz::math::Quaternion FrameRotation(float _duration) const;

	Ref<OzzAnimationState> new_state();

	Ref<OzzAnimationState> new_state_bin(PackedByteArray data);

	void apply_animation_state(Ref<OzzAnimationState> state);

	bool play_animation(Ref<OzzAnimationState> state, float delta, bool update_cache = true, bool sample_motion = false, bool sample_events = true);

	void blend2(Ref<OzzAnimationState> output, Ref<OzzAnimationState> base, Ref<OzzAnimationState> blend, float blend_amount);

	void add2(Ref<OzzAnimationState> output, Ref<OzzAnimationState> base, Ref<OzzAnimationState> add, float blend_amount, int global_bone = -1);

	void _convert_f4x4_to_soa(ozz::vector<ozz::math::Float4x4> &in, ozz::vector<ozz::math::SoaTransform> &out, bool zero_translation = false);

	void _add_children_to_joint(int bone_idx, ozz::animation::offline::RawSkeleton::Joint *joint);

	bool load_skeleton();

	int find_bone(String bone_name);

	// TODO: Add error logs and error checks
	PackedByteArray convert_animation_to_ozz(Ref<Animation> animation, bool extract_motion, int root_bone = 0, bool use_scale = false, bool delta = false, bool optimize = true);

	ozz::animation::offline::RawAnimation _load_animation(Ref<Animation> animation, bool use_scale = false, bool optimize = true);

	void update_skeleton_interpolated(float delta, RID skeleton_rid, RID visibility_notifier_rid);

	void update_skeleton(RID skeleton_rid, RID visibility_notifier_rid);

	void update_skeleton_ragdoll(Transform3D global_transform, Dictionary bodies, RID skeleton_rid, RID visibility_notifier_rid);

	Array get_parentless_bones();
	// The point of the skin is to map bones to weight indices
	// Here we create the inverse bind matrix
	// In the skin shader we use the inverse bind matrix to compute local bone pose
	TypedArray<Transform3D> build_skin();

	Transform3D get_bone_model_pose(int bone_id);

	static void _bind_methods();
};

struct OzzBlendPoint {
	Ref<OzzAnimationState> animation_state;
	Vector2 position = Vector2();
};

typedef struct OzzBlendPoint OzzBlendPoint;

class OzzBlendSpace2D : public RefCounted {
	GDCLASS(OzzBlendSpace2D, RefCounted);

public:
	Ref<OzzAnimationState> output_pose;

	Ref<OzzAnimationState> get_pose() { return output_pose; }

	std::vector<OzzBlendPoint> blend_points;
	Array triangles;
	bool sample_motion = false;
	bool sample_events = true;

	Array get_triangles() { return triangles; }
	void set_triangles(Array p_triangles) { triangles = p_triangles; }

	OzzBlendSpace2D() {
		triangles = Array();
	}

	~OzzBlendSpace2D() {
	}

	void init(OzzGD* ozz) {
		output_pose = ozz->new_state();
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
		if (output_pose == nullptr) {
			ERR_PRINT("call init() on blend space before use!");
			output_pose = ozz->new_state();
		}
		Ref<OzzAnimationState> out = _get_animation_state(ozz, p_position);
		output_pose->copy_pose(out);
		return output_pose;
	}

	inline Ref<OzzAnimationState> _get_animation_state(OzzGD *ozz, Vector2 p_position) {
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
			// handle case where blend_pos is on point
			for (int x = 0; x < tri.size(); x++) {
				int index = (int)tri[x];
				if (p_position.distance_to(blend_points[index].position) <= CMP_EPSILON) {
					Ref<OzzAnimationState> as = blend_points[index].animation_state;
					ozz->play_animation(as, 0.0, true, sample_motion, sample_events);
					return as;
				}
			}

			if (tri.size() != 3) {
				continue; // FUTURE: Handle case where we create a simple line?
			}

			// handle case where blend_pos is inside triangle
			if (Geometry2D::is_point_in_triangle(p_position, blend_points[(int)tri[0]].position, blend_points[(int)tri[1]].position, blend_points[(int)tri[2]].position)) {
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
					ozz->play_animation(states[0], 0.0, true, sample_motion, sample_events);
					return states[0];
				}

				if (p_position.distance_squared_to(positions[1]) <= CMP_EPSILON) {
					ozz->play_animation(states[1], 0.0, true, sample_motion, sample_events);
					return states[1];
				}

				if (p_position.distance_squared_to(positions[2]) <= CMP_EPSILON) {
					ozz->play_animation(states[2], 0.0, true, sample_motion, sample_events);
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
					ozz->play_animation(states[0], 0.0, true, sample_motion, sample_events);
					return states[0];
				}

				real_t v = (d11 * d20 - d01 * d21) / denom;
				real_t w = (d00 * d21 - d01 * d20) / denom;
				real_t u = 1.0 - v - w;

				ozz->play_animation(states[0], 0.0, true, false, false);
				ozz->play_animation(states[1], 0.0, true, sample_motion, sample_events);
				ozz->play_animation(states[2], 0.0, true, false, false);

				ozz->blend2(states[0], states[0], states[1], v);
				ozz->blend2(states[0], states[0], states[2], w);

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
			ozz->play_animation(states[0], 0.0, true, sample_motion, sample_events);
			return states[0];
		}

		if (Math::is_equal_approx(blend_weights[1], 1.0f)) {
			ozz->play_animation(states[1], 0.0, true, sample_motion, sample_events);
			return states[1];
		}

		if (Math::is_equal_approx(blend_weights[2], 1.0f)) {
			ozz->play_animation(states[2], 0.0, true, sample_motion, sample_events);
			return states[2];
		}

		Ref<OzzAnimationState> state;

		if (blend_weights[0] > 0.0 && blend_weights[1] > 0.0) {
			state = states[0];
			ozz->play_animation(state, 0.0, true, sample_motion, sample_events);
			ozz->play_animation(states[1], 0.0, true, false, false);
			ozz->blend2(state, state, states[1], blend_weights[1]);
		}

		if (blend_weights[0] > 0.0 && blend_weights[2] > 0.0) {
			state = states[0];
			ozz->play_animation(state, 0.0, true, sample_motion, sample_events);
			ozz->play_animation(states[2], 0.0, true, false, false);
			ozz->blend2(state, state, states[2], blend_weights[2]);
		}

		if (blend_weights[1] > 0.0 && blend_weights[2] > 0.0) {
			state = states[1];
			ozz->play_animation(state, 0.0, true, sample_motion, sample_events);
			ozz->play_animation(states[2], 0.0, true, false, false);
			ozz->blend2(state, state, states[2], blend_weights[2]);
		}

		if (state.is_null()) {
			OS::get_singleton()->printerr("AnimationState is null. I thought all cases were covered? This should never happen!");
			return states[0];
		}

		return state;
	}

	static void _bind_methods() {
		ClassDB::bind_method(D_METHOD("init"), &OzzBlendSpace2D::init);
		ClassDB::bind_method(D_METHOD("replace", "index", "animation_state"), &OzzBlendSpace2D::replace);
		ClassDB::bind_method(D_METHOD("add_blend_point", "animation", "point"), &OzzBlendSpace2D::add_blend_point);
		ClassDB::bind_method(D_METHOD("get_animation_state", "ozz", "position"), &OzzBlendSpace2D::get_animation_state);
		ClassDB::bind_method(D_METHOD("update", "ozz", "delta"), &OzzBlendSpace2D::update, DEFVAL(0.f));
		ClassDB::bind_method(D_METHOD("set_triangles", "triangles"), &OzzBlendSpace2D::set_triangles, DEFVAL(Array()));
		ClassDB::bind_method(D_METHOD("get_triangles"), &OzzBlendSpace2D::get_triangles);
		ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "triangles"), "set_triangles", "get_triangles");
		ClassDB::bind_method(D_METHOD("get_pose"), &OzzBlendSpace2D::get_pose);
		ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "pose"), "", "get_pose");
	}
};

class OzzAimOffset : public RefCounted {
	GDCLASS(OzzAimOffset, RefCounted);

public:
	Ref<OzzAnimationState> output_pose;
	Ref<OzzAnimationState> x_pose;
	Ref<OzzAnimationState> y_pose;
	std::vector<Ref<OzzAnimationState>> poses;

	OzzAimOffset() {}

	int load(Ref<OzzGD> ozz, Ref<OzzAnimationState> animation) {
		// must assert there are 10 frames
		// frame 0: reference pose
		// frame 1: down left
		// frame 2: down front
		// frame 3: down right
		// frame 4: forward left
		// frame 5: forward front
		// frame 6: forward right
		// frame 7: up left
		// frame 8: up front
		// frame 9: up right
		ozz::span<const float> timepoints = animation->animation->timepoints();
		if (timepoints.size() != 10) {
			return -1;
		}

		// Sample poses at each timepoint. Copy the locals into their own state
		poses.resize(9);
		output_pose = ozz->new_state();
		x_pose = ozz->new_state();
		y_pose = ozz->new_state();
		for (int i = 0; i < 9; i++) {
			poses[i] = ozz->new_state();
			ozz::animation::SamplingJob sampling_job;
			sampling_job.animation = animation->animation.get();
			sampling_job.context = &animation->context;
			sampling_job.ratio = timepoints[i+1];
			sampling_job.output = make_span(poses[i]->locals);

			// Samples animation.
			if (!sampling_job.Run()) {
				return -2;
			}
		}

		return 0;
	}

	Ref<OzzAnimationState> sample(Ref<OzzGD> ozz, Vector2 pos) {
		if (poses.size() != 9) {
			return nullptr;
		}
		// 0 = reference
		// 1,2,3 = down
		// 4,5,6 = forward
		// 7,8,9 = up
		// separate x and y blends 

		float x = CLAMP(pos.x, -1.f, 1.f);
		float y = -CLAMP(pos.y, -1.f, 1.f);
		int col = (int)(roundf(x + 1.0f));
		int row = (int)(roundf(y + 1.0f));

		// issue: row and col SNAP across 0.5 boundaries
		Array args;
		args.append(row);
		args.append(col);
		print_line(String("row {0} col {1}").format(args));

		// x, col blend
		{
			if (x == 0.f) {
				Ref<OzzAnimationState> a = poses[4];
				x_pose->copy_pose(a);
			}
			else if (x < 0.f) { // col 0
				Ref<OzzAnimationState> center = poses[row * 3 + 1];
				Ref<OzzAnimationState> other = poses[row * 3];
				float w = fabsf(x);
				ozz->blend2(x_pose, center, other, w);
			}
			else if (x > 0.f) {
				Ref<OzzAnimationState> center = poses[row * 3 + 1];
				Ref<OzzAnimationState> other = poses[row * 3 + 2];
				float w = fabsf(x);
				ozz->blend2(x_pose, center, other, w);
			}
		}

		// y, row blend
		{
			if (y == 0.f) {
				Ref<OzzAnimationState> a = poses[4];
				y_pose->copy_pose(a);
			}
			else if (y < 0.f) { // blend row 0 + row 1
				Ref<OzzAnimationState> center = poses[1 * 3 + col];
				Ref<OzzAnimationState> other = poses[col];
				float w = fabsf(y);
				ozz->blend2(y_pose, center, other, w);
			}
			else if (y > 0.f) {
				Ref<OzzAnimationState> center = poses[1 * 3 + col];
				Ref<OzzAnimationState> other = poses[2 * 3 + col];
				float w = fabsf(y);
				ozz->blend2(y_pose, center, other, w);
			}
		}

		float w = 0.5;
		ozz->blend2(output_pose, x_pose, y_pose, w);
		return output_pose;
		
	}

	static void _bind_methods() {
   		ClassDB::bind_method(D_METHOD("load", "ozz", "animation"), &OzzAimOffset::load);
		ClassDB::bind_method(D_METHOD("sample", "ozz", "pos"), &OzzAimOffset::sample);
	}

};

#endif
