#ifndef ANIME_CLASS_H
#define ANIME_CLASS_H

// TODO
// 8. Make this shit a module instead of gdextension

// DONE
// 1. Combine transform tracks into a single transform track cache. Must add flags to check if rot,pos,scale exists. Would reduce num of loops at complexity of finding/compiling cache
// 2. Threads (some safety)
// 3. Function to preprocess animationstate and setup all the nodes and stuff
// 4. Function to advance blend space in sync without doing any animation stuff

// Pitfalls
// 1. Dont forget C++ will create new objects when you access vectors if not careful.
// 2. Set all pointers to nullptr. Can cause many headaches.

// We don't need windows.h in this plugin but many others do and it throws up on itself all the time
// So best to include it and make sure CI warns us when we use something Microsoft took for their own goals....
// #ifdef WIN32
// #include <windows.h>
// #endif

#define ADD_GETTER(class, name) ClassDB::bind_method(D_METHOD(#name), &##class ::##name);
#define ADD_SETTER(class, name, arg, defval) ClassDB::bind_method(D_METHOD(#name, #arg), &##class ::##name, DEFVAL(##defval));

#include "core/object/class_db.h"
#include "core/string/print_string.h"
#include "core/object/ref_counted.h"
#include "scene/3d/skeleton_3d.h"
//#include "core/string.h"
#include "core/os/os.h"
#include "scene/animation/animation_player.h"
#include "scene/resources/animation.h"
#include "core/math/geometry_2d.h"
#include "core/templates/a_hash_map.h"

#include <vector>
#include <cmath>

class TrackCache : public RefCounted
{
    GDCLASS(TrackCache, RefCounted);

public:
    // Animation::TrackType type = Animation::TrackType::TYPE_POSITION_3D;
    // int object_id = -1;
    // Node* node = nullptr; // Replace with obj id?
    // NodePath property;
    int thash = 0;

    // Transform3D

    bool has_pos = false;
    bool has_rot = false;
    bool has_scale = false;
    bool has_method = false;

    int pos_track = -1;
    int rot_track = -1;
    int scale_track = -1;
    int method_track = -1;

    Vector3 pos;
    Vector3 scale;
    Quaternion rot;
    int bone_idx = -1;
    
    Quaternion rest_rot;

    // Value
    Variant value;

    // Bezier
    float bezier;

    TrackCache()
    {
        scale = Vector3(1, 1, 1);
    }

    static inline int make_thash(String path)
    {
        return path.hash();
    }

    void init(Animation *animation, int track_idx)
    {
        if (track_idx > -1) {
            ERR_FAIL_NULL(animation);
            // type = animation->track_get_type(track_idx);
            // property = animation->track_get_path(track_idx);
            thash = make_thash(animation->track_get_path(track_idx).get_concatenated_subnames());
        }
    }

    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "animation", "track_idx"), &TrackCache::init, DEFVAL(0), DEFVAL(0));
    }
};

// This is more like SkeletonAnimationState. Lets ignore other animation thingos
class AnimationState : public RefCounted
{
    GDCLASS(AnimationState, RefCounted);

public:
    Skeleton3D *skeleton = nullptr;
    Ref<Animation> animation;
    float time = 0.0;
    bool never_loop = false;
    AHashMap<int, TrackCache *, HashHasher> track_cache;

    void set_animation(Ref<Animation> p_animation) { animation = p_animation; }
    Ref<Animation> get_animation() { return animation; }
    void set_time(float p_time) { time = p_time; }
    float get_time() { return time; }
    void set_never_loop(bool p_never_loop) { never_loop = p_never_loop; }
    bool get_never_loop() { return never_loop; }

    AnimationState()
    {
        track_cache = AHashMap<int, TrackCache *, HashHasher>();
    }

    void init(Ref<Animation> p_animation)
    {
        ERR_FAIL_NULL(p_animation);
        ERR_FAIL_NULL(skeleton);
        animation = p_animation;

        int64_t track_count = animation->get_track_count();
        for (int i = 0; i < track_count; i++)
        {
            if (!animation->track_is_enabled(i)) {
                continue;
            }
            Animation::TrackType type = animation->track_get_type(i);
            NodePath path = animation->track_get_path(i);

            int thash = TrackCache::make_thash(path.get_concatenated_subnames());

            TrackCache *track = nullptr;
            if (track_cache.has(thash))
            {
                track = track_cache.get(thash);
            }
            else
            {
                track = memnew(TrackCache);
                track->init(*animation, i);
                track_cache[thash] = track;
            }
            ERR_FAIL_NULL(track);

            if (track->bone_idx == -1 && (type == Animation::TYPE_POSITION_3D || type == Animation::TYPE_ROTATION_3D))
            {
                String bone = path.get_concatenated_subnames();
                int bone_idx = skeleton->find_bone(bone);
                if (bone_idx == -1)
                {
                    track_cache.erase(thash);
                    memfree(track);
                    continue;
                } else {
                    track->bone_idx = bone_idx;
                    track->rest_rot = skeleton->get_bone_rest(track->bone_idx).basis.get_rotation_quaternion();
                }
            }

            switch (type)
            {
            case Animation::TYPE_POSITION_3D:
            {
                track->has_pos = true;
                track->pos_track = i;
                track->pos = skeleton->get_bone_rest(i).origin;
                break;
            }
            case Animation::TYPE_ROTATION_3D:
            {
                track->has_rot = true;
                track->rot_track = i;
                //track->rot = skeleton->get_bone_rest(i).basis.get_rotation_quaternion();
                break;
            }
            case Animation::TYPE_METHOD:
            {
                track->has_method = true;
                track->method_track = i;
                break;
            }
            default:
            {
                WARN_PRINT_ONCE("AnimationState loaded unsupported track type.");
                break;
            }
            }
        }

        // TODO: Make a track for every single bone in the skeleton
        // Add tracks for any bones no inside the animation. They wont be updated unless blended with an animation that has them
        int bones = skeleton->get_bone_count();
        for (int i = 0; i < bones; i++) {
            String bone_name = skeleton->get_bone_name(i);
            int thash = TrackCache::make_thash(bone_name);

            TrackCache *track = nullptr;
            if (!track_cache.has(thash))
            {
                track = memnew(TrackCache);
                track->init(*animation, -1);
                track_cache[thash] = track;
                track->has_rot = true;
                track->has_pos = true;
                track->has_scale = true;
                track->bone_idx = i;
                track->pos = skeleton->get_bone_rest(i).origin;
            }
        }


    }

    int find_thash_from_bone(int bone_idx)
    {
        for (const auto t : track_cache)
        {
            if (t.value->bone_idx == bone_idx)
            {
                return t.value->thash;
            }
        }
        return -1;
    }

    Quaternion get_track_rotation(int thash)
    {
        if (track_cache.has(thash))
        {
            return track_cache.get(thash)->rot;
        }
        return Quaternion();
    }

    void set_track_rotation(int thash, Quaternion rot)
    {
        TrackCache *track = nullptr;
        if (track_cache.has(thash))
        {
            track = track_cache.get(thash);
        }
        if (track != nullptr)
        {
            track->rot = rot;
        }
    }

    void track_get_key_indices_in_range(int p_track, double p_time, double p_delta, List<int> *p_indices, Animation::LoopedFlag p_looped_flag) {
        if (p_delta == 0) {
            return; // Prevent to get key continuously.
        }

        double from_time = p_time - p_delta;
        double to_time = p_time;

        bool is_backward = false;
        if (from_time > to_time) {
            is_backward = true;
            SWAP(from_time, to_time);
        }
        
        // Todo: set loop modes
        Animation::LoopMode loop_mode = Animation::LOOP_NONE;
        double length = animation->get_length();
        Animation::TrackType type = animation->track_get_type(p_track);

        switch (loop_mode) {
            case Animation::LOOP_NONE: {
                if (from_time < 0) {
                    from_time = 0;
                }
                if (from_time > length) {
                    from_time = length;
                }

                if (to_time < 0) {
                    to_time = 0;
                }
                if (to_time > length) {
                    to_time = length;
                }
            } break;
            case Animation::LOOP_LINEAR: {
                if (from_time > length || from_time < 0) {
                    from_time = Math::fposmod(from_time, length);
                }
                if (to_time > length || to_time < 0) {
                    to_time = Math::fposmod(to_time, length);
                }

                if (from_time > to_time) {
                    // Handle loop by splitting.
                    double anim_end = length + CMP_EPSILON;
                    double anim_start = -CMP_EPSILON;

                    switch (type) {
                        case Animation::TYPE_METHOD: {
                            if (!is_backward) {
                                _track_get_key_indices_in_range(p_track, from_time, anim_end, p_indices, is_backward);
                                _track_get_key_indices_in_range(p_track, anim_start, to_time, p_indices, is_backward);
                            } else {
                                _track_get_key_indices_in_range(p_track, anim_start, to_time, p_indices, is_backward);
                                _track_get_key_indices_in_range(p_track, from_time, anim_end, p_indices, is_backward);
                            }
                        } break;
                    }
                    return;
                }

                // Not from_time > to_time but most recent of looping...
                if (p_looped_flag != Animation::LOOPED_FLAG_NONE) {
                    if (!is_backward && Math::is_equal_approx(from_time, 0.0)) {
                        int edge = animation->track_find_key(p_track, 0, Animation::FIND_MODE_EXACT);
                        if (edge >= 0) {
                            p_indices->push_back(edge);
                        }
                    } else if (is_backward && Math::is_equal_approx(to_time, length)) {
                        int edge = animation->track_find_key(p_track, length, Animation::FIND_MODE_EXACT);
                        if (edge >= 0) {
                            p_indices->push_back(edge);
                        }
                    }
                }
            } break;
        }

        switch (type) {
            case Animation::TYPE_METHOD: {
                _track_get_key_indices_in_range(p_track, from_time, to_time, p_indices, is_backward);
            } break;
        }
    }

    void _track_get_key_indices_in_range(int p_track, double from_time, double to_time, List<int> *p_indices, bool p_is_backward) const {
        // Loop over every key and get keys in range
        // Skip keys that exist before from_time
        // Exit early when finding a key that exists after to_time
        int keys = animation->track_get_key_count(p_track);
        int len = keys;
        if (len == 0) {
            return;
        }

        int from = 0;
        int to = len - 1;

        if (!p_is_backward) {
            while (animation->track_get_key_time(p_track, from) < from_time || Math::is_equal_approx(animation->track_get_key_time(p_track, from), from_time)) {
                from++;
                if (to < from) {
                    return;
                }
            }
            while (animation->track_get_key_time(p_track, to) > to_time && !Math::is_equal_approx(animation->track_get_key_time(p_track, to), to_time)) {
                to--;
                if (to < from) {
                    return;
                }
            }
        } else {
            while (animation->track_get_key_time(p_track, from) < from_time && !Math::is_equal_approx(animation->track_get_key_time(p_track, from), from_time)) {
                from++;
                if (to < from) {
                    return;
                }
            }
            while (animation->track_get_key_time(p_track, to) > to_time || Math::is_equal_approx(animation->track_get_key_time(p_track, to), to_time)) {
                to--;
                if (to < from) {
                    return;
                }
            }
        }

        if (from == to) {
            p_indices->push_back(from);
            return;
        }

        if (!p_is_backward) {
            for (int i = from; i <= to; i++) {
                p_indices->push_back(i);
            }
        } else {
            for (int i = to; i >= from; i--) {
                p_indices->push_back(i);
            }
        }
    }

    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("init", "animation"), &AnimationState::init, DEFVAL(0));

        ClassDB::bind_method(D_METHOD("find_thash_from_bone", "bone_idx"), &AnimationState::find_thash_from_bone);
        ClassDB::bind_method(D_METHOD("get_track_rotation", "thash"), &AnimationState::get_track_rotation);
        ClassDB::bind_method(D_METHOD("set_track_rotation", "thash", "rot"), &AnimationState::set_track_rotation);

        // ClassDB::bind_method(D_METHOD("copy"), &AnimationState::copy);

        // setgets
        ADD_SETTER(AnimationState, set_animation, animation, 0)
        ADD_GETTER(AnimationState, get_animation)
        ADD_SETTER(AnimationState, set_time, time, 0.0f)
        ADD_GETTER(AnimationState, get_time)
        ADD_SETTER(AnimationState, set_never_loop, loop, true)
        ADD_GETTER(AnimationState, get_never_loop)

        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "animation"), "set_animation", "get_animation");
        ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "time"), "set_time", "get_time");
        ADD_PROPERTY(PropertyInfo(Variant::BOOL, "never_loop"), "set_never_loop", "get_never_loop");
        // register_property<AnimationState, Array>("track_cache", &AnimationState::track_cache, Array());
    }
};

class Anime : public RefCounted
{
    GDCLASS(Anime, RefCounted);

public:
    // bool debug_advance_time = false;
    // bool playing = false;
    AnimationPlayer *animation_player = nullptr;
    int root_bone_idx = -1;
    Transform3D root_transform;
    Dictionary bone_whitelist;
    Skeleton3D* skeleton = nullptr;
    Array bone_poses;
    bool events_only = false;

    int get_root_bone_index() { return root_bone_idx; }
    Skeleton3D* get_skeleton() { return skeleton; }
    AnimationPlayer *get_animation_player() { return animation_player; }
    Transform3D get_root_transform() { return root_transform; }
    Dictionary get_bone_whitelist() { return bone_whitelist; }
    bool get_events_only() { return events_only; }

    void set_root_bone_index(int idx) { root_bone_idx = idx; }
    void set_skeleton(Skeleton3D* p_skeleton) { skeleton = p_skeleton; }
    void set_animation_player(AnimationPlayer *p_animation_player) { animation_player = p_animation_player; }
    void set_root_transform(Transform3D p_root_transform) { root_transform = p_root_transform; }
    void set_bone_whitelist(Dictionary whitelist) { bone_whitelist = whitelist; }
    void set_events_only(bool val) { events_only = val; }

    Anime()
    {
        this->animation_player = nullptr;
        this->root_transform = Transform3D();
    }

    void init()
    {
        if (skeleton != nullptr) {
            bone_poses.resize(skeleton->get_bone_count());
            bone_poses.fill(Transform3D());
        }
    }

    Ref<AnimationState> new_state(String animation_name)
    {
        Ref<Animation> animation = animation_player->get_animation(animation_name);
        if (animation.is_null())
        {
            return nullptr;
        }
        Ref<AnimationState> as = Ref<AnimationState>(memnew(AnimationState));
        as->skeleton = skeleton;
        as->init(animation);
        return as;
    }

    void apply_animation_state(AnimationState *state, bool cache_only = false)
    {
        // Only called once right at the end
        // TODO: Make sure the methods are only being called once!
        ERR_FAIL_NULL(state);
        ERR_FAIL_NULL(skeleton);
        root_transform = Transform3D();

        for (KeyValue<int, TrackCache *> &K : state->track_cache)
        {
            TrackCache *track = K.value;

            if (cache_only == false) {
                if (track->has_pos)
                {
                    if (track->bone_idx == root_bone_idx) {
                        root_transform.origin = track->pos;
                    } else {
                        skeleton->set_bone_pose_position(track->bone_idx, track->pos);
                    }
                }
                
                if (track->has_rot)
                {
                    skeleton->set_bone_pose_rotation(track->bone_idx, track->rot);
                }
            }

            if (track->bone_idx > -1) {
                bone_poses[track->bone_idx] = Transform3D(Basis(track->rot), track->pos);
            }
        }
    }

    Array get_bone_poses() {
        return bone_poses;
    }

    Transform3D get_bone_pose(int bone_idx) {
        return bone_poses[bone_idx];
    }

    inline void play_animation(Ref<AnimationState> state, float delta, bool update_cache = true)
    {
        _play_animation(state, delta, update_cache, Dictionary());
    }

    inline void play_animation_filter(Ref<AnimationState> state, float delta, bool update_cache = true, Dictionary filter = Dictionary())
    {
        _play_animation(state, delta, update_cache, filter);
    }

    void _play_animation(Ref<AnimationState> state, float delta, bool update_cache = true, Dictionary filter = Dictionary())
    {
        ERR_FAIL_NULL_MSG(state, "Animation state is null!");
        Ref<Animation> animation = state->animation;
        ERR_FAIL_COND_MSG(animation.is_null(), "Animation is null");

        float time = state->time;
        state->time += delta;

        if (state->time > animation->get_length())
        {
            if (animation->get_loop_mode() != Animation::LOOP_NONE && state->never_loop == false)
            {
                state->time = 0.0;
            }
            else
            {
                state->time = animation->get_length();
            }
        }
        else if (state->time < 0.0)
        { // Loop backwards
            if (animation->get_loop_mode() != Animation::LOOP_NONE && state->never_loop == false)
            {
                state->time = animation->get_length();
            }
            else
            {
                state->time = 0.0;
            }
        }

        if (update_cache == false)
        {
            return;
        }

        bool use_whitelist = bone_whitelist.is_empty() == false;

        for (KeyValue<int, TrackCache *> &K : state->track_cache)
        {
            TrackCache *track = K.value;

            if (use_whitelist)
            {
                String bone_name = state->skeleton->get_bone_name(track->bone_idx);
                if (!bone_whitelist.has(bone_name))
                {
                    continue;
                }
            }

            if (track->has_rot && !events_only)
            {
                if (track->rot_track > -1) {
                    track->rot = animation->rotation_track_interpolate(track->rot_track, state->time);
                } else {
                    track->rot = Quaternion();
                }
            }

            if (track->has_pos && !events_only)
            {
                if (track->pos_track > -1) {
                    track->pos = animation->position_track_interpolate(track->pos_track, state->time) * skeleton->get_motion_scale();
                } else {
                    track->pos = Vector3();
                }
            }

            if (track->has_method)
            {
                List<int> indices;
                state->track_get_key_indices_in_range(track->method_track, time, delta, &indices, Animation::LOOPED_FLAG_END);
                for (int &F : indices) {
                    StringName method = animation->method_track_get_name(track->method_track, F);
                    Vector<Variant> params = animation->method_track_get_params(track->method_track, F);
                    if (animation_player->has_method(method)) {
                        animation_player->call_thread_safe(method, params[0], params[1]);
                    } else {
                        OS::get_singleton()->printerr("Cant call method as it is not found on animation player!");
                    }
                }
            }
        }
    }

    enum BlendMode
    {
        BLEND,
        ADD,
        SUB
    };

    void blend_animation(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount)
    {
        _blend_animation(state, blend_state, blend_amount, BlendMode::BLEND, Dictionary());
    }

    void blend_animation_filter(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount, Dictionary filter)
    {
        _blend_animation(state, blend_state, blend_amount, BlendMode::BLEND, filter);
    }

    void add_animation(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount)
    {
        _blend_animation(state, blend_state, blend_amount, BlendMode::ADD, Dictionary());
    }

    void add_animation_filter(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount, Dictionary filter)
    {
        _blend_animation(state, blend_state, blend_amount, BlendMode::ADD, filter);
    }

    void sub_animation(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount)
    {
        _blend_animation(state, blend_state, blend_amount, BlendMode::SUB, Dictionary());
    }

    void sub_animation_filter(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount, Dictionary filter)
    {
        _blend_animation(state, blend_state, blend_amount, BlendMode::SUB, filter);
    }

    void _blend_animation(Ref<AnimationState> state, Ref<AnimationState> blend_state, float blend_amount, enum BlendMode blend_mode, Dictionary filter)
    {
        // WARNING: Only blend tracks that exist in state, extra tracks in blend_state, wont be blended.
        // This is why we must add a root transform track to every animation state
        if (Math::is_equal_approx(blend_amount, 0.0f) && filter.is_empty())
        {
            return;
        }

        ERR_FAIL_NULL(skeleton);

        bool use_whitelist = bone_whitelist.is_empty() == false;
        bool use_filter = filter.is_empty() == false;

        for (KeyValue<int, TrackCache *> &K : state->track_cache)
        {
            TrackCache *state_track = K.value;
            TrackCache *blend_track = nullptr;
            if (blend_state->track_cache.has(K.key))
            {
                blend_track = blend_state->track_cache.get(K.key);
            }

            if (blend_track == nullptr)
            {
                continue;
            }

            if (use_whitelist)
            {
                String bone_name = skeleton->get_bone_name(state_track->bone_idx);
                if (!bone_whitelist.has(bone_name))
                {
                    continue;
                }
            }

            float blend = CLAMP(abs(blend_amount), 0.0f, 1.0f);
            if (state_track->bone_idx == -1)
            {
                continue;
            }

            if (use_filter)
            {
                String bone_name = skeleton->get_bone_name(state_track->bone_idx);
                if (filter.has(bone_name))
                {
                    Variant weight = filter[bone_name];
                    if (weight.get_type() == Variant::FLOAT)
                    {
                        blend = CLAMP(abs((float)weight * blend), 0.0f, 1.0f);
                        if (Math::is_zero_approx(blend))
                        {
                            continue;
                        }
                    }
                }
                else
                {
                    continue; // Performance: Ignore bones not in the filter
                }
            }

            if (events_only) {
                if (state_track->bone_idx != root_bone_idx) {
                    continue; // Only play root motion in events only
                }
            }

            Vector3 pos = state_track->pos;
            Vector3 scale = state_track->scale;
            Quaternion rot = state_track->rot;

            switch (blend_mode)
            {
            case BlendMode::SUB: {
                if (blend_track->has_pos)
                {
                    pos -= (blend_track->pos * blend);
                    state_track->pos = pos;
                }

                if (blend_track->has_rot)
                {
                    rot = (rot * Quaternion().slerp(blend_track->rot, -blend)).normalized();
                    state_track->rot = rot;
                }
                break;
            }
            case BlendMode::ADD:
            {
                if (blend_track->has_pos)
                {
                    pos += (blend_track->pos * blend);
                    state_track->pos = pos;
                }

                if (blend_track->has_rot)
                {
                    rot = (rot * Quaternion().slerp(blend_track->rot, blend)).normalized();
                    state_track->rot = rot;
                }

                break;
            }
            case BlendMode::BLEND:
            {
                if (blend_track->has_pos)
                {
                    pos = pos.lerp(blend_track->pos, blend);
                    state_track->pos = pos;
                }
                if (blend_track->has_rot)
                {
                    rot = rot.slerp(blend_track->rot.normalized(), blend).normalized();
                    state_track->rot = rot;
                }
                break;
            }
            }

            if (blend_track->has_scale)
            {
                scale = scale.lerp(blend_track->scale, abs(blend));
                state_track->scale = scale;
            }
        }
    }

    static void _bind_methods()
    {
        ADD_SETTER(Anime, set_animation_player, animation_player, 0);
        ADD_GETTER(Anime, get_animation_player);
        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "animation_player"), "set_animation_player", "get_animation_player");

        ADD_SETTER(Anime, set_root_bone_index, idx, -1);
        ADD_GETTER(Anime, get_root_bone_index);
        ADD_PROPERTY(PropertyInfo(Variant::STRING, "root_bone_index"), "set_root_bone_index", "get_root_bone_index");

        ADD_SETTER(Anime, set_root_transform, root, Transform3D());
        ADD_GETTER(Anime, get_root_transform);
        ADD_PROPERTY(PropertyInfo(Variant::TRANSFORM3D, "root_transform"), "set_root_transform", "get_root_transform");

        ADD_SETTER(Anime, set_bone_whitelist, root, Dictionary());
        ADD_GETTER(Anime, get_bone_whitelist);
        ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "bone_whitelist"), "set_bone_whitelist", "get_bone_whitelist");

        ADD_SETTER(Anime, set_skeleton, skeleton, 0);
        ADD_GETTER(Anime, get_skeleton);
        ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "skeleton"), "set_skeleton", "get_skeleton");

        ADD_SETTER(Anime, set_events_only, value, 0);
        ADD_GETTER(Anime, get_events_only);
        ADD_PROPERTY(PropertyInfo(Variant::DICTIONARY, "events_only"), "set_events_only", "get_events_only");

        ClassDB::bind_method(D_METHOD("init"), &Anime::init);
        ClassDB::bind_method(D_METHOD("new_state", "animation_name"), &Anime::new_state, DEFVAL(""));
        ClassDB::bind_method(D_METHOD("apply_animation_state", "state", "cache_only"), &Anime::apply_animation_state);
        ClassDB::bind_method(D_METHOD("play_animation", "animation_state", "delta", "update_cache"), &Anime::play_animation);
        ClassDB::bind_method(D_METHOD("play_animation_filter", "animation_state", "delta", "update_cache", "filter"), &Anime::play_animation_filter);
        ClassDB::bind_method(D_METHOD("blend_animation"), &Anime::blend_animation);
        ClassDB::bind_method(D_METHOD("blend_animation_filter"), &Anime::blend_animation_filter);
        ClassDB::bind_method(D_METHOD("add_animation"), &Anime::add_animation);
        ClassDB::bind_method(D_METHOD("add_animation_filter"), &Anime::add_animation_filter);
        ClassDB::bind_method(D_METHOD("sub_animation"), &Anime::sub_animation);
        ClassDB::bind_method(D_METHOD("sub_animation_filter"), &Anime::sub_animation_filter);
        ClassDB::bind_method(D_METHOD("get_bone_poses"), &Anime::get_bone_poses);
        ClassDB::bind_method(D_METHOD("get_bone_pose"), &Anime::get_bone_pose);
    }
};

struct BlendPoint
{
    Ref<AnimationState> animation_state;
    Vector2 position = Vector2();
};

typedef struct BlendPoint BlendPoint;

class BlendSpace2D : public RefCounted
{
    GDCLASS(BlendSpace2D, RefCounted);

public:
    BlendSpace2D()
    {
        triangles = Array();
        root = nullptr;
    }

    ~BlendSpace2D()
    {
    }

    void add_blend_point(String animation_name, Vector2 p_position)
    {
        ERR_FAIL_NULL(root);
        ERR_FAIL_NULL(root->animation_player);
        if (!root->animation_player->has_animation(animation_name))
        {
            CRASH_NOW_MSG(String("Cant find animation {0}").format(animation_name));
            return;
        }

        Ref<AnimationState> as = root->new_state(animation_name);
        ERR_FAIL_COND(as.is_null());
        blend_points.push_back({as, p_position});
    }

    /*
    BlendPoint* get_blend_point(int idx) {
        if (idx < blend_points.size()) {
            return &blend_points[idx];
        } else {
            return nullptr;
        }
    }
    */

    void update(float delta)
    {
        // Update all animations in sync
        for (const auto &bp : blend_points)
        {
            // Advance time but dont update cache for all
            root->play_animation(bp.animation_state, delta, false);
        }
    }

    Ref<AnimationState> get_animation_state(Vector2 p_position)
    {
        p_position.x = CLAMP(p_position.x, -1.0f, 1.0f);
        p_position.y = CLAMP(p_position.y, -1.0f, 1.0f);

        bool first = false;
        Vector2 best_point = Vector2(INFINITY, INFINITY);
        Array best_tri;
        Array res;
        float blend_weights[3] = {0.f, 0.f, 0.f};

        // Find blend triangle
        for (int i = 0; i < triangles.size(); i++)
        {
            Array tri = triangles[i];
            // HANDLE CASE WHERE BLEND_POS IS ON POINT
            for (int x = 0; x < tri.size(); x++)
            {
                int index = tri[x];
                if (p_position.distance_to(blend_points[index].position) <= CMP_EPSILON)
                {
                    Ref<AnimationState> as = blend_points[index].animation_state;
                    root->play_animation(as, 0.0, true);
                    return as;
                }
            }

            // HANDLE CASE WHERE BLEND_POS IS INSIDE TRIANGLE
            // Vector2 centroid = (blend_points[tri[0]].position + blend_points[tri[1]].position + blend_points[tri[2]].position) / 3.0;
            Vector2 dir0; // = centroid.direction_to(blend_points[tri[0]].position) * CMP_EPSILON;
            Vector2 dir1; // = centroid.direction_to(blend_points[tri[1]].position) * CMP_EPSILON;
            Vector2 dir2; // = centroid.direction_to(blend_points[tri[2]].position) * CMP_EPSILON;
            if (Geometry2D::is_point_in_triangle(p_position, blend_points[tri[0]].position + dir0, blend_points[tri[1]].position + dir1, blend_points[tri[2]].position + dir2))
            {

                Ref<AnimationState> states[3] = {
                    blend_points[tri[0]].animation_state, // You dont need to copy. This was done to remove the cached values but these are overriden when play_animation is called with update_cache = true
                    blend_points[tri[1]].animation_state,
                    blend_points[tri[2]].animation_state};

                Vector2 positions[3] = {
                    blend_points[tri[0]].position,
                    blend_points[tri[1]].position,
                    blend_points[tri[2]].position};

                if (p_position.distance_squared_to(positions[0]) <= CMP_EPSILON)
                {
                    root->play_animation(states[0], 0.0, true);
                    return states[0];
                }

                if (p_position.distance_squared_to(positions[1]) <= CMP_EPSILON)
                {
                    root->play_animation(states[1], 0.0, true);
                    return states[1];
                }

                if (p_position.distance_squared_to(positions[2]) <= CMP_EPSILON)
                {
                    root->play_animation(states[2], 0.0, true);
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
                if (denom == 0)
                {
                    root->play_animation(states[0], 0.0, true);
                    return states[0];
                }

                real_t v = (d11 * d20 - d01 * d21) / denom;
                real_t w = (d00 * d21 - d01 * d20) / denom;
                real_t u = 1.0 - v - w;

                root->play_animation(states[0], 0.0, true);
                root->play_animation(states[1], 0.0, true);
                root->play_animation(states[2], 0.0, true);

                root->blend_animation(states[0], states[1], v);
                root->blend_animation(states[0], states[2], w);

                return states[0];
            }

            // HANDLE CASE WHERE BLEND_POS IS OUTSIDE OF ALL TRIANGLES
            // Get closest segment
            for (int j = 0; j < 3; j++)
            {
                Vector2 segment_a = blend_points[tri[j]].position;
                Vector2 segment_b = blend_points[tri[(j + 1) % 3]].position;
                Vector2 closest = Geometry2D::get_closest_point_to_segment(p_position, segment_a, segment_b);
                if (first || closest.distance_to(p_position) < best_point.distance_to(p_position))
                {
                    best_point = closest;
                    first = false;
                    best_tri = tri;
                    float d = segment_a.distance_to(segment_b);
                    if (d == 0.0)
                    {
                        blend_weights[j] = 1.0;
                        blend_weights[(j + 1) % 3] = 0.0;
                        blend_weights[(j + 2) % 3] = 0.0;
                    }
                    else
                    {
                        float c = segment_a.distance_to(closest) / d;

                        blend_weights[j] = 1.0 - c;
                        blend_weights[(j + 1) % 3] = c;
                        blend_weights[(j + 2) % 3] = 0.0;
                    }
                }
            }
        }

        // If here, must be outside triangle case
        Ref<AnimationState> states[3] = {
            blend_points[best_tri[0]].animation_state, // You dont need to copy. This was done to remove the cached values but these are overriden when play_animation is called with update_cache = true
            blend_points[best_tri[1]].animation_state,
            blend_points[best_tri[2]].animation_state};

        if (Math::is_equal_approx(blend_weights[0], 1.0f))
        {
            root->play_animation(states[0], 0.0, true);
            return states[0];
        }

        if (Math::is_equal_approx(blend_weights[1], 1.0f))
        {
            root->play_animation(states[1], 0.0, true);
            return states[1];
        }

        if (Math::is_equal_approx(blend_weights[2], 1.0f))
        {
            root->play_animation(states[2], 0.0, true);
            return states[2];
        }

        Ref<AnimationState> state;

        if (blend_weights[0] > 0.0 && blend_weights[1] > 0.0)
        {
            state = states[0];
            root->play_animation(state, 0.0, true);
            root->play_animation(states[1], 0.0, true);
            root->blend_animation(state, states[1], blend_weights[1]);
        }

        if (blend_weights[0] > 0.0 && blend_weights[2] > 0.0)
        {
            state = states[0];
            root->play_animation(state, 0.0, true);
            root->play_animation(states[2], 0.0, true);
            root->blend_animation(state, states[2], blend_weights[2]);
        }

        if (blend_weights[1] > 0.0 && blend_weights[2] > 0.0)
        {
            state = states[1];
            root->play_animation(state, 0.0, true);
            root->play_animation(states[2], 0.0, true);
            root->blend_animation(state, states[2], blend_weights[2]);
        }

        if (state.is_null())
        {
            OS::get_singleton()->printerr("AnimationState is null. I thought all cases were covered? This should never happen!");
            return states[0];
        }

        return state;
    }

    Anime *root = nullptr;
    std::vector<BlendPoint> blend_points;
    Array triangles;

    Anime *get_root() { return root; }
    // std::vector<BlendPoint> get_blend_points() {return blend_points; }
    Array get_triangles() { return triangles; }

    void set_root(Anime *p_root) { root = p_root; }
    // void set_blend_points(std::vector<BlendPoint> points) { blend_points = points;}
    void set_triangles(Array p_triangles) { triangles = p_triangles; }

    static void _bind_methods()
    {
        ClassDB::bind_method(D_METHOD("add_blend_point", "animation_name", "point"), &BlendSpace2D::add_blend_point, DEFVAL(""), DEFVAL(Vector2()));
        // ClassDB::bind_method(D_METHOD("get_blend_point"), &BlendSpace2D::get_blend_point);
        // ClassDB::bind_method(D_METHOD("get_triangle"), &BlendSpace2D::get_triangle);
        ClassDB::bind_method(D_METHOD("get_animation_state", "position"), &BlendSpace2D::get_animation_state);
        ClassDB::bind_method(D_METHOD("update", "delta"), &BlendSpace2D::update, DEFVAL(0.f));

        ADD_SETTER(BlendSpace2D, set_root, root, 0)
        ADD_GETTER(BlendSpace2D, get_root)

        ADD_SETTER(BlendSpace2D, set_triangles, triangles, Array())
        ADD_GETTER(BlendSpace2D, get_triangles)

        ADD_PROPERTY(PropertyInfo(Variant::OBJECT, "root"), "set_root", "get_root");
        ADD_PROPERTY(PropertyInfo(Variant::ARRAY, "triangles"), "set_triangles", "get_triangles");
        // register_property<BlendSpace2D, Array>("blend_points", &BlendSpace2D::blend_points, Array());
    }
};

/*
class Anime : public RefCounted {
    GDCLASS(Anime, RefCounted);
public:
    Anime() { }

    // `_init` must exist as it is called by Godot.
    void _init() { }

    void test_void_method() {
        Godot::OS::print("This is test");
    }

    Variant method(Variant arg) {
        Variant ret;
        ret = arg;

        return ret;
    }

    static void _register_methods() {
        register_method("test_void_method", &Anime::test_void_method);
        register_method("method", &Anime::method);


        // * The line below is equivalent to the following GDScript export:
        // *     export var _name = "Anime"
        register_property<Anime, String>("base/name", &Anime::_name, String("Anime"));

        // Alternatively, with getter and setter methods:
        register_property<Anime, int>("base/value", &Anime::set_value, &Anime::get_value, 0);

        // Registering a signal:
        // register_signal<Anime>("signal_name");
        // register_signal<Anime>("signal_name", "string_argument", GODOT_VARIANT_TYPE_STRING)
    }

    String _name;
    int _value;

    void set_value(int p_value) {
        _value = p_value;
    }

    int get_value() const {
        return _value;
    }
};
*/

#endif