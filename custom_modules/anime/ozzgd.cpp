#include "ozzgd.h"

#include "ozz/animation/offline/tools/gltf2ozz.h"


bool OzzGD::init(
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
    // todo: models should be initialised to global rests
    std::fill(models.begin(), models.end(), ozz::math::Float4x4::identity());

    return true;
}

void OzzGD::update_hitboxes(Transform3D global_transform, Dictionary hitboxes) {
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
ozz::math::Quaternion OzzGD::FrameRotation(float _duration) const {
    const float angle = angular_velocity * _duration;
    return ozz::math::Quaternion::FromEuler({ angle, 0, 0 });
}

Ref<OzzAnimationState> OzzGD::new_state() {
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

Ref<OzzAnimationState> OzzGD::new_state_bin(PackedByteArray data) {
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

void OzzGD::apply_animation_state(Ref<OzzAnimationState> state) {
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

bool OzzGD::play_animation(Ref<OzzAnimationState> state, float delta, bool update_cache, bool sample_motion, bool sample_events) {
    if (state.is_null()) {
        return false;
    }
    if (state->animation == nullptr) {
        return false;
    }

    int loops = 0;
    if (delta > 0.f) { // dont call controller.update with delta 0. it makes the prev ratio == current ratio. which is bad for track triggering
        state->controller.Update(*state->animation, delta);
    }

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
        job.to = state->controller.time_ratio();
        job.threshold = 0.5f;
        // if from > to, playing in reverse, rising edges become falling
        bool rising = job.to > job.from;
        float d = job.to - job.from;
        if (state->controller.playback_speed() > 0.f && d < 0.f) {
            // loop as occured
            job.from = job.to;
        } else if (state->controller.playback_speed() < 0.f && d > 0.f) {
            job.to = job.from;
        }

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
                if (edge.rising == rising) {
                    String track_name = String(track->name());
                    emit_signal("event_triggered", String(state->animation->name()), track_name);
                }
            }
        }
    }

    return true;
}

void OzzGD::blend2(Ref<OzzAnimationState> output, Ref<OzzAnimationState> base, Ref<OzzAnimationState> blend, float blend_amount) {
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

void OzzGD::add2(Ref<OzzAnimationState> output, Ref<OzzAnimationState> base, Ref<OzzAnimationState> add, float blend_amount, int global_bone) {
    if (output.is_null() || base.is_null() || add.is_null()) {
        return;
    }

    if (global_bone > -1) {
        // Converts from local space to model space
        ozz::vector<ozz::math::Float4x4> base_models;
        base_models.resize(skeleton->num_joints());
        ozz::animation::LocalToModelJob ltm_job;
        ltm_job.skeleton = skeleton.get();
        ltm_job.input = make_span(base->locals);
        ltm_job.output = make_span(base_models);
        if (!ltm_job.Run()) return;

        ozz::span<const int16_t> parents = skeleton->joint_parents();
        for (int i = 0; i < skeleton->num_joints(); i++) {
            int block = i / 4;
            int lane = i % 4;

            int parent = parents[i];

            if (i == global_bone) {
                if (parent == ozz::animation::Skeleton::kNoParent) {
                    continue;
                } else {
                    ozz::math::Float4x4 parent_model_matrix = base_models[parent]; //ozz::math::Invert(base_models[parent]);
                    ozz::math::SimdFloat4 r = ozz::math::ToQuaternion(parent_model_matrix);
                    float* rp = reinterpret_cast<float*>(&r);
                    ozz::math::Quaternion parent_model_rotation = ozz::math::Quaternion(rp[0], rp[1], rp[2], rp[3]);
                    ozz::math::Quaternion inv_parent_rotation = ozz::math::Conjugate(parent_model_rotation);
                    
                    // 3. Define your desired absolute model space rotation (e.g., Identity for no rotation)
                    // ozz::math::SoaTransform goal_transform = add->locals[block];
                    // float* gx = reinterpret_cast<float*>(&goal_transform.rotation.x);
                    // float* gy = reinterpret_cast<float*>(&goal_transform.rotation.y);
                    // float* gz = reinterpret_cast<float*>(&goal_transform.rotation.z);
                    // float* gw = reinterpret_cast<float*>(&goal_transform.rotation.w);
                    Transform3D global_rest = rests[i];
                    Quaternion q = global_rest.basis.get_rotation_quaternion();
                    ozz::math::Quaternion desired_model_rotation = ozz::math::Quaternion(q.x, q.y, q.z, q.w);

                    // 4. Calculate the corrected local rotation
                    // Note: ozz-animation quaternion multiplication order is Left * Right
                    ozz::math::Quaternion corrected_local_rotation = inv_parent_rotation * desired_model_rotation;
    
                    float* px = reinterpret_cast<float*>(&output->locals[block].rotation.x);
                    float* py = reinterpret_cast<float*>(&output->locals[block].rotation.y);
                    float* pz = reinterpret_cast<float*>(&output->locals[block].rotation.z);
                    float* pw = reinterpret_cast<float*>(&output->locals[block].rotation.w);

                    // Overwrite the specific lane
                    px[lane] = corrected_local_rotation.x;
                    py[lane] = corrected_local_rotation.y;
                    pz[lane] = corrected_local_rotation.z;
                    pw[lane] = corrected_local_rotation.w;	
                }
            }
        }

        ozz::animation::BlendingJob::Layer blend_layers[2];
        blend_layers[0].transform = make_span(base->locals);
        blend_layers[0].weight = 1.f;
        blend_layers[0].joint_weights = make_span(base->joint_weights);
        blend_layers[1].transform = make_span(add->locals);
        blend_layers[1].weight = blend_amount;
        blend_layers[1].joint_weights = make_span(add->joint_weights);
        ozz::animation::BlendingJob add_job;
        add_job.threshold = threshold;
        add_job.layers = blend_layers;
        add_job.rest_pose = skeleton->joint_rest_poses();
        add_job.output = make_span(output->locals);
        add_job.Run();
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

void OzzGD::_convert_f4x4_to_soa(ozz::vector<ozz::math::Float4x4> &in, ozz::vector<ozz::math::SoaTransform> &out, bool zero_translation) {
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

void OzzGD::_add_children_to_joint(int bone_idx, ozz::animation::offline::RawSkeleton::Joint *joint) {
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

bool OzzGD::load_skeleton() {
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

int OzzGD::find_bone(String bone_name) {
    for (int i = 0; i < names.size(); i++) {
        if (names[i] == bone_name) {
            return i;
        }
    }
    return -1;
}

// TODO: Add error logs and error checks
PackedByteArray OzzGD::convert_animation_to_ozz(Ref<Animation> animation, bool extract_motion, int root_bone, bool use_scale, bool delta, bool optimize) {
    PackedByteArray data;
    ozz::io::MemoryStream buf;
    ozz::io::OArchive output(&buf);

    ozz::animation::offline::RawAnimation raw_animation = _load_animation(animation, use_scale, optimize);

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
                            int track_index = -1;
                            for (int ti = 0; ti < raw_event_tracks.size(); ti++) {
                                if (strcmp(raw_event_tracks[ti].name.c_str(), event_name_string.utf8().get_data()) == 0) {
                                    track_index = ti;
                                    break;
                                }
                            }
                            if (track_index == -1) {
                                raw_event_tracks.resize(raw_event_tracks.size() + 1);
                                track_index = raw_event_tracks.size() - 1;
                            }
                            ozz::animation::offline::RawFloatTrack& raw_event_track = raw_event_tracks[track_index];
                            ozz::animation::offline::RawTrackKeyframe<float> frame;
                            frame.value = 1.f;
                            double ratio = CLAMP(animation->track_get_key_time(i, j) / animation->get_length(), 0.0002, 1.0);
                            frame.ratio = (float)ratio;
                            frame.interpolation = ozz::animation::offline::RawTrackInterpolation::kStep;
                            ozz::animation::offline::RawTrackKeyframe<float> low;
                            low.value = 0.f;
                            low.ratio = frame.ratio - 0.0001f;
                            low.interpolation = ozz::animation::offline::RawTrackInterpolation::kStep;
                            raw_event_track.keyframes.push_back(low);
                            raw_event_track.keyframes.push_back(frame);

                            if (raw_event_track.name.empty()) {
                                raw_event_track.name = ozz::string(event_name_string.utf8().get_data());
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

    data.resize(buf.Size());
    buf.Seek(0, ozz::io::MemoryStream::kSet);
    buf.Read(data.ptrw(), buf.Size());
    return data;
}

ozz::animation::offline::RawAnimation OzzGD::_load_animation(Ref<Animation> animation, bool use_scale, bool optimize) {
    // Use the Ozz animation builder to create an ozz animation.
    ozz::animation::offline::RawAnimation raw_animation;
    // All the animation keyframes times must be within range [0, duration].
    raw_animation.duration = animation->get_length();
    raw_animation.name = ozz::string(animation->get_name().utf8().get_data());

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

    if (optimize) {

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
    return raw_animation;
}

void OzzGD::update_skeleton_interpolated(float delta, RID skeleton_rid, RID visibility_notifier_rid) {
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

void OzzGD::update_skeleton(RID skeleton_rid, RID visibility_notifier_rid) {
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

void OzzGD::update_skeleton_ragdoll(Transform3D global_transform, Dictionary bodies, RID skeleton_rid, RID visibility_notifier_rid) {
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

Array OzzGD::get_parentless_bones() {
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
TypedArray<Transform3D> OzzGD::build_skin() {
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

Transform3D OzzGD::get_bone_model_pose(int bone_id) {
    if (bone_id < 0 || bone_id > models.size()) {
        return Transform3D();
    }
    return ozz_to_godot_xform(models[bone_id]);
}

void OzzGD::_bind_methods() {
    ClassDB::bind_method(D_METHOD("init", "names", "children", "rests", "global_rests", "bind_poses", "binds"), &OzzGD::init);
    ClassDB::bind_method(D_METHOD("play_animation", "state", "delta", "update_state", "sample_motion", "sample_events"), &OzzGD::play_animation);
    ClassDB::bind_method(D_METHOD("get_names"), &OzzGD::get_names);
    ClassDB::bind_method(D_METHOD("new_state"), &OzzGD::new_state);
    ClassDB::bind_method(D_METHOD("new_state_bin", "data"), &OzzGD::new_state_bin);
    ClassDB::bind_method(D_METHOD("blend2", "output", "a", "b", "blend_amount"), &OzzGD::blend2);
    ClassDB::bind_method(D_METHOD("add2", "output", "a", "b", "blend_amount", "global"), &OzzGD::add2, DEFVAL(-1));
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
    ClassDB::add_signal("OzzGD", MethodInfo("event_triggered", PropertyInfo(Variant::STRING, "anim_name"), PropertyInfo(Variant::STRING, "event_name")));
}