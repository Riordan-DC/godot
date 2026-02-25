//----------------------------------------------------------------------------//
//                                                                            //
// ozz-animation is hosted at http://github.com/guillaumeblanc/ozz-animation  //
// and distributed under the MIT License (MIT).                               //
//                                                                            //
// Copyright (c) Guillaume Blanc                                              //
//                                                                            //
// Permission is hereby granted, free of charge, to any person obtaining a    //
// copy of this software and associated documentation files (the "Software"), //
// to deal in the Software without restriction, including without limitation  //
// the rights to use, copy, modify, merge, publish, distribute, sublicense,   //
// and/or sell copies of the Software, and to permit persons to whom the      //
// Software is furnished to do so, subject to the following conditions:       //
//                                                                            //
// The above copyright notice and this permission notice shall be included in //
// all copies or substantial portions of the Software.                        //
//                                                                            //
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR //
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,   //
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL    //
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER //
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING    //
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER        //
// DEALINGS IN THE SOFTWARE.                                                  //
//                                                                            //
//----------------------------------------------------------------------------//

#include "ozz/samples/motion_utils.h"

#include <cassert>

#include "ozz/animation/runtime/track_sampling_job.h"
#include "ozz/base/containers/vector.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/transform.h"

namespace ozz {
namespace sample {

bool LoadMotionTrack(const char* _filename, MotionTrack* _track) {
  assert(_filename && _track);
  ozz::log::Out() << "Loading motion tracks archive: " << _filename << "."
                  << std::endl;
  ozz::io::File file(_filename, "rb");
  if (!file.opened()) {
    ozz::log::Err() << "Failed to open motion tracks file " << _filename << "."
                    << std::endl;
    return false;
  }
  ozz::io::IArchive archive(&file);

  // Once the tag is validated, reading cannot fail.
  {
    if (!archive.TestTag<ozz::animation::Float3Track>()) {
      ozz::log::Err()
          << "Failed to load position motion track instance from file "
          << _filename << "." << std::endl;
      return false;
    }
    archive >> _track->position;

    if (!archive.TestTag<ozz::animation::QuaternionTrack>()) {
      ozz::log::Err()
          << "Failed to load rotation motion track instance from file "
          << _filename << "." << std::endl;
      return false;
    }
    archive >> _track->rotation;
  }

  return true;
}

bool SampleMotion(const MotionTrack& _tracks, float _ratio,
                  ozz::math::Transform* _transform) {
  // Get position from motion track
  ozz::animation::Float3TrackSamplingJob position_sampler;
  position_sampler.track = &_tracks.position;
  position_sampler.result = &_transform->translation;
  position_sampler.ratio = _ratio;
  if (!position_sampler.Run()) {
    return false;
  }

  // Get rotation from motion track
  ozz::animation::QuaternionTrackSamplingJob rotation_sampler;
  rotation_sampler.track = &_tracks.rotation;
  rotation_sampler.result = &_transform->rotation;
  rotation_sampler.ratio = _ratio;
  if (!rotation_sampler.Run()) {
    return false;
  }

  _transform->scale = ozz::math::Float3::one();

  return true;
}

void MotionDeltaAccumulator::Update(const ozz::math::Transform& _delta) {
  Update(_delta, ozz::math::Quaternion::identity());
}

void MotionDeltaAccumulator::Update(const ozz::math::Transform& _delta,
                                    const ozz::math::Quaternion& _rotation) {
  // Remembers previous transform to be able to compute delta
  const auto previous = current;

  // Accumulates rotation.
  rotation_accum = Normalize(rotation_accum * _rotation);

  // Updates current transform.
  current.translation =
      current.translation + TransformVector(rotation_accum, _delta.translation);
  current.rotation = Normalize(current.rotation * _delta.rotation * _rotation);

  // Computes motion delta.
  delta.translation = current.translation - previous.translation;
  delta.rotation = Conjugate(previous.rotation) * current.rotation;
}

void MotionDeltaAccumulator::Teleport(const ozz::math::Transform& _origin) {
  current = _origin;

  // No delta between last and current
  delta = ozz::math::Transform::identity();

  // Resets rotation accumulator.
  rotation_accum = ozz::math::Quaternion::identity();
}

void MotionAccumulator::Update(const ozz::math::Transform& _new) {
  return Update(_new, ozz::math::Quaternion::identity());
}

// Accumulates motion deltas (new transform - last).
void MotionAccumulator::Update(const ozz::math::Transform& _new,
                               const ozz::math::Quaternion& _delta_rotation) {
  // Computes animated delta.
  const ozz::math::Transform animated_delta = {
      _new.translation - last.translation,
      Conjugate(last.rotation) * _new.rotation};

  // Updates current transform based on computed delta.
  MotionDeltaAccumulator::Update(animated_delta, _delta_rotation);

  // Next time, delta will be computed from the _new transform.
  last = _new;
}

void MotionAccumulator::ResetOrigin(const ozz::math::Transform& _origin) {
  last = _origin;
}

void MotionAccumulator::Teleport(const ozz::math::Transform& _origin) {
  MotionDeltaAccumulator::Teleport(_origin);

  // Resets current transform to new _origin
  last = current;
}

bool MotionSampler::Update(const MotionTrack& _motion, float _ratio,
                           int _loops) {
  return Update(_motion, _ratio, _loops, ozz::math::Quaternion::identity());
}

bool MotionSampler::Update(const MotionTrack& _motion, float _ratio, int _loops,
                           const ozz::math::Quaternion& _delta_rotation) {
  ozz::math::Transform sample;

  if (_loops != 0) {
    // When animation is looping, it's important to take into account the
    // motion done during the loop(s).

    // Uses a local accumulator to accumulate motion during loops.
    MotionAccumulator local_accumulator;
    local_accumulator.Teleport(last);

    for (; _loops; _loops > 0 ? --_loops : ++_loops) {
      // Samples motion at loop end (or begin depending on playback
      // direction).
      if (!SampleMotion(_motion, _loops > 0 ? 1.f : 0.f, &sample)) {
        return false;
      }
      local_accumulator.Update(sample);

      // Samples motion at the new origin.
      if (!SampleMotion(_motion, _loops > 0 ? 0.f : 1.f, &sample)) {
        return false;
      }
      local_accumulator.ResetOrigin(sample);
    }

    // Samples track at _ratio and compute motion since the loop end.
    if (!SampleMotion(_motion, _ratio, &sample)) {
      return false;
    }
    local_accumulator.Update(sample);

    // Update "this" accumulator with the one accumulated during the loop(s).
    // This way, _rot is applied to the whole motion, including what happened
    // during the loop(s).
    MotionAccumulator::Update(local_accumulator.current, _delta_rotation);

    // Next time, delta will be computed from the new origin, aka after the
    // loop.
    ResetOrigin(sample);
  } else {
    // Samples motion at the current ratio.
    if (!SampleMotion(_motion, _ratio, &sample)) {
      return false;
    }
    // Apply motion
    MotionAccumulator::Update(sample, _delta_rotation);
  }

  return true;
}

}  // namespace sample
}  // namespace ozz
