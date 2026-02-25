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

#include "ozz/samples/utils.h"

#include <cassert>
#include <limits>

#include "ozz/animation/offline/raw_animation.h"
#include "ozz/animation/offline/raw_skeleton.h"
#include "ozz/animation/runtime/animation.h"
#include "ozz/animation/runtime/local_to_model_job.h"
#include "ozz/animation/runtime/skeleton.h"
#include "ozz/animation/runtime/track.h"
#include "ozz/base/io/archive.h"
#include "ozz/base/io/stream.h"
#include "ozz/base/log.h"
#include "ozz/base/maths/box.h"
#include "ozz/base/maths/simd_math.h"
#include "ozz/base/maths/simd_quaternion.h"
#include "ozz/base/maths/soa_transform.h"
#include "ozz/base/memory/allocator.h"
#include "ozz/geometry/runtime/skinning_job.h"

using namespace ozz;	// {}
using namespace sample; // {}

PlaybackController::PlaybackController()
	: time_ratio_(0.f),
	  previous_time_ratio_(0.f),
	  playback_speed_(1.f),
	  play_(true),
	  loop_(true) {}

int PlaybackController::Update(const animation::Animation &_animation,
							   float _dt)
{
	float new_ratio = time_ratio_;

	if (play_)
	{
		new_ratio = time_ratio_ + _dt * playback_speed_ / _animation.duration();
	}

	// Must be called even if time doesn't change, in order to update previous
	// frame time ratio. Uses set_time_ratio function in order to update
	// previous_time_ and wrap time value in the unit interval (depending on loop
	// mode).
	return set_time_ratio(new_ratio);
}

int PlaybackController::set_time_ratio(float _ratio)
{
	//  Number of loops completed within _ratio, possibly negative if going
	//  backward.
	previous_time_ratio_ = time_ratio_;
	if (loop_)
	{
		// Wraps in the unit interval [0:1]
		const float loops = floorf(_ratio);
		time_ratio_ = _ratio - loops;
		return static_cast<int>(loops);
	}
	else
	{
		// Clamps in the unit interval [0:1].
		time_ratio_ = math::Clamp(0.f, _ratio, 1.f);
		return 0;
	}
}

// Gets animation current time.
float PlaybackController::time_ratio() const { return time_ratio_; }

// Gets animation time of last update.
float PlaybackController::previous_time_ratio() const
{
	return previous_time_ratio_;
}

void PlaybackController::Reset()
{
	previous_time_ratio_ = time_ratio_ = 0.f;
	playback_speed_ = 1.f;
	play_ = true;
}

// Loop through matrices and collect min and max bounds.
void ComputePostureBounds(ozz::span<const ozz::math::Float4x4> _models,
						  const ozz::math::Float4x4 &_transform,
						  math::Box *_bound)
{
	assert(_bound);

	// Set a default box.
	*_bound = ozz::math::Box();

	if (_models.empty())
	{
		return;
	}

	// Loops through matrices and stores min/max.
	math::SimdFloat4 min =
		math::simd_float4::Load1(std::numeric_limits<float>::max());
	math::SimdFloat4 max = -min;

	for (const auto &current : _models)
	{
		const auto transform = _transform * current;
		min = math::Min(min, transform.cols[3]);
		max = math::Max(max, transform.cols[3]);
	}

	// Stores in math::Box structure.
	math::Store3PtrU(min, &_bound->min.x);
	math::Store3PtrU(max, &_bound->max.x);

	return;
}

void MultiplySoATransformQuaternion(
	int _index, const ozz::math::SimdQuaternion &_quat,
	const ozz::span<ozz::math::SoaTransform> &_transforms)
{
	assert(_index >= 0 && static_cast<size_t>(_index) < _transforms.size() * 4 &&
		   "joint index out of bound.");

	// Convert soa to aos in order to perform quaternion multiplication, and gets
	// back to soa.
	ozz::math::SoaTransform &soa_transform_ref = _transforms[_index / 4];
	ozz::math::SimdQuaternion aos_quats[4];
	ozz::math::Transpose4x4(&soa_transform_ref.rotation.x, &aos_quats->xyzw);

	ozz::math::SimdQuaternion &aos_quat_ref = aos_quats[_index & 3];
	aos_quat_ref = aos_quat_ref * _quat;

	ozz::math::Transpose4x4(&aos_quats->xyzw, &soa_transform_ref.rotation.x);
}

bool LoadSkeleton(const char *_filename, ozz::animation::Skeleton *_skeleton)
{
	assert(_filename && _skeleton);
	ozz::log::Out() << "Loading skeleton archive " << _filename << "."
					<< std::endl;
	ozz::io::File file(_filename, "rb");
	if (!file.opened())
	{
		ozz::log::Err() << "Failed to open skeleton file " << _filename << "."
						<< std::endl;
		return false;
	}
	ozz::io::IArchive archive(&file);
	if (!archive.TestTag<ozz::animation::Skeleton>())
	{
		ozz::log::Err() << "Failed to load skeleton instance from file "
						<< _filename << "." << std::endl;
		return false;
	}

	// Once the tag is validated, reading cannot fail.
	{
		ProfileFctLog profile{"Skeleton loading time"};
		archive >> *_skeleton;
	}
	return true;
}

bool LoadAnimation(const char *_filename,
				   ozz::animation::Animation *_animation)
{
	assert(_filename && _animation);
	ozz::log::Out() << "Loading animation archive: " << _filename << "."
					<< std::endl;
	ozz::io::File file(_filename, "rb");
	if (!file.opened())
	{
		ozz::log::Err() << "Failed to open animation file " << _filename << "."
						<< std::endl;
		return false;
	}
	ozz::io::IArchive archive(&file);
	if (!archive.TestTag<ozz::animation::Animation>())
	{
		ozz::log::Err() << "Failed to load animation instance from file "
						<< _filename << "." << std::endl;
		return false;
	}

	// Once the tag is validated, reading cannot fail.
	{
		ProfileFctLog profile{"Animation loading time"};
		archive >> *_animation;
	}

	return true;
}

bool LoadRawAnimation(const char *_filename,
					  ozz::animation::offline::RawAnimation *_animation)
{
	assert(_filename && _animation);
	ozz::log::Out() << "Loading raw animation archive: " << _filename << "."
					<< std::endl;
	ozz::io::File file(_filename, "rb");
	if (!file.opened())
	{
		ozz::log::Err() << "Failed to open raw animation file " << _filename << "."
						<< std::endl;
		return false;
	}
	ozz::io::IArchive archive(&file);
	if (!archive.TestTag<ozz::animation::offline::RawAnimation>())
	{
		ozz::log::Err() << "Failed to load raw animation instance from file "
						<< _filename << "." << std::endl;
		return false;
	}

	// Once the tag is validated, reading cannot fail.
	{
		ProfileFctLog profile{"RawAnimation loading time"};
		archive >> *_animation;
	}

	return true;
}

template <typename _Track>
bool LoadTrackImpl(const char *_filename, _Track *_track)
{
	assert(_filename && _track);
	ozz::log::Out() << "Loading track archive: " << _filename << "." << std::endl;
	ozz::io::File file(_filename, "rb");
	if (!file.opened())
	{
		ozz::log::Err() << "Failed to open track file " << _filename << "."
						<< std::endl;
		return false;
	}
	ozz::io::IArchive archive(&file);
	if (!archive.TestTag<_Track>())
	{
		ozz::log::Err() << "Failed to load float track instance from file "
						<< _filename << "." << std::endl;
		return false;
	}

	// Once the tag is validated, reading cannot fail.
	{
		ProfileFctLog profile{"Track loading time"};
		archive >> *_track;
	}

	return true;
}

bool LoadTrack(const char *_filename, ozz::animation::FloatTrack *_track)
{
	return LoadTrackImpl(_filename, _track);
}
bool LoadTrack(const char *_filename, ozz::animation::Float2Track *_track)
{
	return LoadTrackImpl(_filename, _track);
}
bool LoadTrack(const char *_filename, ozz::animation::Float3Track *_track)
{
	return LoadTrackImpl(_filename, _track);
}
bool LoadTrack(const char *_filename, ozz::animation::Float4Track *_track)
{
	return LoadTrackImpl(_filename, _track);
}
bool LoadTrack(const char *_filename, ozz::animation::QuaternionTrack *_track)
{
	return LoadTrackImpl(_filename, _track);
}

ProfileFctLog::ProfileFctLog(const char *_name) : name_{_name}, begin_{clock::now()} {}

ProfileFctLog::~ProfileFctLog()
{
	std::chrono::duration<float, std::milli> duration = clock::now() - begin_;
	ozz::log::Out() << name_ << ": " << duration.count() << "ms" << std::endl;
}

	// namespace sample
	// namespace ozz
