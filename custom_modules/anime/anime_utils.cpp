#include "anime_utils.h"

#include "ozz/base/maths/simd_math.h"


// Helper to convert from Ozz Mat4x4 to godot transform3d
Transform3D ozz_to_godot_xform(ozz::math::Float4x4 m) {
	return Transform3D(
			ozz::math::GetX(m.cols[0]), ozz::math::GetX(m.cols[1]), ozz::math::GetX(m.cols[2]),
			ozz::math::GetY(m.cols[0]), ozz::math::GetY(m.cols[1]), ozz::math::GetY(m.cols[2]),
			ozz::math::GetZ(m.cols[0]), ozz::math::GetZ(m.cols[1]), ozz::math::GetZ(m.cols[2]),
			ozz::math::GetX(m.cols[3]), ozz::math::GetY(m.cols[3]), ozz::math::GetZ(m.cols[3])

	);
}

ozz::math::Float4x4 godot_to_ozz(Transform3D m) {
	ozz::math::Float4x4 r;
	float* c1 = reinterpret_cast<float*>(&r.cols[0]);
	c1[0] = m.basis.rows[0].x;
	c1[1] = m.basis.rows[1].x;
	c1[2] = m.basis.rows[2].x;
	float* c2 = reinterpret_cast<float*>(&r.cols[1]);
	c2[0] = m.basis.rows[0].y;
	c2[1] = m.basis.rows[1].y;
	c2[2] = m.basis.rows[2].y;
	float* c3 = reinterpret_cast<float*>(&r.cols[0]);
	c3[0] = m.basis.rows[0].z;
	c3[1] = m.basis.rows[1].z;
	c3[2] = m.basis.rows[2].z;
	float* c4 = reinterpret_cast<float*>(&r.cols[0]);
	c4[0] = m.origin.x;
	c4[1] = m.origin.y;
	c4[2] = m.origin.z;
	c4[3] = 1.f;
	return r;
}