#ifndef FIXED_ARRAYV_H
#define FIXED_ARRAYV_H

#include <core/error_macros.h>

// TODO Could use std::array, but due to how Godot compiles,
// I couldn't find a way to enable boundary checks without failing to link my module with the rest of Godot...
template <typename T, unsigned int N>
class FixedArrayV {
public:
	inline FixedArrayV() {}

	inline FixedArrayV(T defval) {
		fill(defval);
	}

	inline void fill(T v) {
		for (unsigned int i = 0; i < N; ++i) {
			_data[i] = v;
		}
	}

	// TODO Optimization: move semantics

	inline T &operator[](unsigned int i) {
#ifdef DEBUG_ENABLED
		CRASH_COND(i >= N);
#endif
		return _data[i];
	}

	inline const T &operator[](unsigned int i) const {
#ifdef DEBUG_ENABLED
		CRASH_COND(i >= N);
#endif
		return _data[i];
	}

	inline bool equals(const FixedArrayV<T, N> &other) const {
		for (unsigned int i = 0; i < N; ++i) {
			if (_data[i] != other._data[i]) {
				return false;
			}
		}
		return true;
	}

	inline bool operator==(const FixedArrayV<T, N> &other) const {
		return equals(other);
	}

	inline bool operator!=(const FixedArrayV<T, N> &other) const {
		return !equals(other);
	}

	inline void operator=(const FixedArrayV<T, N> &other) {
		for (unsigned int i = 0; i < N; ++i) {
			_data[i] = other._data[i];
		}
	}

	inline T *data() {
		return _data;
	}

	inline const T *data() const {
		return _data;
	}

	inline unsigned int size() const {
		return N;
	}

private:
	T _data[N];
};


#endif // FIXED_ARRAYV_H
