/*
 * SequentialBuffer.h
 *
 *  Created on: Dec 27, 2024
 *      Author: admin
 */

#ifndef CHAOS_PP_RING_SEQBUFFER_H_
#define CHAOS_PP_RING_SEQBUFFER_H_

#include "basic_types.h"
#include <array>
#include <cassert> // for assert

namespace buffers::sbuf {

template <class T, reg _Capacity>
class SeqBuffer
{
	static_assert(_Capacity > 0, "Capacity must be greater than 0");
	static_assert(_Capacity < 0x0000FFFF, "Capacity must be less than 0x0000FFFF");

public:
	SeqBuffer() 	= default;
	~SeqBuffer() 	= default;

	// Clearing the buffer
	inline constexpr void clear() { length = 0; currentIndex = 0; }

	// For copying
	inline bool push(const T& val) {
		if (length < cap) {
			buffer[length] = val;  // Copy the value
			++length;
			return true;
		}
		return false;
	}


	// Get the next element from the buffer
	inline T& front() { return buffer[currentIndex]; }

	// Go to the next element (commit)
	inline void pop() {
		if (length == 0) {
			return;
		}

		++currentIndex; // Cyclic index
		if(currentIndex == length) {
			currentIndex = 0;
		}
	}

	// Check if there are any elements to read
	inline constexpr bool isEmpty() const { return length == 0; }
	inline constexpr bool isLast() const { return (currentIndex + 1) == length; }

	// Number of elements in the buffer
	inline constexpr reg size() const { return length; }
	inline constexpr reg capacity() const { return _Capacity; }

    // Iterator methods for range-based for loops
    inline constexpr T* begin() { return buffer.begin(); }
    inline constexpr T* end() { return buffer.data() + length; }


private:
	static constexpr reg cap 		= _Capacity;

	std::array<T, _Capacity> buffer{}; // Buffer for storing elements
	reg length = 0;
	reg currentIndex = 0;
};

} /* namespace sbuf */

#endif /* CHAOS_PP_RING_SEQBUFFER_H_ */
