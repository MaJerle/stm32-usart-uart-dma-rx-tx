#ifndef CHAOSPP_STATICQUEUE_H_
#define CHAOSPP_STATICQUEUE_H_

#include "StaticRingBase.h"
#include <array>
#include <cassert>

/**
 * @brief Fixed-size FIFO queue implemented as a static ring buffer.
 *
 * This class implements a fixed-capacity FIFO queue based on a static ring buffer.
 * The capacity is defined at compile time via the template parameter Capacity.
 * It provides methods for enqueuing and dequeuing elements as well as for directly accessing
 * a writable slot through a two-step process: claim() reserves the slot and publish() commits it.
 *
 * @tparam T The type of elements stored in the queue.
 * @tparam Capacity The maximum number of elements the queue can hold.
 */

namespace buffers::sbuf {

template <class T, reg Capacity>
class StaticQueue : public StaticRingBase<Capacity>
{
	using Base = StaticRingBase<Capacity>;

public:
	/**
	 * @brief Default constructor.
	 */
	StaticQueue() = default;

	/**
	 * @brief Default destructor.
	 */
	~StaticQueue() = default;

	inline constexpr bool hasError(const T& ref) const { return &ref == &error_value_; }

	/**
	 * @brief Enqueues an element into the queue.
	 *
	 * If the queue is full, an assertion failure is triggered and the function returns.
	 *
	 * @param value The element to enqueue.
	 */
	inline constexpr void push(const T& value) {
		if (Base::isFull()) {
			return;
		}
		buffer_[Base::writePos()] = value;
		Base::incrementHead();
	}


	/**
	 * @brief Returns a reference to the front element in the queue.
	 *
	 * The front element is the oldest element in the queue.
	 *
	 * @return Reference to the front element.
	 */
	inline constexpr T& front() {
		if(Base::isEmpty()) {
			return error_value_;
		}

		return buffer_[Base::readPos()];
	}

	/**
	 * @brief Removes the front element from the queue.
	 *
	 * If the queue is empty, an assertion failure will occur.
	 */
	inline constexpr void pop() {
		if (Base::isEmpty()) {
			return;
		}
		Base::incrementTail();
	}

	/**
	 * @brief Reserves a writable slot in the queue.
	 *
	 * This method reserves the next available slot for writing and returns a reference to it.
	 * The user must check that the queue is not full before calling this method.
	 *
	 * @return Reference to the writable slot.
	 */
	inline constexpr T& claim() {
		if (Base::isFull()) {
			return error_value_;
		}

		return buffer_[Base::writePos()];
	}

	/**
	 * @brief Publishes the written element.
	 *
	 * After writing to the slot obtained via claim(), call this method to commit the write
	 * by incrementing the head index. An assertion is triggered if the queue is full.
	 */
	inline constexpr void publish() {
		if (Base::isFull()) {
			return;
		}
		Base::incrementHead();
	}


	/**
	 * @brief Returns the current number of elements in the queue.
	 *
	 * @return The size of the queue.
	 */
	inline constexpr reg size() const noexcept {
		return Base::length();
	}

	/**
	 * @brief Returns the maximum number of elements the queue can hold.
	 *
	 * @return The capacity of the queue.
	 */
	static inline constexpr reg capacity() noexcept {
		return Capacity;
	}

	/**
	 * @brief Provides read-only indexed access to elements in the queue.
	 *
	 * @param index The index of the element to access.
	 * @return Constant reference to the element at the specified index.
	 */
	inline constexpr const T& operator[](const reg index) const {
		return buffer_[(Base::getTail() + index) & Base::getMask()];
	}
private:
	std::array<T, Capacity> buffer_{}; ///< Internal buffer for storing elements.
	static inline T error_value_ = T();
};

} /* namespace sbuf */

#endif /* CHAOSPP_STATICQUEUE_H_ */
