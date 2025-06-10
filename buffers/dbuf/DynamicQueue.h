/*
 * DynamicQueue.h
 *
 *  Created on: Feb 20, 2025
 *      Author: admin
 */

#ifndef BUFFERS_DBUF_DYNAMICQUEUE_H_
#define BUFFERS_DBUF_DYNAMICQUEUE_H_

#include "DynamicRingBase.h"

namespace buffers::dbuf {

template <class T, bool UseAtomic = false>
class DynamicQueue : public DynamicRingBase<UseAtomic>
{
	using Base = DynamicRingBase<UseAtomic>;
public:
	/**
	 * @brief Constructor.
	 *
	 * Scenario A: If an external buffer is provided (userBuffer != nullptr),
	 * the queue uses it and sets m_dealoc to false.
	 *
	 * Scenario B: If no external buffer is provided (userBuffer == nullptr),
	 * memory is allocated internally and m_dealoc is set to true.
	 *
	 * @param capacity The capacity of the queue (must be a power of 2).
	 * @param userBuffer Optional external buffer provided by the user.
	 */
	DynamicQueue() = default;
	explicit DynamicQueue(const reg capacity, T* userBuffer = nullptr) { this->init(capacity, userBuffer); }
	~DynamicQueue();

	bool init(const reg capacity, T* const userBuffer = nullptr);

	/**
	 * @brief Sets an external buffer.
	 *
	 * Allows the user to provide their own buffer and sets m_dealoc flag accordingly.
	 *
	 * @param externalBuffer The external buffer pointer.
	 * @param capacity The capacity for the buffer.
	 * @return true if successful; false otherwise.
	 */
	bool installBuffer(T* const buffer, const reg capacity, const reg tail = 0, const reg head = 0);


	/**
	 * @brief Checks if the buffer is available.
	 *
	 * @return true if the buffer exists; false otherwise.
	 */
	inline constexpr bool hasBuffer() const { return (m_buffer != nullptr); }
	inline constexpr bool hasError(const T& ref) const { return &ref == &error_value_; }

	/**
	 * @brief Enqueues an element into the queue.
	 *
	 * Before performing the operation, the method checks if the buffer exists.
	 */
	inline constexpr void push(const T& value) {
		if (!hasBuffer() || Base::isFull()) {
			return;
		}

		m_buffer[Base::writePos()] = value;
		Base::incrementHead();
	}

	/**
	 * @brief Returns a reference to the front element.
	 *
	 * Checks for buffer existence and emptiness.
	 */
	inline constexpr T& front() {
		if (!hasBuffer() ||Base::isEmpty()) {
			return error_value_;
		}
		return m_buffer[Base::readPos()];
	}

	/**
	 * @brief Removes the front element from the queue.
	 *
	 * Checks for buffer existence and emptiness.
	 */
	inline constexpr void pop() {
		if (!hasBuffer() ||Base::isEmpty()) {
			return;
		}
		Base::incrementTail();
	}

	/**
	 * @brief Reserves a writable slot (claim) in the queue.
	 *
	 * Checks for buffer existence and fullness.
	 *
	 * @return Reference to the claimed slot.
	 */
	inline constexpr T& claim() {
		if (!hasBuffer() || Base::isFull()) {
			return error_value_;
		}

		return m_buffer[Base::writePos()];
	}

	/**
	 * @brief Publishes the written element.
	 *
	 * Checks for buffer existence and fullness.
	 */
	inline constexpr void publish() {
		if (!hasBuffer() || Base::isFull()) {
			return;
		}

		Base::incrementHead();
	}

	/**
	 * @brief Returns the current number of elements in the queue.
	 */
	inline constexpr reg size() const noexcept {
		return hasBuffer() ? Base::length() : 0;
	}

	/**
	 * @brief Returns the capacity of the queue.
	 */
	inline constexpr reg capacity() const noexcept {
		return hasBuffer() ? Base::capacity() : 0;
	}

	/**
	 * @brief Provides read-only indexed access to elements in the queue.
	 *
	 * The index is relative to the current tail.
	 */
	inline constexpr const T& operator[](const reg index) const {
		if (!hasBuffer()) {
			return error_value_;
		}
		return m_buffer[(Base::getTail() + index) & Base::getMask()];
	}

private:
	T* m_buffer = nullptr;  ///< Pointer to the dynamically allocated array.
	bool m_dealoc = false;  ///< Flag indicating if the buffer was allocated internally.
	static inline T error_value_ = T();
};



template<class T, bool UseAtomic>
DynamicQueue<T, UseAtomic>::~DynamicQueue()
{
	// Free memory only if it was allocated internally.
	if (m_dealoc && m_buffer) {
		delete[] m_buffer;
	}
	m_buffer = nullptr;
	m_dealoc = false;
}

template<class T, bool UseAtomic>
bool DynamicQueue<T, UseAtomic>::init(const reg capacity, T* const userBuffer)
{
    // If memory was already allocated internally, free it.
	if (m_dealoc && m_buffer) {
		delete[] m_buffer;

		m_buffer = nullptr;
		m_dealoc = false;
	}

	// Scenario A: No external buffer, allocate memory internally.
	if(userBuffer == nullptr) {
		const reg capacity2 = Base::next_power_of_2(capacity);

		// Initialize the base (dynamic ring buffer) with the specified capacity.
        if (!Base::init(capacity2)) {
            goto error;  // If base initialization fails, jump to error handling.
        }

        // Allocate memory with the specified capacity.
        T* const tmp = new T[capacity2];

        if(tmp == nullptr) {
            goto error;  // Allocation failed, handle error.
        }

        m_buffer = tmp;
        m_dealoc = true;  // Memory is managed internally.
	}
	// Scenario B: External buffer provided.
	else {
        // Initialize the base (dynamic ring buffer) with the specified capacity.
        if (!Base::init(capacity)) {
            goto error;  // If base initialization fails, jump to error handling.
        }

        m_buffer = userBuffer;
        m_dealoc = false; // Memory is not managed internally.
	}

	return true;

    /* ------------------------------------ marks --------------------------------------------*/
	error: {
		// In case of any error, reset the internal state.
		m_buffer = nullptr;
		m_dealoc = false;
		Base::clear();
		return false;
	}
}

template<class T, bool UseAtomic>
bool DynamicQueue<T, UseAtomic>::installBuffer(T* const buffer, const reg capacity, const reg tail = 0, const reg head = 0)
{
	// Scenario A: When no external buffer is provided (buffer == nullptr)
	if(buffer == nullptr) {
		const reg capacity2 = Base::next_power_of_2(capacity);

		// If memory is already allocated and capacity matches, simply clear the buffer.
		if( m_dealoc && m_buffer && (capacity2 == Base::capacity()) ) {
			goto borders;
		}

	    // If memory was already allocated internally, free it.
		if (m_dealoc) {
			delete[] m_buffer;

			m_buffer = nullptr;
			m_dealoc = false;
		}

		T* tmp = nullptr;

		// If a new capacity is provided and it differs from current Base::capacity(),
		// initialize the base part with the new capacity.
		if(capacity && capacity2 != Base::capacity()) {
			if (!Base::init(capacity2)) {
				goto error;
			}

			tmp = new T[capacity2];
		}
		// If Base::capacity() is not set (i.e. 0), this is an error.
		else if(Base::capacity() == 0) {
			goto error;
		}
		// Otherwise, allocate memory with the current capacity.
		else {
			tmp = new T[Base::capacity()];
		}

		if(tmp == nullptr) {
			goto error;
		}

		m_buffer = tmp;
		m_dealoc = true;
	}

	// Scenario B: When an external buffer is provided (buffer != nullptr)
	else {
		// If the provided buffer matches the current buffer and capacity matches,
		// just clear the buffer (no need to change allocation).
		if( (buffer == m_buffer) && (capacity == Base::capacity()) ) {
			goto borders;
		}

	    // If memory was already allocated internally, free it.
		if (m_dealoc) {
			delete[] m_buffer;

			m_buffer = nullptr;
			m_dealoc = false;
		}

		// If a new capacity is provided and it differs from current Base::capacity(),
		// initialize the base part with the new capacity.
		if(capacity && capacity != Base::capacity()) {
			if (!Base::init(capacity)) {
				goto error;
			}
		}
		// If Base::capacity() is not set, it's an error.
		else if(Base::capacity() == 0) {
			goto error;
		}

		// Use the external buffer provided.
		m_buffer = buffer;
		m_dealoc = false;
	}

	/* ------------------------------------ marks --------------------------------------------*/
	borders: {
		// Check if the current indices (head and tail) are within valid bounds.
		// If the difference (head - tail) exceeds the capacity, clear the buffer.
		if((head - tail) > Base::capacity()) {
			Base::clear();
		} else {
			Base::setTail(tail);
			Base::setHead(head);
		}

		return true;
	}


	error: {
		// In case of any error, reset the internal state.
		m_buffer = nullptr;
		m_dealoc = false;
		Base::clear();

		return false;
	}

}



} // namespace buffers::dbuf

#endif /* BUFFERS_DBUF_DYNAMICQUEUE_H_ */
