/*
 * PointerRingBuffer.h
 *
 *  Created on: Feb 19, 2025
 *      Author: admin
 *
 * @note This implementation is not thread-safe unless UseAtomic = true is specified,
 * providing atomic operations for head and tail indices. For multi-threaded use,
 * external synchronization (e.g., mutexes) is required for complex operations like put() and get().
 * @note Assumes that `reg` is an unsigned integer type (e.g., uint32_t) with modular
 * arithmetic for handling overflow.
 * @note The capacity for external buffers must be a non-zero power of 2.
 */

#ifndef BUFFERS_DBUF_POINTERRINGBUFFER_H_
#define BUFFERS_DBUF_POINTERRINGBUFFER_H_

#include "DynamicRingBase.h"
#include "RingBuffInterface.h"
#include <cstdlib>

namespace buffers::dbuf {

template <bool UseAtomic = false>
class PointerRB : public IRingBuff<PointerRB<UseAtomic>, DynamicRingBase<UseAtomic>>
{
	using Base      = DynamicRingBase<UseAtomic>;
	using Interface = IRingBuff<PointerRB<UseAtomic>, DynamicRingBase<UseAtomic>>;

public:
	PointerRB() = default;
	~PointerRB();

	bool init(const reg capacity, void* const buffer = nullptr);
	bool installBuffer(void* const buffer = nullptr,
			const reg capacity = 0, const reg tail = 0, const reg head = 0);

	// need to Interface ------------------------------------------------
	inline constexpr bool hasBuffer() const { return m_buffer != nullptr && Base::capacity() > 0; }
	inline constexpr void* const data() const { return m_buffer; }
	inline constexpr void* data() { return m_buffer; }
	// ------------------------------------------------------------------

private:
	void* m_buffer = nullptr;
	bool  m_dealoc = false;
};

template <bool UseAtomic>
PointerRB<UseAtomic>::~PointerRB()
{
	if (m_dealoc && m_buffer) {
		std::free(m_buffer);
	}

	m_buffer = nullptr;
	m_dealoc = false;
	Base::clear();
}

template <bool UseAtomic>
bool PointerRB<UseAtomic>::init(const reg capacity, void* const buffer)
{
	// If memory was already allocated internally, free it.
	if (m_dealoc && m_buffer) {
		std::free(m_buffer);
		m_dealoc = false;
		m_buffer = nullptr;
	}

	// Scenario A: External buffer is not provided (buffer == nullptr).
	if (buffer == nullptr) {
		const reg capacity2 = Base::next_power_of_2(capacity);

		// Initialize the base (dynamic ring buffer) with the specified capacity.
		if (!Base::init(capacity2)) {
			m_buffer = nullptr;
			m_dealoc = false;
			Base::clear();
			return false;
		}

		// Allocate memory with the specified capacity.
		void* const tmp = std::malloc(capacity2);
		if (tmp == nullptr) {
			m_buffer = nullptr;
			m_dealoc = false;
			Base::clear();
			return false;
		}
		m_buffer = tmp;
		m_dealoc = true;  // Memory is managed internally.
	}
	// Scenario B: External buffer is provided.
	else {
		// Initialize the base (dynamic ring buffer) with the specified capacity.
		if (!Base::init(capacity)) {
			m_buffer = nullptr;
			m_dealoc = false;
			Base::clear();
			return false;
		}

		m_buffer = buffer;
		m_dealoc = false; // Memory is not managed internally.
	}

	return true;
}

template <bool UseAtomic>
bool PointerRB<UseAtomic>::installBuffer(void* const buffer,
		const reg capacity, const reg tail, const reg head)
{
	// Scenario A: When no external buffer is provided (buffer == nullptr)
	if (buffer == nullptr) {
		const reg capacity2 = Base::next_power_of_2(capacity);

		// If memory is already allocated and capacity matches, simply clear the buffer.
		if (m_dealoc && m_buffer && (capacity2 == Base::capacity())) {
			if ((head - tail) > Base::capacity()) {
				Base::clear();
			} else {
				Base::setTail(tail);
				Base::setHead(head);
			}
			return true;
		}

		// If previously allocated memory exists, free it.
		if (m_dealoc && m_buffer) {
			std::free(m_buffer);
			m_dealoc = false;
			m_buffer = nullptr;
		}

		// If a new capacity is provided and it differs from current Base::capacity(),
		// initialize the base part with the new capacity.
		if (capacity == 0 || !Base::init(capacity2)) {
			m_buffer = nullptr;
			m_dealoc = false;
			Base::clear();
			return false;
		}

		void* tmp = std::malloc(Base::capacity());
		if (tmp == nullptr) {
			m_buffer = nullptr;
			m_dealoc = false;
			Base::clear();
			return false;
		}

		m_buffer = tmp;
		m_dealoc = true;
	}
	// Scenario B: When an external buffer is provided (buffer != nullptr)
	else {
		// If the provided buffer matches the current buffer and capacity matches,
		// just clear the buffer (no need to change allocation).
		if (buffer == m_buffer && capacity == Base::capacity()) {
			if ((head - tail) > Base::capacity()) {
				Base::clear();
			} else {
				Base::setTail(tail);
				Base::setHead(head);
			}
			return true;
		}

		// If previously allocated memory exists, free it.
		if (m_dealoc && m_buffer) {
			std::free(m_buffer);
			m_dealoc = false;
			m_buffer = nullptr;
		}

		// If a new capacity is provided and it differs from current Base::capacity(),
		// initialize the base part with the new capacity.
		if (!Base::init(capacity)) {
			m_buffer = nullptr;
			m_dealoc = false;
			Base::clear();
			return false;
		}

		// Use the external buffer provided.
		m_buffer = buffer;
		m_dealoc = false;
	}

	// Check if the current indices (head and tail) are within valid bounds.
	// If the difference (head - tail) exceeds the capacity, clear the buffer.
	if ((head - tail) > Base::capacity()) {
		Base::clear();
	} else {
		Base::setTail(tail);
		Base::setHead(head);
	}

	return true;
}

/*
 * WO and RO realization
 *
 *
 */
template <bool UseAtomic = false>
class ReadOnlyRB : public IReadOnlyRB<PointerRB<UseAtomic>>
{
	using Base = PointerRB<UseAtomic>;

public:
	~ReadOnlyRB() = default;
	template <class TRing> inline ReadOnlyRB(TRing&);

	/* PointerRB initialization delete */
	bool init(const reg capacity, void* const buffer = nullptr) = delete;
	bool installBuffer(void* const buffer = nullptr,
			const reg capacity = 0, const reg tail = 0, const reg head = 0) = delete;
};

template <bool UseAtomic>
template<class TRing>
inline ReadOnlyRB<UseAtomic>::ReadOnlyRB(TRing& other)
{
	if (!other.hasBuffer()) {
		return;
	}
	// Initialize this object based on the existing PointerRB
	Base::installBuffer(other.data(), other.capacity(), other.getTail(), other.getHead());
}

template <bool UseAtomic = false>
class WriteOnlyRB : public IWriteOnlyRB<PointerRB<UseAtomic>>
{
	using Base = PointerRB<UseAtomic>;

public:
	~WriteOnlyRB() = default;
	template<class TRing> inline WriteOnlyRB(TRing&);

	/* PointerRB initialization delete */
	bool init(const reg capacity, void* const buffer = nullptr) = delete;
	bool installBuffer(void* const buffer = nullptr,
			const reg capacity = 0, const reg tail = 0, const reg head = 0) = delete;
};

template <bool UseAtomic>
template<class TRing>
inline WriteOnlyRB<UseAtomic>::WriteOnlyRB(TRing& other)
{
	if (!other.hasBuffer()) {
		return;
	}
	// Initialize this object based on the existing PointerRB
	Base::installBuffer(other.data(), other.capacity(), other.getTail(), other.getHead());
}

} // namespace buffers::dbuf

#endif /* BUFFERS_DBUF_POINTERRINGBUFFER_H_ */
