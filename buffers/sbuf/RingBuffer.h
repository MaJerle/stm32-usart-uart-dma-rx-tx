/*
 * ArrayRingBuffer.h
 *
 *  Created on: Feb 12, 2025
 *      Author: admin
 *
 * @note This implementation is not thread-safe unless UseAtomic = true is specified,
 * providing atomic operations for head and tail indices. For multi-threaded use,
 * external synchronization (e.g., mutexes) is required for complex operations like put() and get().
 * @note Assumes that `reg` is an unsigned integer type (e.g., uint32_t) with modular
 * arithmetic for handling overflow.
 * @note Capacity must be a non-zero power of 2.
 */

#ifndef BUFFERS_SBUF_ARRAYRINGBUFFER_H_
#define BUFFERS_SBUF_ARRAYRINGBUFFER_H_

#include "StaticRingBase.h"
#include "RingBuffInterface.h"

#include <cstdlib>
#include <array>

namespace buffers::sbuf {

/*
 * Array static ring buffer
 */
template <reg Capacity, bool UseAtomic = false>
class ArrayRB : public IRingBuff<ArrayRB<Capacity, UseAtomic>, StaticRingBase<Capacity, UseAtomic>>
{
	using Base 		= StaticRingBase<Capacity, UseAtomic>;
	using Interface = IRingBuff<ArrayRB<Capacity, UseAtomic>, StaticRingBase<Capacity, UseAtomic>>;

public:
	ArrayRB() = default;
	~ArrayRB() = default;

	// need to Interface ------------------------------------------------
	static inline constexpr bool hasBuffer() { return Capacity > 0U; }
	inline constexpr u8* const data() { return m_buffer.data(); }
	inline constexpr const u8* const data() const { return m_buffer.data(); }
	// ------------------------------------------------------------------

private:
	std::array<u8, Capacity> m_buffer = {};
};

/*
 * Pointer static ring buffer
 */
template <reg Capacity, bool UseAtomic = false>
class PointerRB : public IRingBuff<PointerRB<Capacity, UseAtomic>, StaticRingBase<Capacity, UseAtomic>>
{
	using Base 		= StaticRingBase<Capacity, UseAtomic>;
	using Interface = IRingBuff<PointerRB<Capacity, UseAtomic>, StaticRingBase<Capacity, UseAtomic>>;

public:
	PointerRB() = default;
	~PointerRB();

	bool init(void* const buffer = nullptr);
	bool installBuffer(void* const buffer = nullptr, const reg tail = 0, const reg head = 0);

	// need to Interface ------------------------------------------------
	inline constexpr bool hasBuffer() const { return (Capacity > 0) && m_buffer != nullptr; }
	inline constexpr const void* const data() const { return m_buffer; }
	inline constexpr void* const data() { return m_buffer; }
	// ------------------------------------------------------------------

private:
	void* m_buffer 	= nullptr;
	bool m_dealoc 	= false;
};

template<reg Capacity, bool UseAtomic>
PointerRB<Capacity, UseAtomic>::~PointerRB()
{
	if (m_dealoc && m_buffer) {
		std::free(m_buffer);
	}

	m_buffer 	= nullptr;
	m_dealoc 	= false;
	Base::clear();
}

template<reg Capacity, bool UseAtomic>
bool PointerRB<Capacity, UseAtomic>::init(void *const buffer)
{
	if constexpr (Capacity == 0) {
		m_buffer = nullptr;
		m_dealoc = false;
		Base::clear();
		return false;
	} else {
		if (m_dealoc && m_buffer) {
			std::free(m_buffer);
			m_dealoc 	= false;
			m_buffer 	= nullptr;
		}

		if (buffer == nullptr) {
			void* const tmp = std::malloc(Capacity);
			if (tmp == nullptr) {
				m_buffer = nullptr;
				m_dealoc = false;
				Base::clear();
				return false;
			}
			m_buffer 	= tmp;
			m_dealoc 	= true;
		} else {
			m_buffer 	= buffer;
			m_dealoc 	= false;
		}

		Base::clear();
		return true;
	}
}

template<reg Capacity, bool UseAtomic>
bool PointerRB<Capacity, UseAtomic>::installBuffer(void* const buffer, const reg tail, const reg head)
{
	if (buffer == nullptr) {
		if (m_dealoc && m_buffer) {
			if ((head - tail) > Base::capacity()) {
				Base::clear();
			} else {
				Base::setTail(tail);
				Base::setHead(head);
			}
			return true;
		}
		return this->init();
	}

	if (m_dealoc && m_buffer) {
		std::free(m_buffer);
	}
	m_buffer 	= buffer;
	m_dealoc 	= false;

	if ((head - tail) > Base::capacity()) {
		Base::clear();
	} else {
		Base::setTail(tail);
		Base::setHead(head);
	}

	return true;
}

/*
 * Read-only static ring buffer
 */
template <class TRing, bool UseAtomic = false>
class ReadOnlyRB : public IReadOnlyRB<PointerRB<TRing::capacity(), UseAtomic || TRing::isAtomic()>>
{
	static_assert(TRing::capacity() > 0, "Capacity must be greater than 0");

	using Base = PointerRB<TRing::capacity(), UseAtomic || TRing::isAtomic()>;

public:
	~ReadOnlyRB() = default;
	ReadOnlyRB(const TRing&);

	/* PointerRB initialization delete */
	bool init(void* const buffer = nullptr) = delete;
	bool installBuffer(void* const buffer = nullptr, const reg tail = 0, const reg head = 0) = delete;
};

template<class TRing, bool UseAtomic>
inline ReadOnlyRB<TRing, UseAtomic>::ReadOnlyRB(const TRing& other)
{
	if (!other.hasBuffer()) {
		return;
	}
	// Initialize this object based on the existing PointerRB
	Base::installBuffer(other.data(), other.getTail(), other.getHead());
}

/*
 * Write-only static ring buffer
 */
template <class TRing, bool UseAtomic = false>
class WriteOnlyRB : public IWriteOnlyRB<PointerRB<TRing::capacity(), UseAtomic || TRing::isAtomic()>>
{
	static_assert(TRing::capacity() > 0, "Capacity must be greater than 0");

	using Base = PointerRB<TRing::capacity(), UseAtomic || TRing::isAtomic()>;

public:
	~WriteOnlyRB() = default;
	WriteOnlyRB(const TRing&);

	/* PointerRB initialization delete */
	bool init(void* const buffer = nullptr) = delete;
	bool installBuffer(void* const buffer = nullptr, const reg tail = 0, const reg head = 0) = delete;
};

template<class TRing, bool UseAtomic>
inline WriteOnlyRB<TRing, UseAtomic>::WriteOnlyRB(const TRing& other)
{
	if (!other.hasBuffer()) {
		return;
	}
	// Initialize this object based on the existing PointerRB
	Base::installBuffer(other.data(), other.getTail(), other.getHead());
}

} /* namespace sbuf */

#endif /* BUFFERS_SBUF_ARRAYRINGBUFFER_H_ */
