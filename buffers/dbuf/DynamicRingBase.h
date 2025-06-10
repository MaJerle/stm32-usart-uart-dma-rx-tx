#ifndef CHAOSPP_RINGBASE_H_
#define CHAOSPP_RINGBASE_H_

#include "basic_types.h"
#include <limits>
#if __cplusplus >= 201103L
#include <atomic>
#endif

/**
 * @brief A class for a dynamic ring buffer with variable capacity.
 *
 * The dynamic ring buffer follows the FIFO (First-In-First-Out) principle.
 * The class supports dynamic capacity (power of 2) and optionally
 * uses atomic head/tail indices for SPSC scenarios.
 *
 * @tparam UseAtomic  If true, head/tail are std::atomic<reg> with relaxed ordering.
 */
namespace buffers::dbuf {

template <bool UseAtomic = false>
class DynamicRingBase
{
	static_assert(!UseAtomic || (__cplusplus >= 201103L),
			"Atomics require C++11 or newer");

protected:
	DynamicRingBase() = default;
	explicit DynamicRingBase(reg capacity) { init(capacity); }
	~DynamicRingBase() = default;

public:
	/**
	 * @brief Initializes the ring buffer with the specified capacity.
	 *
	 * @param capacity The capacity of the buffer. Should be a power of 2.
	 * @return true if initialization is successful, false otherwise.
	 */

	bool init(const reg capacity);

	inline constexpr void clear() noexcept;
	inline constexpr bool isEmpty() const noexcept;
	inline constexpr bool isFull() const noexcept;
	inline constexpr reg length() const noexcept;
	inline constexpr bool isLast() const noexcept;
	inline constexpr reg emptyCellsCount() const noexcept;
	inline constexpr reg getHead() const noexcept;
	inline constexpr reg getTail() const noexcept;
	inline constexpr reg capacity() const noexcept;

protected:
	inline constexpr reg getMask() const noexcept;
	inline constexpr void setHead(reg new_head) noexcept;
	inline constexpr void incrementHead() noexcept;
	inline constexpr void setTail(reg new_tail) noexcept;
	inline constexpr void incrementTail() noexcept;
	inline constexpr reg writePos() const noexcept;
	inline constexpr reg readPos() const noexcept;
	inline constexpr reg writeToEndSpace() const noexcept;
	inline constexpr reg readToEndSpace() const noexcept;

	inline constexpr bool isAtomic() const noexcept { return UseAtomic; }

public:
	/**
	 * @brief Checks if a given number is a power of 2.
	 *
	 * @param x The number to check.
	 * @return true if the number is a power of 2, false otherwise.
	 */
	static inline constexpr bool is_power_of_2(const reg x) noexcept {
		return (x != 0) && ((x & (x - 1)) == 0);
	}
	static constexpr reg next_power_of_2(reg n) noexcept {
		if (n == 0) {
			return 1;
		}

		--n;
		n |= n >> 1;  n |= n >> 2;
		n |= n >> 4;  n |= n >> 8;
		n |= n >> 16;

		if constexpr (std::numeric_limits<reg>::digits > 32) {
		    n |= n >> 32;
		}

		return n + 1;
	}

	static inline constexpr reg countr_zero(const reg x) noexcept {
		// Check if m_elementSize is a power of two
		if (!is_power_of_2(x)) {
			// Handle error or return a default value
			return 0; // or another appropriate value
		}

#if defined(__cpp_lib_bitops) // Check for C++20 support
		return std::countr_zero(x);
#elif defined(__GNUC__) // For GCC and Clang compilers
		return __builtin_ctzll(x);
#elif defined(_MSC_VER) // For Microsoft Visual Studio compiler
		unsigned long index;
		_BitScanForward64(&index, x);
		return static_cast<unsigned>(index);
#else
		// If specialized functions are not available, use a standard method
		reg shift 	= 0;
		reg size 	= x;

		while (size > 1) {
			size >>= 1;
			++shift;
		}
		return shift;
#endif
	}

private:
	reg cap  = 0;              ///< buffer capacity (power of two)
	reg mask = 0;              ///< cap - 1, for masking
	using counter_t = std::conditional_t<UseAtomic, std::atomic<reg>, reg>;
	counter_t head{0};         ///< write index
	counter_t tail{0};         ///< read index
};

// Realization

template <bool UseAtomic>
bool DynamicRingBase<UseAtomic>::init(const reg capacity)
{
	if (!is_power_of_2(capacity)) {
		return false;
	}

	cap  = capacity;
	mask = capacity - 1;
	if constexpr (UseAtomic) {
		head.store(0, std::memory_order_relaxed);
		tail.store(0, std::memory_order_relaxed);
	} else {
		head = tail = 0;
	}
	return true;
}

template <bool UseAtomic>
inline constexpr void DynamicRingBase<UseAtomic>::clear() noexcept
{
	if constexpr (UseAtomic) {
		head.store(0, std::memory_order_relaxed);
		tail.store(0, std::memory_order_relaxed);
	} else {
		head = tail = 0;
	}
}

template <bool UseAtomic>
inline constexpr bool DynamicRingBase<UseAtomic>::isEmpty() const noexcept
{
	if constexpr (UseAtomic) {
		return head.load(std::memory_order_relaxed) == tail.load(std::memory_order_relaxed);
	} else {
		return head == tail;
	}
}

template <bool UseAtomic>
inline constexpr bool DynamicRingBase<UseAtomic>::isFull() const noexcept
{
	if constexpr (UseAtomic) {
		return (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed)) == cap;
	} else {
		return (head - tail) == cap;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::length() const noexcept
{
	if constexpr (UseAtomic) {
		return head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed);
	} else {
		return head - tail;
	}
}

template <bool UseAtomic>
inline constexpr bool DynamicRingBase<UseAtomic>::isLast() const noexcept
{
	if constexpr (UseAtomic) {
		return (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed)) == 1;
	} else {
		return (head - tail) == 1U;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::emptyCellsCount() const noexcept
{
	if constexpr (UseAtomic) {
		return cap - (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed));
	} else {
		return cap - (head - tail);
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::getHead() const noexcept
{
	if constexpr (UseAtomic) {
		return head.load(std::memory_order_relaxed);
	} else {
		return head;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::getTail() const noexcept
{
	if constexpr (UseAtomic) {
		return tail.load(std::memory_order_relaxed);
	} else {
		return tail;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::capacity() const noexcept
{
	return cap;
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::getMask() const noexcept
{
	return mask;
}

template <bool UseAtomic>
inline constexpr void DynamicRingBase<UseAtomic>::setHead(reg new_head) noexcept
{
	if constexpr (UseAtomic) {
		head.store(new_head, std::memory_order_relaxed);
	} else {
		head = new_head;
	}
}

template <bool UseAtomic>
inline constexpr void DynamicRingBase<UseAtomic>::incrementHead() noexcept
{
	if constexpr (UseAtomic) {
		head.fetch_add(1, std::memory_order_relaxed);
	} else {
		++head;
	}
}

template <bool UseAtomic>
inline constexpr void DynamicRingBase<UseAtomic>::setTail(reg new_tail) noexcept
{
	if constexpr (UseAtomic) {
		tail.store(new_tail, std::memory_order_relaxed);
	} else {
		tail = new_tail;
	}
}

template <bool UseAtomic>
inline constexpr void DynamicRingBase<UseAtomic>::incrementTail() noexcept
{
	if constexpr (UseAtomic) {
		tail.fetch_add(1, std::memory_order_relaxed);
	} else {
		++tail;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::writePos() const noexcept
{
	if constexpr (UseAtomic) {
		return head.load(std::memory_order_relaxed) & mask;
	} else {
		return head & mask;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::readPos() const noexcept
{
	if constexpr (UseAtomic) {
		return tail.load(std::memory_order_relaxed) & mask;
	} else {
		return tail & mask;
	}
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::writeToEndSpace() const noexcept
{
	return cap - writePos();
}

template <bool UseAtomic>
inline constexpr reg DynamicRingBase<UseAtomic>::readToEndSpace() const noexcept
{
	return cap - readPos();
}

} // namespace buffers::dbuf

#endif // CHAOSPP_RINGBASE_H_
