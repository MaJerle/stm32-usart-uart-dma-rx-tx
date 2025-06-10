#ifndef CHAOSPP_STATICRINGBASE_H_
#define CHAOSPP_STATICRINGBASE_H_

#include "basic_types.h"
#include <type_traits>
#if __cplusplus >= 201103L
#include <atomic>
#endif

/**
 * @brief A template base class for a static ring buffer with a fixed capacity.
 *
 * Supports two modes for head/tail indexing via template flag UseAtomic:
 *  - UseAtomic = false: plain counters (reg)
 *  - UseAtomic = true:  std::atomic<reg> with relaxed ordering
 *
 * @tparam _Capacity  Capacity (must be power of 2)
 * @tparam UseAtomic  Enable atomic operations if true
 */
namespace buffers::sbuf {

template <reg _Capacity, bool UseAtomic = false>
class StaticRingBase
{
    static_assert(_Capacity == 0 || ((_Capacity & (_Capacity - 1)) == 0),
                  "Capacity must be 0 or a power of 2");

protected:
    StaticRingBase() = default;
    ~StaticRingBase() = default;

public:
    inline constexpr void    clear() noexcept;
    inline constexpr void    hsync() noexcept;
    inline constexpr void    tsync() noexcept;
    inline constexpr bool    isEmpty() const noexcept;
    inline constexpr bool    isFull() const noexcept;
    inline constexpr reg     length() const noexcept;
    inline constexpr bool    isLast() const noexcept;
    inline constexpr reg     emptyCellsCount() const noexcept;
    inline constexpr reg     getHead() const noexcept;
    inline constexpr reg     getTail() const noexcept;
    inline static constexpr reg capacity() noexcept { return _Capacity; }

protected:
    inline static constexpr reg getMask() noexcept { return mask; }
    inline constexpr void       setHead(reg new_head) noexcept;
    inline constexpr void       incrementHead() noexcept;
    inline constexpr void       setTail(reg new_tail) noexcept;
    inline constexpr void       incrementTail() noexcept;
    inline constexpr reg        readPos() const noexcept;
    inline constexpr reg        writePos() const noexcept;
    inline constexpr reg        writeToEndSpace() const noexcept;
    inline constexpr reg        readToEndSpace() const noexcept;

    inline constexpr bool isAtomic() const noexcept { return UseAtomic; }
private:

    static constexpr reg cap  = _Capacity;
    static constexpr reg mask = (_Capacity != 0 ? _Capacity - 1U : 0);

    using counter_t = std::conditional_t<UseAtomic, std::atomic<reg>, volatile reg>;
    static constexpr std::memory_order ordeder = std::memory_order_relaxed;

    counter_t head{0};  ///< Head index (write)
    counter_t tail{0};  ///< Tail index (read)
};

// Definitions

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::clear() noexcept
{
    if constexpr (_Capacity == 0) {
        // nothing to do
    }
    else if constexpr (UseAtomic) {
        head.store(0, std::memory_order_relaxed);
        tail.store(0, std::memory_order_relaxed);
    }
    else {
        head = 0;
        tail = 0;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::hsync() noexcept
{
    if constexpr (_Capacity == 0) {
        // nothing to do
    }
    else if constexpr (UseAtomic) {
        head.store(tail.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
    else {
        head = tail;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::tsync() noexcept
{
    if constexpr (_Capacity == 0) {
        // nothing to do
    }
    else if constexpr (UseAtomic) {
    	tail.store(head.load(std::memory_order_relaxed), std::memory_order_relaxed);
    }
    else {
        tail = head;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr bool StaticRingBase<_Capacity, UseAtomic>::isEmpty() const noexcept
{
    if constexpr (_Capacity == 0) {
        return true;
    }
    else if constexpr (UseAtomic) {
        return head.load(std::memory_order_relaxed) == tail.load(std::memory_order_relaxed);
    }
    else {
        return head == tail;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr bool StaticRingBase<_Capacity, UseAtomic>::isFull() const noexcept
{
    if constexpr (_Capacity == 0) {
        return true;
    }
    else if constexpr (UseAtomic) {
        return (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed)) == _Capacity;
    }
    else {
        return (head - tail) == _Capacity;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::length() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else if constexpr (UseAtomic) {
        return head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed);
    }
    else {
        return head - tail;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr bool StaticRingBase<_Capacity, UseAtomic>::isLast() const noexcept
{
    if constexpr (_Capacity == 0) {
        return false;
    }
    else if constexpr (UseAtomic) {
        return (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed)) == 1U;
    }
    else {
        return (head - tail) == 1U;
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::emptyCellsCount() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else if constexpr (UseAtomic) {
        return cap - (head.load(std::memory_order_relaxed) - tail.load(std::memory_order_relaxed));
    }
    else {
        return cap - (head - tail);
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::getHead() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else {
        if constexpr (UseAtomic) {
            return head.load(std::memory_order_relaxed);
        }
        else {
            return head;
        }
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::getTail() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else {
        if constexpr (UseAtomic) {
            return tail.load(std::memory_order_relaxed);
        }
        else {
            return tail;
        }
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::setHead(reg new_head) noexcept
{
    if constexpr (_Capacity != 0) {
        if constexpr (UseAtomic) {
            head.store(new_head, std::memory_order_relaxed);
        }
        else {
            head = new_head;
        }
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::incrementHead() noexcept
{
    if constexpr (_Capacity != 0) {
        if constexpr (UseAtomic) {
            head.fetch_add(1, std::memory_order_relaxed);
        }
        else {
            ++head;
        }
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::setTail(reg new_tail) noexcept
{
    if constexpr (_Capacity != 0) {
        if constexpr (UseAtomic) {
            tail.store(new_tail, std::memory_order_relaxed);
        }
        else {
            tail = new_tail;
        }
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr void StaticRingBase<_Capacity, UseAtomic>::incrementTail() noexcept
{
    if constexpr (_Capacity != 0) {
        if constexpr (UseAtomic) {
            tail.fetch_add(1, std::memory_order_relaxed);
        }
        else {
            ++tail;
        }
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::readPos() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else if constexpr (UseAtomic) {
        return (tail.load(std::memory_order_relaxed) & mask);
    }
    else {
        return (tail & mask);
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::writePos() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else if constexpr (UseAtomic) {
        return (head.load(std::memory_order_relaxed) & mask);
    }
    else {
        return (head & mask);
    }
}


template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::writeToEndSpace() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else {
        return cap - writePos();
    }
}

template <reg _Capacity, bool UseAtomic>
inline constexpr reg StaticRingBase<_Capacity, UseAtomic>::readToEndSpace() const noexcept
{
    if constexpr (_Capacity == 0) {
        return 0;
    }
    else {
        return cap - readPos();
    }
}

} // namespace buffers::sbuf

#endif // CHAOSPP_STATICRINGBASE_H_
