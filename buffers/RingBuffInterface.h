/**
 * @file RingBuff.h
 * @brief This header defines an abstract interface for a ring buffer using CRTP.
 *
 * This header provides a set of common operations (read, write, peek, pop, etc.) and
 * CRC calculation for ring buffers. The implementation relies on the derived
 * class to provide specific methods (e.g., hasBuffer(), data(), getTail(), getHead(),
 * setTail(), setHead(), isEmpty(), isFull(), length(), capacity(), getMask(),
 * readToEndSpace(), writeToEndSpace()).
 *
 * Created on: Jul 16, 2024
 * Author: admin
 */

#ifndef _RINGBUFFABSTRACT_H_
#define _RINGBUFFABSTRACT_H_

#include "basic_types.h"        // Defines 'reg', 'u8', etc.
#include "crc/CrcCalculator.h"  // CRC calculation interface

#include <utility>
#include <cstring>
#include <algorithm>

namespace buffers {

//===----------------------------------------------------------------------===//
// Read-only adapter: disables write operations
//===----------------------------------------------------------------------===//
template<class Base>
class IReadOnlyRB : public Base
{
private:
    // Delete clear and all write operations
    inline void clear() = delete;

    // Disable write operations
    template <class T> inline void push(const T&) = delete;
    std::pair<u8* const, const reg> claim() = delete;
    template <class T> inline void publish() = delete;
    void publish(const reg) = delete;

    // Delete PUT operations
    reg put(const void* const, const reg) = delete;
    bool putc(const u8) = delete;
    template <class T> inline bool put(const T&) = delete;

    // Delete stream operator for writing
    template<class T> inline Base& operator<<(const T) = delete;

public:
    template <class TRing>
    bool commit(TRing& other) const {
        if (!other.hasBuffer() || !Base::hasBuffer() || other.data() != Base::data()) {
            return false;
        }
        const reg distance = other.getHead() - Base::getTail();
        if (distance > other.capacity() || distance > Base::capacity()) {
            return false;
        }
        other.setTail(Base::getTail());
        return true;
    }
};

//===----------------------------------------------------------------------===//
// Write-only adapter: disables read operations
//===----------------------------------------------------------------------===//
template<class Base>
class IWriteOnlyRB : public Base
{
private:
    // Delete clear and all read operations
    inline void clear() = delete;

    // Disable read operations
    std::pair<const u8* const, const reg> front() const = delete;
    template <class T> inline void pop() = delete;
    void pop(const reg) = delete;

    // Delete GET operations
    reg get(void* const, const reg) = delete;
    u8 getc() = delete;
    template <class T> inline T get() = delete;

    // Delete PEEK operations
    reg peek(void* const, const reg) const = delete;
    u8 peekc() const = delete;
    template <class T> inline T peek() const = delete;

    // Delete stream operator for reading
    template<class T> Base& operator>>(T&) = delete;

public:
    template<class TRing>
    bool commit(TRing& other) const {
        if (!other.hasBuffer() || !Base::hasBuffer() || other.data() != Base::data()) {
            return false;
        }
        const reg distance = Base::getHead() - other.getTail();
        if (distance > other.capacity() || distance > Base::capacity()) {
            return false;
        }
        other.setHead(Base::getHead());
        return true;
    }
};

//===----------------------------------------------------------------------===//
// CRTP interface providing common ring buffer operations
//===----------------------------------------------------------------------===//
template <class Derived, class Interface>
class IRingBuff : public Interface
{
    template<class Base> friend class IReadOnlyRB;
    template<class Base> friend class IWriteOnlyRB;

protected:
    IRingBuff() = default;
    ~IRingBuff() = default;

public:
    // C++ Standard Access Methods
    std::pair<const u8* const, const reg> front() const;
    template <class T> inline void pop() { pop(sizeof(T)); }
    inline void pop(const reg size);

    // Write operations
    template <class T> inline void push(const T& value) { put<T>(value); }
    std::pair<u8* const, const reg> claim();
    template <class T> inline void publish() { publish(sizeof(T)); }
    inline void publish(const reg size);

    inline constexpr reg size() const noexcept { return Interface::length(); }

    // GET Operations
    reg get(void* const buffer, const reg size);
    u8 getc();
    template <class T> inline T get();

    // PEEK Operations
    reg peek(void* const buffer, const reg size) const;
    u8 peekc() const;
    template <class T> inline T peek() const;

    // PUT Operations
    reg put(const void* const buffer, const reg size);
    bool putc(const u8 c);
    template <class T> inline bool put(const T& value);

    // STREAM Operators
    template<class T> inline Derived& operator<<(const T& value) {
        put<T>(value);
        return static_cast<Derived&>(*this);
    }
    template<class T> inline Derived& operator>>(T& value) {
        value = get<T>();
        return static_cast<Derived&>(*this);
    }

    // CRC Calculation
    static void crcFrom(CrcCalculator &crc, const Derived &from);
    inline void calcCRC(CrcCalculator &crc);
};

/*
  * *********************************************************
  * C++ Standard Access Methods Implementation
  * *********************************************************
  */

template<class Derived, class Interface>
std::pair<const u8* const, const reg> IRingBuff<Derived, Interface>::front() const
{
    const Derived& base = static_cast<const Derived&>(*this);

    if(!base.hasBuffer() || Interface::isEmpty()) {
        return {nullptr, 0};
    }

    // move to registers------------------------------------------
    const reg avail = Interface::length();                	// get n elements
    const reg toEnd = Interface::readToEndSpace();          // get remaining from tail to end
    // do logic ------------------------------------------------------------
    return {static_cast<const u8* const>(base.data()) + Interface::readPos(),
            std::min(avail, toEnd)};
}

template<class Derived, class Interface>
inline void IRingBuff<Derived, Interface>::pop(const reg size)
{
    if(size == 0U) {
        return;
    }

    if(size > Interface::length()) {
        // proceed signals
        Interface::setTail(Interface::getTail() + Interface::length());
    } else {
        // proceed signals
        Interface::setTail(Interface::getTail() + size);
    }
}

template<class Derived, class Interface>
std::pair<u8* const, const reg> IRingBuff<Derived, Interface>::claim()
{
    Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isFull()) {
        return {nullptr, 0};
    }

    // move to registers------------------------------------------------------
    const reg space = Interface::emptyCellsCount();
    const reg toEnd = Interface::writeToEndSpace();
    // do logic ------------------------------------------------------------
    return {static_cast<u8*>(base.data()) + Interface::writePos(),
            std::min(space, toEnd)};
}

template<class Derived, class Interface>
inline void IRingBuff<Derived, Interface>::publish(const reg size)
{
    if( (size == 0U) || (size > Interface::emptyCellsCount()) ) {
        return;
    }

    // proceed signals
    Interface::setHead(Interface::getHead() + size);
}

/*
 * *********************************************************
 * GET Operations Implementation
 * *********************************************************
 */

template <class Derived, class Interface>
reg IRingBuff<Derived, Interface>::get(void *const buffer, const reg size)
{
    Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isEmpty()) {
        return 0;
    }

    // move to registers------------------------------------------
    const u8* const ring_ptr   = static_cast<u8*>(base.data());

    const reg tail_reg         = Interface::getTail();
    const reg msk_reg          = Interface::getMask();

    const reg n_elem           = Interface::length();                        // get n elements
    const reg size_constr      = std::min(size, n_elem);                     // constrain input size
    const reg tail_pos         = tail_reg & msk_reg;                         // get tail position
    const reg remaining_to_end = Interface::capacity() - tail_pos;           // get remaining from tail to end

    // do logic --------------------------------------------------
    if(size_constr > remaining_to_end) {
        const reg remaining_n = size_constr - remaining_to_end;
        /* first get the data from fifo->out until the end of the buffer */
        std::memcpy(buffer, ring_ptr + tail_pos, remaining_to_end);
        /* then get the rest (if any) from the beginning of the buffer */
        std::memcpy(static_cast<u8*>(buffer) + remaining_to_end, ring_ptr, remaining_n);
    } else {
        /* get all the data */
        std::memcpy(buffer, ring_ptr + tail_pos, size_constr);
    }

    // proceed signals
    Interface::setTail(tail_reg + size_constr);
    return size_constr;
}

template <class Derived, class Interface>
u8 IRingBuff<Derived, Interface>::getc()
{
    Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isEmpty()) {
        return 0;
    }

    // move to registers------------------------------------------
    const u8* const ring_ptr = static_cast<u8*>(base.data());

    const reg tail_reg       = Interface::getTail();
    const reg msk_reg        = Interface::getMask();
    const reg tail_pos       = tail_reg & msk_reg;                    // get tail position

    // do logic --------------------------------------------------
    const u8 value = *(ring_ptr + tail_pos);

    // proceed signals
    Interface::setTail(tail_reg + 1U);
    return value;
}

template <class Derived, class Interface>
template<class T>
inline T IRingBuff<Derived, Interface>::get()
{
    if constexpr (std::is_same_v<T, u8>) {
        return getc();
    } else {
        T value;
        if (get(&value, sizeof(T)) != sizeof(T)) {
            return T();
        }
        return value;
    }
}

/*
  * *********************************************************
  * PEEK Operations Implementation
  * *********************************************************
  */

template <class Derived, class Interface>
reg IRingBuff<Derived, Interface>::peek(void *const buffer, const reg size) const
{
    const Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isEmpty()) {
        return 0;
    }

    // move to registers------------------------------------------
    const u8* const ring_ptr = static_cast<u8*>(base.data());

    const reg tail_reg         = Interface::getTail();
    const reg msk_reg          = Interface::getMask();

    const reg n_elem           = Interface::length();                        // get n elements
    const reg size_constr      = std::min(size, n_elem);                     // constrain input size
    const reg tail_pos         = tail_reg & msk_reg;                         // get tail position
    const reg remaining_to_end = Interface::capacity() - tail_pos;           // get remaining from tail to end

    // do logic --------------------------------------------------
    if(size_constr > remaining_to_end) {
        const reg remaining_n = size_constr - remaining_to_end;
        /* first get the data from fifo->out until the end of the buffer */
        std::memcpy(buffer, ring_ptr + tail_pos, remaining_to_end);
        /* then get the rest (if any) from the beginning of the buffer */
        std::memcpy(static_cast<u8*>(buffer) + remaining_to_end, ring_ptr, remaining_n);
    } else {
        /* get all the data */
        std::memcpy(buffer, ring_ptr + tail_pos, size_constr);
    }
    return size_constr;
}

template <class Derived, class Interface>
u8 IRingBuff<Derived, Interface>::peekc() const
{
    const Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isEmpty()) {
        return 0;
    }

    // move to registers------------------------------------------
    const u8* const ring_ptr = static_cast<u8*>(base.data());

    const reg tail_reg  = Interface::getTail();
    const reg msk_reg   = Interface::getMask();
    const reg tail_pos  = tail_reg & msk_reg;                    // get tail position

    // do logic --------------------------------------------------
    const u8 value = *(ring_ptr + tail_pos);
    return value;
}

template <class Derived, class Interface>
template<class T>
inline T IRingBuff<Derived, Interface>::peek() const
{
    if constexpr (std::is_same_v<T, u8>) {
        return peekc();
    } else {
        T value;
        if (peek(&value, sizeof(T)) != sizeof(T)) {
            return T();
        }
        return value;
    }
}

/*
 * *********************************************************
 * PUT Operations Implementation
 * *********************************************************
 */

template <class Derived, class Interface>
reg IRingBuff<Derived, Interface>::put(const void *const buffer, const reg size)
{
    Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isFull()) {
        return 0;
    }

    // move to registers------------------------------------------------------
    u8* const ring_ptr = static_cast<u8*>(base.data());

    const reg tail_reg        = Interface::getTail();
    const reg head_reg        = Interface::getHead();
    const reg msk_reg         = Interface::getMask();
    const reg cap_reg         = Interface::capacity();

    const reg head_pos        = head_reg & msk_reg;
    const reg remaining_to_end = cap_reg - head_pos;                    // get remaining from head to end
    const reg n_empty         = cap_reg - (head_reg - tail_reg);        // get empty cells (good version)
    const reg size_constr     = std::min(size, n_empty);                // constrain input size

    // do logic ------------------------------------------------------------
    if(size_constr > remaining_to_end) {
        const reg remaining_n = size_constr - remaining_to_end;
        /* first put the data starting from fifo->in to buffer end */
        std::memcpy(ring_ptr + head_pos, buffer, remaining_to_end);
        /* then put the rest (if any) at the beginning of the buffer */
        std::memcpy(ring_ptr, static_cast<const u8*>(buffer) + remaining_to_end, remaining_n);
    } else {
        /* put all the data */
        std::memcpy(ring_ptr + head_pos, buffer, size_constr);
    }

    // proceed signals
    Interface::setHead(head_reg + size_constr);
    return size_constr;
}

template <class Derived, class Interface>
bool IRingBuff<Derived, Interface>::putc(const u8 c)
{
    Derived& base = static_cast<Derived&>(*this);

    if(!base.hasBuffer() || Interface::isFull()) {
        return 0;
    }

    // move to registers------------------------------------------------------
    u8* const ring_ptr = static_cast<u8*>(base.data());

    const reg head_reg = Interface::getHead();
    const reg msk_reg  = Interface::getMask();
    const reg head_pos = head_reg & msk_reg;

    *(ring_ptr + head_pos) = c;

    // proceed signals
    Interface::setHead(head_reg + 1U);
    return true;
}

template <class Derived, class Interface>
template<class T>
inline bool IRingBuff<Derived, Interface>::put(const T& value)
{
    if constexpr (std::is_same_v<T, u8>) {
        return putc(value);
    } else {
        return put(&value, sizeof(T)) == sizeof(T);
    }
}

/*
  * *********************************************************
  * CRC Calculation Implementation
  * *********************************************************
  */

template <class Derived, class Interface>
void IRingBuff<Derived, Interface>::crcFrom(CrcCalculator &crc, const Derived &from)
{
    if(!from.hasBuffer() || from.isEmpty()) {
        return;
    }

    // move to registers------------------------------------------
    const u8* const ring_ptr = static_cast<u8*>(from.data());

    const reg tail_reg         = from.getTail();
    const reg msk_reg          = from.getMask();

    const reg n_elem           = from.length();                        // get n elements
    const reg tail_pos         = tail_reg & msk_reg;                   // get tail position
    const reg remaining_to_end = from.capacity() - tail_pos;           // get remaining from tail to end

    // do logic --------------------------------------------------
    if(n_elem > remaining_to_end) {
        const reg remaining_n = n_elem - remaining_to_end;
        /* first get the data from fifo->out until the end of the buffer */
        crc.array(ring_ptr + tail_pos, remaining_to_end);
        /* then get the rest (if any) from the beginning of the buffer */
        crc.array(ring_ptr, remaining_n);
    } else {
        /* get all the data */
        crc.array(ring_ptr + tail_pos, n_elem);
    }
}

template <class Derived, class Interface>
inline void IRingBuff<Derived, Interface>::calcCRC(CrcCalculator &crc)
{
    const Derived& base = static_cast<Derived&>(*this);
    IRingBuff::crcFrom(crc, base);
}

} /* namespace buffers */

#endif /* _RINGBUFFABSTRACT_H_ */
