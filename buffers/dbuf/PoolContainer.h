/**
 * @file PoolContainer.h
 * @brief Contains the declaration of the PoolContainer class.
 *
 * This container manages a pool of buffers allocated dynamically.
 * Each buffer has a fixed size, and the total number of buffers is adjusted
 * to the next power of two.
 *
 * Created on: Jul 16, 2024
 * Author: admin
 */

#ifndef CHAOSPP_POOLCONTAINER_H_
#define CHAOSPP_POOLCONTAINER_H_

#include "DynamicRingBase.h"
#include <cstring>
#include <cstdlib> // For malloc and free

namespace buffers::dbuf {

/**
 * @class PoolContainer
 * @brief A container that manages a pool of allocated buffers.
 *
 * The PoolContainer class inherits from DynamicRingBase and provides methods
 * to initialize, push data into, retrieve, and manage a pool of buffers.
 */
template <bool UseAtomic = false>
class PoolContainer : public DynamicRingBase<UseAtomic>
{
	using Base      = DynamicRingBase<UseAtomic>;
public:
    /**
     * @brief Default constructor.
     */
    PoolContainer() = default;

    /**
     * @brief Destructor.
     * Frees allocated buffers and resets internal state.
     */
    ~PoolContainer();

    /**
     * @brief Initializes the container with the size of each buffer and total capacity.
     *
     * @param buffer_size The size of each individual buffer.
     * @param capacity The total capacity (number of buffers).
     * @return true if initialization succeeds; false otherwise.
     */
    bool init(const reg buffer_size, const reg capacity);

    /// @brief Checks if the buffer pool is initialized.
    inline bool hasBuffer() const { return (m_pool != nullptr); }

    /// @brief Checks if the pointer matches error value.
    inline bool hasError(const void* const ref) const { return (ref == nullptr); }

    /**
     * @brief Copies data to the next available buffer.
     * @param value Pointer to the data to copy.
     * @param size Size of the data in bytes.
     * @return true if the push is successful; false otherwise.
     */
    bool push(const void* const value, const reg size);

    /**
     * @brief Template method to push a value of type T.
     * @tparam T The type of the value.
     * @param value The value to push.
     * @return true if successful; false otherwise.
     */
    template<class T>
    inline bool push(const T& value);

    /**
     * @brief Gets a pointer to the front buffer.
     *
     * @return Pointer to the front buffer if available, or nullptr if the queue is empty
     *         or the buffer pool is not initialized.
     */
    inline const void* const front() const {
    	// If the queue is empty or the buffer is not initialized, return nullptr
        return Base::isEmpty() || !hasBuffer() ? nullptr
               : m_pool[Base::readPos()];
    }

    /**
     * @brief Template method to get a pointer to the front buffer as type T.
     * @tparam T The desired type.
     * @return A pointer to T if available, or nullptr if T's size exceeds buffer size.
     */
    template<class T>
    inline const T* const front();

    /**
     * @brief Removes the front buffer.
     *
     * If the queue is empty or the buffer pool is not initialized, the method does nothing.
     */
    inline void pop() {
    	// If the queue is empty or the buffer is not initialized, just return
        if (Base::isEmpty() || !hasBuffer()) {
            return;
        }
        Base::incrementTail();
    }

    /**
     * @brief Reserves the next writable buffer.
     *
     * @return A pointer to the reserved buffer if available; otherwise, returns nullptr.
     */
    inline void* const claim() {
    	// If the queue is full or the buffer is not initialized, return nullptr
        return Base::isFull() || !hasBuffer() ? nullptr
               : m_pool[Base::writePos()];
    }

    /**
     * @brief Commits the reserved buffer.
     *
     * If the queue is full or the buffer pool is not initialized, the method does nothing.
     */
    inline void publish() {
    	// If the queue is full or the buffer is not initialized, do nothing
        if (Base::isFull() || !hasBuffer()) {
            return;
        }
        Base::incrementHead();
    }

    /**
     * @brief Returns the current number of elements in the pool.
     * @return The number of elements in the pool.
     */
    inline reg elements() const noexcept {
        return hasBuffer() ? Base::length() : 0;
    }

    /**
     * @brief Returns the total number of buffers in the pool.
     * @return The capacity (number of buffers) of the pool.
     */
    inline reg capacity() const noexcept {
        return hasBuffer() ? Base::capacity() : 0;
    }

    /**
     * @brief Returns the size of each buffer.
     * @return The size of each buffer.
     */
    inline reg size() const noexcept {
        return hasBuffer() ? m_bufferSize : 0;
    }

    /**
     * @brief Provides read-only indexed access to buffers (relative to tail).
     *
     * @param index The index relative to the current tail.
     * @return A pointer to the buffer at the given index, or nullptr if the pool is not initialized.
     */
    const void* const operator[](const reg index) const {
        if (!hasBuffer()) {
            return nullptr;
        }
        return m_pool[(Base::getTail() + index) & Base::getMask()];
    }

private:
    void** m_pool = nullptr;   ///< Array of allocated buffers.
    reg m_bufferSize = 0;      ///< Size of each buffer.
};

template <bool UseAtomic>
PoolContainer<UseAtomic>::~PoolContainer()
{
    // If the pool exists, free each allocated buffer and then free the pool itself.
    if (m_pool) {
        const reg capacity = Base::capacity();
        for (reg i = 0; i < capacity; ++i) {
            std::free(m_pool[i]);
        }
        std::free(m_pool);
    }
    m_pool = nullptr;
    m_bufferSize = 0;
}

template <bool UseAtomic>
bool PoolContainer<UseAtomic>::init(const reg buffer_size, const reg capacity)
{
    // Calculate capacity2 as the next power of 2 of the given capacity.
    const reg capacity2 = Base::next_power_of_2(capacity);

    // If the buffer size is 0 or base initialization fails, reset state and return false.
    if (buffer_size == 0 || !Base::init(capacity2)) {
        m_pool = nullptr;
        m_bufferSize = 0;
        Base::clear();
        return false;
    }

    // Allocate memory for the array of pointers (pool) and initialize them to zero.
    m_pool = static_cast<void**>(std::calloc(capacity2, sizeof(void*)));
    if (!m_pool) {
        m_bufferSize = 0;
        Base::clear();
        return false;
    }

    // For each index in the pool, allocate memory for an individual buffer of size buffer_size.
    for (reg i = 0; i < capacity2; ++i) {
        void* const ptr = std::malloc(buffer_size);
        if (!ptr) {
            // If allocation fails, free all previously allocated buffers and the pool, reset state, and return false.
            for (reg j = 0; j < i; ++j) {
                std::free(m_pool[j]);
            }
            std::free(m_pool);
            m_pool = nullptr;
            m_bufferSize = 0;
            Base::clear();
            return false;
        }
        m_pool[i] = ptr;
    }

    m_bufferSize = buffer_size;
    return true;
}


template <bool UseAtomic>
bool PoolContainer<UseAtomic>::push(const void *const value, const reg size)
{
    // If the data size exceeds the buffer size, or the queue is full, or the pool is not initialized, return false.
    if (size > m_bufferSize || Base::isFull() || !hasBuffer()) {
        return false;
    }

    // Get the target buffer address from the pool based on the current write position.
    void* const target = m_pool[Base::writePos()];
    std::memcpy(target, value, size);

    // Increment the head index.
    Base::incrementHead();
    return true;
}



template <bool UseAtomic>
template<class T>
inline bool PoolContainer<UseAtomic>::push(const T& value)
{
    if(sizeof(T) > m_bufferSize) {
        return false;
    }
    return push(value, sizeof(T));
}

template <bool UseAtomic>
template<class T>
inline const T* const PoolContainer<UseAtomic>::front()
{
    if(sizeof(T) > m_bufferSize) {
        return nullptr;
    }
    return static_cast<const T* const>(this->front());
}

} /* namespace dbuf */

#endif /* CHAOSPP_POOLCONTAINER_H_ */
