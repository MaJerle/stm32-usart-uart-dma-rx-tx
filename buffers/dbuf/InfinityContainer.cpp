/*
 * InfinityContainer.cpp
 *
 * Created on: Nov 16, 2024
 * Author: admin
 *
 * Description: Implementation of the InfinityContainer class, providing infinite writing
 *              and reading with access to the last written buffer.
 */

#include "InfinityContainer.h"
#include "DynamicRingBase.h"

#include <cstring>
#include <cstdlib>

namespace buffers::dbuf {

InfinityContainer::InfinityContainer(const u16 depth, const reg bufferSize)
{
    init(depth, bufferSize);
}

InfinityContainer::~InfinityContainer()
{
    // Free all allocated buffers and the pool array if it exists
    if (m_bufferPool) {
        for (u16 i = 0; i < m_poolDepth; ++i) {
            std::free(m_bufferPool[i]);
        }
        std::free(m_bufferPool);
        m_bufferPool = nullptr;
    }
}

bool InfinityContainer::init(const u16 depth, const reg bufferSize)
{
    // Ensure depth is adjusted to the next power of 2 for efficient wrapping
    const reg depthPowOfTwo = DynamicRingBase<>::next_power_of_2(depth);

    if (!DynamicRingBase<>::is_power_of_2(depthPowOfTwo)) {
        return false; // Should never happen due to next_power_of_2, but kept for safety
    }

    // Allocate the pool array and initialize it with zeros
    m_bufferPool = static_cast<void**>(std::calloc(depthPowOfTwo, sizeof(void*)));
    if (!m_bufferPool) {
        return false; // Memory allocation failed
    }

    // Allocate each buffer in the pool
    for (reg i = 0; i < depthPowOfTwo; ++i) {
        void* const ptr = std::calloc(1, bufferSize);
        if (!ptr) {
            // If allocation fails, clean up previously allocated buffers and the pool
            for (reg j = 0; j < i; ++j) {
                std::free(m_bufferPool[j]);
            }
            std::free(m_bufferPool);
            m_bufferPool = nullptr;
            return false;
        }
        m_bufferPool[i] = ptr;
    }

    // Set up container parameters
    m_poolDepth = depthPowOfTwo;
    m_bufferSize = bufferSize;
    m_currentIndex = 0;
    m_lastReadIndex = 0;
    m_mask = (depthPowOfTwo - 1U);
    return true;
}

reg InfinityContainer::append(const void* data, const reg len)
{
    // Validate input parameters and container state
    if (!data || len > m_bufferSize || !m_bufferPool) {
        return 0; // No write performed
    }

    // Calculate the write position using the current index and mask
    reg writeIndex = m_currentIndex;
    const reg writePos = writeIndex & m_mask;

    // Copy data into the target buffer
    void* const poolPtr = m_bufferPool[writePos];
    std::memcpy(poolPtr, data, len);

    // Advance the write index
    m_currentIndex = writeIndex + 1U;
    return len; // Return number of bytes written
}

void* const InfinityContainer::next()
{
    // Check if the container is initialized and has data
    if (!m_bufferPool) {
        return nullptr; // No data available
    }

    // Calculate the next write position
    const reg writeIndex = m_currentIndex;
    const reg writePos = writeIndex & m_mask;

    // Return pointer to the next buffer for writing
    return m_bufferPool[writePos];
}

reg InfinityContainer::read(void* const buffer, const reg len)
{
    // Validate input and ensure there's at least one write
    if (!buffer || len > m_bufferSize || !m_bufferPool) {
        return 0; // No data to read
    }

    // Calculate the read position as the previous buffer
    const reg writeIndex = m_currentIndex;
    const reg readIndex = (writeIndex - 1U);
    const reg readPos = readIndex & m_mask;

    // Copy the last written data into the provided buffer
    void* const poolPtr = m_bufferPool[readPos];
    std::memcpy(buffer, poolPtr, len);

    // Update the read index to mark this buffer as read
    m_lastReadIndex = writeIndex;
    return len; // Return number of bytes read
}

void* const InfinityContainer::last()
{
    // Check if the container is initialized and has data
    if (!m_bufferPool) {
        return nullptr; // No data available
    }

    // Calculate the position of the last written buffer
    const reg readIndex = (m_currentIndex - 1U);
    const reg readPos = readIndex & m_mask;

    // Return pointer to the last written buffer
    return m_bufferPool[readPos];
}

void InfinityContainer::initializePool(const initializer_t& function)
{
    // Validate the function and pool state
    if (!function || !m_bufferPool) {
        return;
    }

    // Apply the initialization function to each buffer in the pool
    for (u16 i = 0; i < m_poolDepth; ++i) {
        if (m_bufferPool[i]) {
            function(m_bufferPool[i], m_bufferSize);
        }
    }
}

} /* namespace dbuf */
