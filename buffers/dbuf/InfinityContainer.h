/*
 * InfinityContainer.h
 *
 * Created on: Nov 16, 2024
 * Author: admin
 *
 * Description: A container that allows infinite writing and reading of fixed-size buffers.
 *              It maintains a pool of buffers and always provides access to the last written buffer.
 */

#ifndef CHAOS_PP_BUFFERS_INFINITYCONTAINER_H_
#define CHAOS_PP_BUFFERS_INFINITYCONTAINER_H_

#include "basic_types.h"
#include <functional>

namespace buffers::dbuf {

/**
 * @class InfinityContainer
 * @brief A container for infinite writing and reading with access to the last written buffer.
 *
 * This class manages a pool of buffers with a fixed size and depth (power of 2).
 * Writing appends data to the next available buffer, and reading retrieves the last written buffer.
 * The buffer wraps around using a mask, allowing "infinite" operation within the pool's depth.
 */
class InfinityContainer
{
public:
	/// @brief Default constructor, initializes an empty container.
	InfinityContainer() = default;

	/// @brief Constructs the container with a specified depth and buffer size.
	/// @param depth The number of buffers in the pool (will be rounded to the next power of 2).
	/// @param bufferSize The size of each buffer in bytes.
	explicit InfinityContainer(const u16 depth, const reg bufferSize);

	/// @brief Destructor, frees allocated buffers.
	~InfinityContainer();

	/// @brief Initializes the container with a pool of buffers.
	/// @param depth The desired number of buffers (rounded to the next power of 2).
	/// @param bufferSize The size of each buffer in bytes.
	/// @return True if initialization succeeds, false if memory allocation fails.
	bool init(const u16 depth, const reg bufferSize);

	// --- Write Operations ---

	/// @brief Appends data to the next available buffer.
	/// @param data Pointer to the data to write.
	/// @param len Length of the data in bytes (must not exceed bufferSize).
	/// @return Number of bytes written, or 0 if the operation fails.
	reg append(const void* data, const reg len);

	/// @brief Returns a pointer to the next buffer available for writing.
	/// @return Pointer to the next buffer, or undefined if the container is not initialized.
	void* const next();

	/// @brief Commits the current write operation, advancing to the next buffer.
	inline void commit() { ++m_currentIndex; }

	// --- Read Operations ---

	/// @brief Retrieves the last written buffer's data.
	/// @param buffer Destination buffer to copy the data into.
	/// @param len Number of bytes to read (must not exceed bufferSize).
	/// @return Number of bytes read, or 0 if no data is available or operation fails.
	reg read(void* const buffer, const reg len);

	/// @brief Returns a pointer to the last written buffer.
	/// @return Pointer to the last written buffer, or nullptr if no data is available.
	void* const last();

	/// @brief Acknowledges reading of the last buffer, updating the read index.
	inline void acknowledge() { m_lastReadIndex = m_currentIndex; }

	/// @brief Checks if there are new data available to be read.
	/// @return True if there are unread data (current index differs from last read index), false otherwise.
	inline bool isEmpty() const { return m_lastReadIndex == m_currentIndex; }

	// --- Information Methods ---

	/// @brief Returns the size of each buffer in the pool.
	/// @return Size of each buffer in bytes.
	inline reg bufferSize() const { return m_bufferSize; }

	/// @brief Returns the number of buffers in the pool.
	/// @return Number of buffers (pool depth).
	inline reg poolSize() const { return m_poolDepth; }

	/// @brief Checks if the container's buffer pool is initialized.
	/// @return True if the pool is initialized, false otherwise.
	inline bool isInitialized() const { return m_bufferPool != nullptr; }

	// --- Buffer Initialization ---

	/// @brief Type definition for a custom initialization function.
	using initializer_t = std::function<void(void* buffer, reg size)>;

	/// @brief Initializes all buffers in the pool using a custom function.
	/// @param function The initialization function to apply to each buffer.
	void initializePool(const initializer_t&);

private:
	void** m_bufferPool = nullptr;       // Array of pointers to individual buffers
	/*volatile*/ reg m_currentIndex = 0; // Current index for writing (increments infinitely)
	/*volatile*/ reg m_lastReadIndex = 0; // Index of the last read buffer
	reg m_mask = 0;                      // Mask for wrapping indices (poolSize - 1)
	reg m_poolDepth = 0;                 // Number of buffers in the pool
	reg m_bufferSize = 0;                // Size of each individual buffer
};

} /* namespace dbuf */

#endif /* CHAOS_PP_BUFFERS_INFINITYCONTAINER_H_ */
