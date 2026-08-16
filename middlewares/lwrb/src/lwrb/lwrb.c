/**
 * \file            lwrb.c
 * \brief           Lightweight ring buffer
 */

/*
 * Copyright (c) 2024 Tilen MAJERLE
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge,
 * publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * This file is part of LwRB - Lightweight ring buffer library.
 *
 * Author:          Tilen MAJERLE <tilen@majerle.eu>
 * Version:         v3.3.0
 */
#include "lwrb/lwrb.h"

/* Memory set and copy functions */
#define BUF_MEMSET      memset
#define BUF_MEMCPY      memcpy

#define BUF_IS_VALID(b) ((b) != NULL && (b)->buff != NULL && (b)->size > 0)
#define BUF_MIN(x, y)   ((x) < (y) ? (x) : (y))
#define BUF_MAX(x, y)   ((x) > (y) ? (x) : (y))
#define BUF_SEND_EVT(b, type, bp)                                                                                      \
    do {                                                                                                               \
        if ((b)->evt_fn != NULL) {                                                                                     \
            (b)->evt_fn((void*)(b), (type), (bp));                                                                     \
        }                                                                                                              \
    } while (0)

/*
 * Optional atomic operations.
 *
 * Disabling this does not just make individual reads/writes non-atomic,
 * it removes any ordering/visibility guarantee between the write and read
 * side entirely. Nothing in this file synchronizes threads or interrupts
 * anymore at that point - it becomes fully the application's job to add
 * whatever barriers, volatile access or locking the target platform needs.
 */
#ifdef LWRB_DISABLE_ATOMIC
#define LWRB_INIT(var, val)        (var) = (val)
#define LWRB_LOAD(var, type)       (var)
#define LWRB_STORE(var, val, type) (var) = (val)
#else
#define LWRB_INIT(var, val)        atomic_init(&(var), (val))
#define LWRB_LOAD(var, type)       atomic_load_explicit(&(var), (type))
#define LWRB_STORE(var, val, type) atomic_store_explicit(&(var), (val), (type))
#endif

/**
 * \brief           Initialize buffer handle to default values with size and buffer data array
 * \param[in]       buff: Ring buffer instance
 * \param[in]       buffdata: Pointer to memory to use as buffer data
 * \param[in]       size: Size of `buffdata` in units of bytes
 *                      Maximum number of bytes buffer can hold is `size - 1`
 * \return          `1` on success, `0` otherwise
 */
uint8_t
lwrb_init(lwrb_t* buff, void* buffdata, lwrb_sz_t size) {
    if (buff == NULL || buffdata == NULL || size == 0) {
        return 0;
    }

    buff->evt_fn = NULL;
    buff->size = size;
    buff->buff = buffdata;
    LWRB_INIT(buff->w_ptr, 0);
    LWRB_INIT(buff->r_ptr, 0);
    return 1;
}

/**
 * \brief           Check if buff is initialized and ready to use
 * \param[in]       buff: Ring buffer instance
 * \return          `1` if ready, `0` otherwise
 */
uint8_t
lwrb_is_ready(lwrb_t* buff) {
    return BUF_IS_VALID(buff);
}

/**
 * \brief           Free buffer memory
 * \note            Since implementation does not use dynamic allocation,
 *                  it just sets buffer handle to `NULL`
 * \param[in]       buff: Ring buffer instance
 */
void
lwrb_free(lwrb_t* buff) {
    if (BUF_IS_VALID(buff)) {
        buff->buff = NULL;
    }
}

/**
 * \brief           Set event function callback for different buffer operations
 * \note            Not thread safe, and not meant to be. Set it once during setup,
 *                      right after \ref lwrb_init, before the buffer is touched from
 *                      more than one thread/interrupt.
 * \param[in]       buff: Ring buffer instance
 * \param[in]       evt_fn: Callback function
 */
void
lwrb_set_evt_fn(lwrb_t* buff, lwrb_evt_fn evt_fn) {
    if (BUF_IS_VALID(buff)) {
        buff->evt_fn = evt_fn;
    }
}

/**
 * \brief           Set custom buffer argument, that can be retrieved in the event function
 * \note            Not thread safe, and not meant to be. Set it once during setup,
 *                      right after \ref lwrb_init, before the buffer is touched from
 *                      more than one thread/interrupt.
 * \param[in]       buff: Ring buffer instance
 * \param[in]       arg: Custom user argument
 */
void
lwrb_set_arg(lwrb_t* buff, void* arg) {
    if (BUF_IS_VALID(buff)) {
        buff->arg = arg;
    }
}

/**
 * \brief           Get custom buffer argument, previously set with \ref lwrb_set_arg
 * \param[in]       buff: Ring buffer instance
 * \return          User argument, previously set with \ref lwrb_set_arg
 */
void*
lwrb_get_arg(lwrb_t* buff) {
    return buff != NULL ? buff->arg : NULL;
}

/**
 * \brief           Write data to buffer.
 *                  Copies data from `data` array to buffer and advances the write pointer for a maximum of `btw` number of bytes.
 * 
 *                  It copies less if there is less memory available in the buffer.
 *                  User must check the return value of the function and compare it to
 *                  the requested write length, to determine if everything has been written
 * 
 * \note            Use \ref lwrb_write_ex for more advanced usage
 *
 * \param[in]       buff: Ring buffer instance
 * \param[in]       data: Pointer to data to write into buffer
 * \param[in]       btw: Number of bytes to write
 * \return          Number of bytes written to buffer.
 *                      When returned value is less than `btw`, there was no enough memory available
 *                      to copy full data array.
 */
lwrb_sz_t
lwrb_write(lwrb_t* buff, const void* data, lwrb_sz_t btw) {
    lwrb_sz_t written = 0;

    if (lwrb_write_ex(buff, data, btw, &written, 0)) {
        return written;
    }
    return 0;
}

/**
 * \brief           Write extended functionality
 * 
 * \param           buff: Ring buffer instance
 * \param           data: Pointer to data to write into buffer
 * \param           btw: Number of bytes to write
 * \param           bwritten: Output pointer to write number of bytes written into the buffer
 * \param           flags: Optional flags.
 *                      \ref LWRB_FLAG_WRITE_ALL: Request to write all data (up to btw).
 *                          Will early return if no memory available
 * \return          `1` if write operation OK, `0` otherwise
 */
uint8_t
lwrb_write_ex(lwrb_t* buff, const void* data, lwrb_sz_t btw, lwrb_sz_t* bwritten, uint16_t flags) {
    lwrb_sz_t tocopy = 0, free = 0, w_ptr = 0;
    const uint8_t* d_ptr = data;

    if (!BUF_IS_VALID(buff) || data == NULL || btw == 0) {
        return 0;
    }

    /* Calculate maximum number of bytes available to write */
    free = lwrb_get_free(buff);
    /* If no memory, or if user wants to write ALL data but no enough space, exit early */
    if (free == 0 || (free < btw && (flags & LWRB_FLAG_WRITE_ALL))) {
        return 0;
    }
    btw = BUF_MIN(free, btw);
    w_ptr = LWRB_LOAD(buff->w_ptr, memory_order_acquire);

    /* Step 1: Write data to linear part of buffer */
    tocopy = BUF_MIN(buff->size - w_ptr, btw);
    BUF_MEMCPY(&buff->buff[w_ptr], d_ptr, tocopy);
    d_ptr += tocopy;
    w_ptr += tocopy;
    btw -= tocopy;

    /* Step 2: Write data to beginning of buffer (overflow part) */
    if (btw > 0) {
        BUF_MEMCPY(buff->buff, d_ptr, btw);
        w_ptr = btw;
    }

    /* Step 3: Check end of buffer */
    if (w_ptr >= buff->size) {
        w_ptr = 0;
    }

    /*
     * Write final value to the actual running variable.
     * This is to ensure no read operation can access intermediate data
     */
    LWRB_STORE(buff->w_ptr, w_ptr, memory_order_release);

    BUF_SEND_EVT(buff, LWRB_EVT_WRITE, tocopy + btw);
    if (bwritten != NULL) {
        *bwritten = tocopy + btw;
    }
    return 1;
}

/**
 * \brief           Read data from buffer.
 *                  Copies data from `data` array to buffer and advances the read pointer for a maximum of `btr` number of bytes.
 * 
 *                  It copies less if there is less data available in the buffer.
 * 
 * \note            Use \ref lwrb_read_ex for more advanced usage
 *
 * \param[in]       buff: Ring buffer instance
 * \param[out]      data: Pointer to output memory to copy buffer data to
 * \param[in]       btr: Number of bytes to read
 * \return          Number of bytes read and copied to data array
 */
lwrb_sz_t
lwrb_read(lwrb_t* buff, void* data, lwrb_sz_t btr) {
    lwrb_sz_t read = 0;

    if (lwrb_read_ex(buff, data, btr, &read, 0)) {
        return read;
    }
    return 0;
}

/**
 * \brief           Read extended functionality
 * 
 * \param           buff: Ring buffer instance
 * \param           data: Pointer to memory to write read data from buffer 
 * \param           btr: Number of bytes to read
 * \param           bread: Output pointer to write number of bytes read from buffer and written to the 
 *                      output `data` variable
 * \param           flags: Optional flags
 *                      \ref LWRB_FLAG_READ_ALL: Request to read all data (up to btr).
 *                          Will early return if no enough bytes in the buffer
 * \return          `1` if read operation OK, `0` otherwise
 */
uint8_t
lwrb_read_ex(lwrb_t* buff, void* data, lwrb_sz_t btr, lwrb_sz_t* bread, uint16_t flags) {
    lwrb_sz_t tocopy = 0, full = 0, r_ptr = 0;
    uint8_t* d_ptr = data;

    if (!BUF_IS_VALID(buff) || data == NULL || btr == 0) {
        return 0;
    }

    /* Calculate maximum number of bytes available to read */
    full = lwrb_get_full(buff);
    if (full == 0 || (full < btr && (flags & LWRB_FLAG_READ_ALL))) {
        return 0;
    }
    btr = BUF_MIN(full, btr);
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_acquire);

    /* Step 1: Read data from linear part of buffer */
    tocopy = BUF_MIN(buff->size - r_ptr, btr);
    BUF_MEMCPY(d_ptr, &buff->buff[r_ptr], tocopy);
    d_ptr += tocopy;
    r_ptr += tocopy;
    btr -= tocopy;

    /* Step 2: Read data from beginning of buffer (overflow part) */
    if (btr > 0) {
        BUF_MEMCPY(d_ptr, buff->buff, btr);
        r_ptr = btr;
    }

    /* Step 3: Check end of buffer */
    if (r_ptr >= buff->size) {
        r_ptr = 0;
    }

    /*
     * Write final value to the actual running variable.
     * This is to ensure no write operation can access intermediate data
     */
    LWRB_STORE(buff->r_ptr, r_ptr, memory_order_release);

    BUF_SEND_EVT(buff, LWRB_EVT_READ, tocopy + btr);
    if (bread != NULL) {
        *bread = tocopy + btr;
    }
    return 1;
}

/**
 * \brief           Read from buffer without changing read pointer (peek only)
 * \note            Not thread safe on its own - safe only if lwrb_peek and lwrb_read
 *                      are always called one after the other from the same thread/interrupt.
 *                      Calling either of them from a second context at the same time is the
 *                      same as having two read exit points and is not supported.
 * \param[in]       buff: Ring buffer instance
 * \param[in]       skip_count: Number of bytes to skip before reading data
 * \param[out]      data: Pointer to output memory to copy buffer data to
 * \param[in]       btp: Number of bytes to peek
 * \return          Number of bytes peeked and written to output array
 */
lwrb_sz_t
lwrb_peek(const lwrb_t* buff, lwrb_sz_t skip_count, void* data, lwrb_sz_t btp) {
    lwrb_sz_t full = 0, tocopy = 0, r_ptr = 0;
    uint8_t* d_ptr = data;

    if (!BUF_IS_VALID(buff) || data == NULL || btp == 0) {
        return 0;
    }

    /*
     * Calculate maximum number of bytes available to read
     * and check if we can even fit to it.
     * 
     * The skip count at size of buffer or above is invalid input,
     * thus we can safely exit the function call
     */
    full = lwrb_get_full(buff);
    if (skip_count >= full) {
        return 0;
    }
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_relaxed);
    r_ptr += skip_count;
    full -= skip_count;
    if (r_ptr >= buff->size) {
        r_ptr -= buff->size;
    }
    btp = BUF_MIN(full, btp);

    /* Step 1: Read data from linear part of the buffer */
    tocopy = BUF_MIN(buff->size - r_ptr, btp);
    BUF_MEMCPY(d_ptr, &buff->buff[r_ptr], tocopy);
    d_ptr += tocopy;
    btp -= tocopy;

    /* Step 2: Read data from the beginning of the buffer (overflow part) */
    if (btp > 0) {
        BUF_MEMCPY(d_ptr, buff->buff, btp);
    }
    return tocopy + btp;
}

/**
 * \brief           Get available size in buffer for write operation
 * \param[in]       buff: Ring buffer instance
 * \return          Number of free bytes in memory
 */
lwrb_sz_t
lwrb_get_free(const lwrb_t* buff) {
    lwrb_sz_t size = 0, w_ptr = 0, r_ptr = 0;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }

    /*
     * Copy buffer pointers to local variables with atomic access.
     *
     * To ensure thread safety (only when in single-entry, single-exit FIFO mode use case),
     * it is important to write buffer r and w values to local w and r variables.
     *
     * Local variables will ensure below if statements will always use the same value,
     * even if buff->w or buff->r get changed during interrupt processing.
     *
     * They may change during load operation, important is that
     * they do not change during if-else operations following these assignments.
     *
     * lwrb_get_free is only called for write purpose, and when in FIFO mode, then:
     * - buff->w pointer will not change by another process/interrupt because we are in write mode just now
     * - buff->r pointer may change by another process. If it gets changed after buff->r has been loaded to local variable,
     *    buffer will see "free size" less than it actually is. This is not a problem, application can
     *    always try again to write more data to remaining free memory that was read just during copy operation
     */
    w_ptr = LWRB_LOAD(buff->w_ptr, memory_order_relaxed);
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_acquire);

    if (w_ptr >= r_ptr) {
        size = buff->size - (w_ptr - r_ptr);
    } else {
        size = r_ptr - w_ptr;
    }

    /* Buffer free size is always 1 less than actual size */
    return size - 1;
}

/**
 * \brief           Get number of bytes currently available in buffer
 * \param[in]       buff: Ring buffer instance
 * \return          Number of bytes ready to be read
 */
lwrb_sz_t
lwrb_get_full(const lwrb_t* buff) {
    lwrb_sz_t size = 0, w_ptr = 0, r_ptr = 0;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }

    /*
     * Copy buffer pointers to local variables.
     *
     * To ensure thread safety (only when in single-entry, single-exit FIFO mode use case),
     * it is important to write buffer r and w values to local w and r variables.
     *
     * Local variables will ensure below if statements will always use the same value,
     * even if buff->w or buff->r get changed during interrupt processing.
     *
     * They may change during load operation, important is that
     * they do not change during if-else operations following these assignments.
     *
     * lwrb_get_full is only called for read purpose, and when in FIFO mode, then:
     * - buff->r pointer will not change by another process/interrupt because we are in read mode just now
     * - buff->w pointer may change by another process. If it gets changed after buff->w has been loaded to local variable,
     *    buffer will see "full size" less than it really is. This is not a problem, application can
     *    always try again to read more data from remaining full memory that was written just during copy operation
     */
    w_ptr = LWRB_LOAD(buff->w_ptr, memory_order_acquire);
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_relaxed);

    if (w_ptr >= r_ptr) {
        size = w_ptr - r_ptr;
    } else {
        size = buff->size - (r_ptr - w_ptr);
    }
    return size;
}

/**
 * \brief           Resets buffer to default values. Buffer size is not modified
 * \note            This function is not thread safe.
 *                      When used, application must ensure there is no active read/write operation
 * \param[in]       buff: Ring buffer instance
 */
void
lwrb_reset(lwrb_t* buff) {
    if (BUF_IS_VALID(buff)) {
        LWRB_STORE(buff->w_ptr, 0, memory_order_release);
        LWRB_STORE(buff->r_ptr, 0, memory_order_release);
        BUF_SEND_EVT(buff, LWRB_EVT_RESET, 0);
    }
}

/**
 * \brief           Get linear address for buffer for fast read
 * \param[in]       buff: Ring buffer instance
 * \return          Linear buffer start address
 */
void*
lwrb_get_linear_block_read_address(const lwrb_t* buff) {
    lwrb_sz_t ptr = 0;

    if (!BUF_IS_VALID(buff)) {
        return NULL;
    }
    ptr = LWRB_LOAD(buff->r_ptr, memory_order_relaxed);
    return &buff->buff[ptr];
}

/**
 * \brief           Get length of linear block address before it overflows for read operation
 * \param[in]       buff: Ring buffer instance
 * \return          Linear buffer size in units of bytes for read operation
 */
lwrb_sz_t
lwrb_get_linear_block_read_length(const lwrb_t* buff) {
    lwrb_sz_t len = 0, w_ptr = 0, r_ptr = 0;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }

    /*
     * Use temporary values in case they are changed during operations.
     * See lwrb_buff_free or lwrb_buff_full functions for more information why this is OK.
     */
    w_ptr = LWRB_LOAD(buff->w_ptr, memory_order_acquire);
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_relaxed);

    if (w_ptr > r_ptr) {
        len = w_ptr - r_ptr;
    } else if (r_ptr > w_ptr) {
        len = buff->size - r_ptr;
    } else {
        len = 0;
    }
    return len;
}

/**
 * \brief           Skip (ignore; advance read pointer) buffer data
 * Marks data as read in the buffer and increases free memory for up to `len` bytes
 *
 * \note            Useful at the end of streaming transfer such as DMA
 * \param[in]       buff: Ring buffer instance
 * \param[in]       len: Number of bytes to skip and mark as read
 * \return          Number of bytes skipped
 */
lwrb_sz_t
lwrb_skip(lwrb_t* buff, lwrb_sz_t len) {
    lwrb_sz_t full = 0, r_ptr = 0;

    if (!BUF_IS_VALID(buff) || len == 0) {
        return 0;
    }

    full = lwrb_get_full(buff);
    len = BUF_MIN(len, full);
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_acquire);
    r_ptr += len;
    if (r_ptr >= buff->size) {
        r_ptr -= buff->size;
    }
    LWRB_STORE(buff->r_ptr, r_ptr, memory_order_release);
    BUF_SEND_EVT(buff, LWRB_EVT_READ, len);
    return len;
}

/**
 * \brief           Get linear address for buffer for fast write
 * \param[in]       buff: Ring buffer instance
 * \return          Linear buffer start address
 */
void*
lwrb_get_linear_block_write_address(const lwrb_t* buff) {
    lwrb_sz_t ptr = 0;

    if (!BUF_IS_VALID(buff)) {
        return NULL;
    }
    ptr = LWRB_LOAD(buff->w_ptr, memory_order_relaxed);
    return &buff->buff[ptr];
}

/**
 * \brief           Get length of linear block address before it overflows for write operation
 * \param[in]       buff: Ring buffer instance
 * \return          Linear buffer size in units of bytes for write operation
 */
lwrb_sz_t
lwrb_get_linear_block_write_length(const lwrb_t* buff) {
    lwrb_sz_t len = 0, w_ptr = 0, r_ptr = 0;

    if (!BUF_IS_VALID(buff)) {
        return 0;
    }

    /*
     * Use temporary values in case they are changed during operations.
     * See lwrb_buff_free or lwrb_buff_full functions for more information why this is OK.
     */
    w_ptr = LWRB_LOAD(buff->w_ptr, memory_order_relaxed);
    r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_acquire);

    if (w_ptr >= r_ptr) {
        len = buff->size - w_ptr;
        /*
         * When read pointer is 0,
         * maximal length is one less as if too many bytes
         * are written, buffer would be considered empty again (r == w)
         */
        if (r_ptr == 0) {
            /*
             * Cannot overflow:
             * - If r is not 0, statement does not get called
             * - buff->size cannot be 0 and if r is 0, len is greater 0
             */
            --len;
        }
    } else {
        len = r_ptr - w_ptr - 1;
    }
    return len;
}

/**
 * \brief           Advance write pointer in the buffer.
 * Similar to skip function but modifies write pointer instead of read
 *
 * \note            Useful when hardware is writing to buffer and application needs to increase number
 *                      of bytes written to buffer by hardware
 * \param[in]       buff: Ring buffer instance
 * \param[in]       len: Number of bytes to advance
 * \return          Number of bytes advanced for write operation
 */
lwrb_sz_t
lwrb_advance(lwrb_t* buff, lwrb_sz_t len) {
    lwrb_sz_t free = 0, w_ptr = 0;

    if (!BUF_IS_VALID(buff) || len == 0) {
        return 0;
    }

    /* Use local variables before writing back to main structure */
    free = lwrb_get_free(buff);
    len = BUF_MIN(len, free);
    w_ptr = LWRB_LOAD(buff->w_ptr, memory_order_acquire);
    w_ptr += len;
    if (w_ptr >= buff->size) {
        w_ptr -= buff->size;
    }
    LWRB_STORE(buff->w_ptr, w_ptr, memory_order_release);
    BUF_SEND_EVT(buff, LWRB_EVT_WRITE, len);
    return len;
}

/**
 * \brief           Searches for a *needle* in an array, starting from given offset.
 * 
 * \note            This function is not thread-safe. 
 * 
 * \param           buff: Ring buffer to search for needle in
 * \param           bts: Constant byte array sequence to search for in a buffer
 * \param           len: Length of the \arg bts array 
 * \param           start_offset: Start offset in the buffer
 * \param           found_idx: Pointer to variable to write index in array where bts has been found
 *                      Must not be set to `NULL`
 * \return          `1` if \arg bts found, `0` otherwise
 */
uint8_t
lwrb_find(const lwrb_t* buff, const void* bts, lwrb_sz_t len, lwrb_sz_t start_offset, lwrb_sz_t* found_idx) {
    lwrb_sz_t full = 0, r_ptr = 0, buff_r_ptr = 0, max_x = 0;
    uint8_t found = 0;
    const uint8_t* needle = bts;

    if (!BUF_IS_VALID(buff) || needle == NULL || len == 0 || found_idx == NULL) {
        return 0;
    }
    *found_idx = 0;

    full = lwrb_get_full(buff);
    /* Verify initial conditions */
    if (full < (len + start_offset)) {
        return 0;
    }

    /* Get actual buffer read pointer for this search */
    buff_r_ptr = LWRB_LOAD(buff->r_ptr, memory_order_relaxed);

    /* Max number of for loops is buff_full - input_len - start_offset of buffer length */
    max_x = full - len;
    for (lwrb_sz_t skip_x = start_offset; !found && skip_x <= max_x; ++skip_x) {
        found = 1; /* Found by default */

        /* Prepare the starting point for reading */
        r_ptr = buff_r_ptr + skip_x;
        if (r_ptr >= buff->size) {
            r_ptr -= buff->size;
        }

        /* Search in the buffer */
        for (lwrb_sz_t idx = 0; idx < len; ++idx) {
            if (buff->buff[r_ptr] != needle[idx]) {
                found = 0;
                break;
            }
            if (++r_ptr >= buff->size) {
                r_ptr = 0;
            }
        }
        if (found) {
            *found_idx = skip_x;
        }
    }
    return found;
}
