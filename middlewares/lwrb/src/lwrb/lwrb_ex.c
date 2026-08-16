/**
 * \file            lwrb_ex.c
 * \brief           Lightweight ring buffer - extended functions
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

#if defined(LWRB_DEV)

/* Do not build if development mode isn't enabled */

#define BUF_IS_VALID(b) ((b) != NULL && (b)->buff != NULL && (b)->size > 0)
#define BUF_MIN(x, y)   ((x) < (y) ? (x) : (y))

/**
 * \brief           Writes data to buffer with overwrite function, if no enough space to hold
 *                  complete input data object.
 * \note            Similar to \ref lwrb_write but overwrites
 * \param[in]       buff: Buffer handle
 * \param[in]       data: Data to write to ring buffer
 * \param[in]       btw: Bytes To Write, length
 * \return          Number of bytes written to buffer, will always return btw
 * \note            Functionality is primary two parts, always writes some linear region, then
 *                      writes the wrap region if there is more data to write. The r indicator is advanced if w overtakes
 *                      it. This operation is a read op as well as a write op. For thread-safety mutexes may be desired,
 *                      see documentation.
 */
lwrb_sz_t
lwrb_overwrite(lwrb_t* buff, const void* data, lwrb_sz_t btw) {
    lwrb_sz_t orig_btw = btw, max_cap;
    const uint8_t* d = data;

    if (!BUF_IS_VALID(buff) || data == NULL || btw == 0) {
        return 0;
    }

    /* Process complete input array */
    max_cap = buff->size - 1; /* Maximum capacity buffer can hold */
    if (btw > max_cap) {
        /*
         * When data to write is larger than max buffer capacity,
         * we can reset the buffer and simply write last part of 
         * the input buffer.
         * 
         * This is done here, by calculating remaining
         * length and then advancing to the end of input buffer
         */
        d += btw - max_cap; /* Advance data */
        btw = max_cap;      /* Limit data to write */
        lwrb_reset(buff);   /* Reset buffer */
    } else {
        /* 
         * Bytes to write is less than capacity
         * We have to perform max one skip operation,
         * but only if free memory is less than
         * btw, otherwise we skip the operation
         * and only write the data.
         */
        lwrb_sz_t f = lwrb_get_free(buff);
        if (f < btw) {
            lwrb_skip(buff, btw - f);
        }
    }
    lwrb_write(buff, d, btw);
    return orig_btw;
}

/**
 * \brief           Move one ring buffer to another, up to the amount of data in the source, or amount 
 *                      of data free in the destination.
 * \param[in]       dest: Buffer handle that the copied data will be written to
 * \param[in]       src:  Buffer handle that the copied data will come from.
 *                      Source buffer will be effectively read upon operation.
 * \return          Number of bytes written to destination buffer
 * \note            This operation is a read op to the source, on success it will update the r index. 
 *                  As well as a write op to the destination, and may update the w index.
 *                  For thread-safety mutexes may be desired, see documentation.
 */
lwrb_sz_t
lwrb_move(lwrb_t* dest, lwrb_t* src) {
    lwrb_sz_t len_to_copy, len_to_copy_orig, src_full, dest_free;

    if (!BUF_IS_VALID(dest) || !BUF_IS_VALID(src)) {
        return 0;
    }
    src_full = lwrb_get_full(src);
    dest_free = lwrb_get_free(dest);
    len_to_copy = BUF_MIN(src_full, dest_free);
    len_to_copy_orig = len_to_copy;

    /* Calculations for available length to copy is done above.
        We safely assume operations inside loop will properly complete. */
    while (len_to_copy > 0) {
        lwrb_sz_t max_seq_read, max_seq_write, op_len;
        const uint8_t* d_src;
        uint8_t* d_dst;

        /* Calculate data */
        max_seq_read = lwrb_get_linear_block_read_length(src);
        max_seq_write = lwrb_get_linear_block_write_length(dest);
        op_len = BUF_MIN(max_seq_read, max_seq_write);
        op_len = BUF_MIN(len_to_copy, op_len);

        /* Get addresses */
        d_src = lwrb_get_linear_block_read_address(src);
        d_dst = lwrb_get_linear_block_write_address(dest);

        /* Byte by byte copy */
        for (lwrb_sz_t i = 0; i < op_len; ++i) {
            *d_dst++ = *d_src++;
        }

        lwrb_advance(dest, op_len);
        lwrb_skip(src, op_len);
        len_to_copy -= op_len;
        if (op_len == 0) {
            /* Hard error... */
            return 0;
        }
    }
    return len_to_copy_orig;
}

#endif /* defined(LWRB_DEV) */
