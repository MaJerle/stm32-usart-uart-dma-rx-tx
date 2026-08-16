/**
 * \file            lwrb.h
 * \brief           LwRB - Lightweight ring buffer
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
#ifndef LWRB_HDR_H
#define LWRB_HDR_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * \defgroup        LWRB Lightweight ring buffer manager
 * \brief           Lightweight ring buffer manager
 * \{
 */

#if !defined(LWRB_DISABLE_ATOMIC) || __DOXYGEN__
#include <stdatomic.h>

/**
 * \brief           Atomic type for size variable.
 * Default value is set to be `unsigned 32-bits` type
 */
typedef atomic_ulong lwrb_sz_atomic_t;

/**
 * \brief           Size variable for all library operations.
 * Default value is set to be `unsigned 32-bits` type
 */
typedef unsigned long lwrb_sz_t;
#else
/*
 * LWRB_DISABLE_ATOMIC is defined, so the atomic type is dropped in favor of a
 * plain one. This is not "atomicity off, everything else still guaranteed" -
 * there is no ordering or visibility guarantee between the write and read
 * side left in the library at all. If you define this, it is entirely up to
 * the application to make sure the target handles concurrent access safely.
 */
typedef unsigned long lwrb_sz_atomic_t;
typedef unsigned long lwrb_sz_t;
#endif

/**
 * \brief           Event type for buffer operations
 */
typedef enum {
    LWRB_EVT_READ,  /*!< Read event */
    LWRB_EVT_WRITE, /*!< Write event */
    LWRB_EVT_RESET, /*!< Reset event */
} lwrb_evt_type_t;

/**
 * \brief           Buffer structure forward declaration
 */
struct lwrb;

/**
 * \brief           Event callback function type
 * \param[in]       buff: Buffer handle for event
 * \param[in]       evt: Event type
 * \param[in]       bp: Number of bytes written or read (when used), depends on event type
 */
typedef void (*lwrb_evt_fn)(struct lwrb* buff, lwrb_evt_type_t evt, lwrb_sz_t bp);

/* List of flags */
#define LWRB_FLAG_READ_ALL  ((uint16_t)0x0001)
#define LWRB_FLAG_WRITE_ALL ((uint16_t)0x0001)

/**
 * \brief           Buffer structure
 */
typedef struct lwrb {
    uint8_t* buff;  /*!< Pointer to buffer data. Buffer is considered initialized when `buff != NULL` and `size > 0` */
    lwrb_sz_t size; /*!< Size of buffer data. Size of actual buffer is `1` byte less than value holds */
    lwrb_sz_atomic_t r_ptr; /*!< Next read pointer.
                                Buffer is considered empty when `r == w` and full when `w == r - 1` */
    lwrb_sz_atomic_t w_ptr; /*!< Next write pointer.
                                Buffer is considered empty when `r == w` and full when `w == r - 1` */
    lwrb_evt_fn evt_fn;     /*!< Pointer to event callback function */
    void* arg;              /*!< Event custom user argument */
} lwrb_t;

uint8_t lwrb_init(lwrb_t* buff, void* buffdata, lwrb_sz_t size);
uint8_t lwrb_is_ready(lwrb_t* buff);
void lwrb_free(lwrb_t* buff);
void lwrb_reset(lwrb_t* buff);
void lwrb_set_evt_fn(lwrb_t* buff, lwrb_evt_fn fn);
void lwrb_set_arg(lwrb_t* buff, void* arg);
void* lwrb_get_arg(lwrb_t* buff);

/* Read/Write functions */
lwrb_sz_t lwrb_write(lwrb_t* buff, const void* data, lwrb_sz_t btw);
lwrb_sz_t lwrb_read(lwrb_t* buff, void* data, lwrb_sz_t btr);
lwrb_sz_t lwrb_peek(const lwrb_t* buff, lwrb_sz_t skip_count, void* data, lwrb_sz_t btp);

/* Extended read/write functions */
uint8_t lwrb_write_ex(lwrb_t* buff, const void* data, lwrb_sz_t btw, lwrb_sz_t* bwritten, uint16_t flags);
uint8_t lwrb_read_ex(lwrb_t* buff, void* data, lwrb_sz_t btr, lwrb_sz_t* bread, uint16_t flags);

/* Buffer size information */
lwrb_sz_t lwrb_get_free(const lwrb_t* buff);
lwrb_sz_t lwrb_get_full(const lwrb_t* buff);

/* Read data block management */
void* lwrb_get_linear_block_read_address(const lwrb_t* buff);
lwrb_sz_t lwrb_get_linear_block_read_length(const lwrb_t* buff);
lwrb_sz_t lwrb_skip(lwrb_t* buff, lwrb_sz_t len);

/* Write data block management */
void* lwrb_get_linear_block_write_address(const lwrb_t* buff);
lwrb_sz_t lwrb_get_linear_block_write_length(const lwrb_t* buff);
lwrb_sz_t lwrb_advance(lwrb_t* buff, lwrb_sz_t len);

/* Search in buffer */
uint8_t lwrb_find(const lwrb_t* buff, const void* bts, lwrb_sz_t len, lwrb_sz_t start_offset, lwrb_sz_t* found_idx);
lwrb_sz_t lwrb_overwrite(lwrb_t* buff, const void* data, lwrb_sz_t btw);
lwrb_sz_t lwrb_move(lwrb_t* dest, lwrb_t* src);

/**
 * \}
 */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LWRB_HDR_H */
