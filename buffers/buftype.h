/*
 * buftype.h
 *
 *  Created on: Jun 4, 2025
 *      Author: admin
 */

#ifndef BUFFERS_BUFTYPE_H_
#define BUFFERS_BUFTYPE_H_

namespace buffers {

enum class memory_order : int
{
	o_none,
	o_volatile,
	o_atomic,
	o_mutex
};


}


#endif /* BUFFERS_BUFTYPE_H_ */
