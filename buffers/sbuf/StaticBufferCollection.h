#ifndef CHAOSPP_STATICBUFFERCOLLECTION_H_
#define CHAOSPP_STATICBUFFERCOLLECTION_H_

#include "StaticQueue.h"

namespace buffers::sbuf {

template<class Type, reg Capacity>
struct StaticBuffer{
	std::array<Type, Capacity> elem;
	reg size = 0;
};

template <class Type, reg Capacity, reg NCollections>
class StaticBufferCollection : public StaticQueue<StaticBuffer<Type, Capacity>, NCollections>
{
public:

    // Type alias for a StaticQueue of std::array<Type, NElements>
    using Base = StaticQueue<StaticBuffer<Type, Capacity>, NCollections>;

    StaticBufferCollection() = default;
    ~StaticBufferCollection() = default;

};

} /* namespace sbuf */

#endif /* CHAOSPP_STATICBUFFERCOLLECTION_H_ */
