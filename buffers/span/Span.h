/*
 * Span.h
 *
 * Created on: Dec 5, 2024
 *      Author: Shpegun60
 *
 * Span class provides a non-owning view over a sequence of elements.
 * It can be constructed from raw pointers, C-style arrays, std::array,
 * std::vector, and std::string.
 *
 * This class allows access to the elements, supports subspans, and can
 * be used in range-based for loops thanks to the iterator methods.
 */

#ifndef MY_SPAN_H_
#define MY_SPAN_H_

#include <basic_types.h>
#include <cstring>
#include <vector>
#include <string>
#include <array>

namespace buffers {

template <class T>
class Span {
public:
    // Default constructor
    constexpr Span() = default;

#if __cplusplus >= 202002L
    constexpr ~Span() = default;
#else
    ~Span() = default;
#endif /* __cplusplus >= 202002L */

    using value_type = T;

    // Constructor from pointer and size
    constexpr Span(T* const data, const reg size) : data_(data), size_(size) {}

    // Constructor from C-style array
    template <reg N>
    constexpr Span(T (&array)[N]) : data_(std::data(array)), size_(N) {}

    // Constructor from std::array
    template <reg N>
    constexpr Span(std::array<T, N>& arr) : data_(arr.data()), size_(N) {}

    // Constructor from std::vector
    explicit Span(std::vector<T>& vec) : data_(vec.data()), size_(vec.size()) {}

    // Constructor from string (base type)
    template <class strT>
    Span(std::basic_string<strT>& str) : data_(str.data()), size_(str.size()) {}
    template <class strT>
    Span(const std::basic_string<strT>& str) : data_(str.data()), size_(str.size()) {}

    // Constructor from std::string
    explicit Span(std::string& str) : data_(str.data()), size_(str.size()) {}
    explicit Span(const std::string& str) : data_(const_cast<T*>(str.data())), size_(str.size()) {}


    // Accessor methods
    constexpr inline T* data() const { return data_; }
    constexpr inline reg size() const { return size_; }
    constexpr inline reg size_bytes() const { return size_ * sizeof(T); }

    // Element access
    constexpr inline T& operator[](const reg index) {
        return data_[index];
    }

    constexpr inline T& operator[](const reg index) const {
        return data_[index];
    }

    constexpr inline T& at(const reg index) const {
    	if(index < size_) {
    		return data_[index];
    	}

    	return 0;
    }

    // Check if the span is empty
    constexpr inline bool empty() const { return size_ == 0; }

    // Subspan creation
    constexpr Span<T> subspan(const reg offset, reg count = std::string::npos) const {
        if (offset > size_) {
            return Span<T>();
        } else if (count == std::string::npos || offset + count > size_) {
            count = size_ - offset;
        }
        return Span<T>(data_ + offset, count);
    }

    // Iterator methods for range-based for loops
    constexpr inline T* begin() const { return data_; }
    constexpr inline T* end() const { return data_ + size_; }

private:
    T* data_ = nullptr;
    reg size_ = 0;
};

// Type deduction for C-style array
template <class T, reg N>
Span(T (&)[N]) -> Span<T>; // Deduce type from C-style array

// Type deduction for std::array
template <class T, reg N>
Span(std::array<T, N>&) -> Span<T>; // Deduce type from std::array

// Type deduction for std::string to automatically use Span<char>
template <class T>
Span(std::basic_string<T>& str) -> Span<T>;  // Deduce type from std::string
template <class T>
Span(const std::basic_string<T>& str) -> Span<T>; // Deduce type from const std::string

// make functions

// For C-style array
template <class T>
constexpr inline Span<T> make_span(T* const data, const reg size) {
    return Span<T>(data, size);
}

// For C-style static array
template <class T, reg N>
constexpr inline Span<T> make_span(T (&array)[N]) {
    return Span<T>(array);
}

// For std::array
template <class T, reg N>
constexpr inline Span<T> make_span(std::array<T, N>& arr) {
    return Span<T>(arr);
}

// For std::vector
template <class T>
constexpr inline Span<T> make_span(std::vector<T>& vec) {
    return Span<T>(vec);
}

// For std::string
template <class T>
constexpr inline Span<T> make_span(std::basic_string<T>& str) {
    return Span<T>(str);
}

template <class T>
constexpr inline Span<T> make_span(const std::basic_string<T>& str) {
    return Span<T>(str);
}

// make array -----------------------
template <class T, reg N>
constexpr inline std::array<T, N> make_array(const T (&arr)[N]) {
    static_assert(N > 0, "Array size must be greater than 0.");

    std::array<T, N> result{};

    if constexpr (std::is_trivially_copyable_v<T>) {
        // For trivially copied types, use memcpy for better performance.
        constexpr reg size = N * sizeof(T);
        std::memcpy(result.data(), arr, size);
    } else {
        // For non-trivially copied types, use std::copy for element-wise copying.
        std::copy(arr, arr + N, result.begin());
    }
    return result;
}

} /* namespace buffers */

#endif /* MY_SPAN_H_ */
