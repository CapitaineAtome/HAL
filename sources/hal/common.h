//
// Created by marmelade on 26/01/23.
//

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

namespace hal {

#define unused_param(param) (void) param

// #define check_bit(bit_set, pos) (((bit_set) >> (pos)) & 1U)

// #define set_bit(bit_set, pos) (bit_set) |= (1U << (pos))

// #define clear_bit(bit_set, pos) bit_set &= ~(1U << (pos))

/*#define set_bit_value(bit_set, pos, value) { \
        if (value) { \
            set_bit(bit_set, pos); \
        } else { \
            clear_bit(bit_set, pos); \
        } \
    }*/

    template<typename T>
    constexpr bool check_bit(const T bit_set, const uint32_t pos) {
        return (bit_set >> pos) & 1U;
    }

    template<typename T>
    constexpr void set_bit(T &bit_set, const uint32_t pos) {

        bit_set |= 1U << pos;
    }

    template<typename T>
    constexpr void clear_bit(T &bit_set, const uint32_t pos) {

        bit_set &= ~(1U << pos);
    }


    template<typename T1>
    constexpr void set_bit_value(T1 bit_set, uint32_t pos, bool value) {

        if(value) {
            set_bit(bit_set, pos);
        } else {
            clear_bit(bit_set, pos);
        }
    }
} // namespace hal

#pragma GCC diagnostic pop

#endif //TEST_COMMON_H
