//
// Created by marmelade on 26/01/23.
//

#ifndef TEST_COMMON_H
#define TEST_COMMON_H

#include <cstdint>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

namespace hal {

#define unused(param) (void) param

    enum class [[nodiscard]] Pin : uint8_t {
        P0  ,   ///< Pin 0
        P1  ,   ///< Pin 1
        P2  ,   ///< Pin 2
        P3  ,   ///< Pin 3
        P4  ,   ///< Pin 4
        P5  ,   ///< Pin 5
        P6  ,   ///< Pin 6
        P7  ,   ///< Pin 7
        P8  ,   ///< Pin 8
        P9  ,   ///< Pin 9
        P10 ,   ///< Pin 10
        P11 ,   ///< Pin 11
        P12 ,   ///< Pin 12
        P13 ,   ///< Pin 13
        P14 ,   ///< Pin 14
        P15 ,   ///< Pin 15
    };

    enum class [[nodiscard]] Port : uint8_t {
        A,  ///< Port A
        B,  ///< Port B
        C,  ///< Port C
        D,  ///< Port D
        E,  ///< Port E
        F,  ///< Port F
    };

    /**
     * Check bits
     * @tparam T
     * @param bit_set
     * @param pos
     * @return
     */
    template<typename T>
    constexpr bool check_bit(const T bit_set, const uint8_t pos) {
        return (bit_set >> pos) & 1U;
    }

    /**
     *
     * @tparam T
     * @param bit_set
     * @param pos
     * @return
     */
    template<typename T>
    constexpr void set_bit(T &bit_set, const uint8_t pos) {

        bit_set |= 1U << pos;
    }

    /**
     *
     * @tparam T
     * @param bit_set
     * @param pos
     * @return
     */
    template<typename T>
    constexpr void clear_bit(T &bit_set, const uint8_t pos) {

        bit_set &= ~(1U << pos);
    }

    /**
     *
     * @tparam T1
     * @param bit_set
     * @param pos
     * @param value
     * @return
     */
    template<typename T1>
    constexpr void set_bit_value(T1 &bit_set, uint8_t pos, bool value) {

        if(value) {
            set_bit(bit_set, pos);
        } else {
            clear_bit(bit_set, pos);
        }
    }

    /**
     *
     * @tparam T1
     * @tparam T2
     * @param bit_set
     * @param position
     * @param value
     * @return
     */
    template<typename T1, typename T2>
    constexpr void change_nth(T1 &bit_set, const uint8_t position, const T2 value) {

        bit_set ^= (-value ^ bit_set) & (1UL << position);
    }

    /**
     *
     * @tparam T1
     * @tparam T2
     * @param bit_set
     * @param position
     * @param mask
     * @return
     */
    template<typename T1, typename T2>
    constexpr T1 get_masked_bytes_at(const T1 bit_set, const uint8_t position, const T2 mask) {

        return bit_set & static_cast<T1>((mask << position));
    }

    /**
     *
     * @tparam T1
     * @tparam T2
     * @param bit_set
     * @param pos
     * @param bit_mask
     * @param range
     * @return
     */
    template<typename T1, typename T2>
    constexpr void set_bits_in_range(T1 &bit_set, uint8_t pos, T2 bit_mask, uint8_t range) {

        for(unsigned int i{0}; i < range; i++) {
            set_bit_value(bit_set, pos + i, (bit_mask >> i) & 0b1);
        }
    }

    /**
     *
     * @tparam range
     * @return
     */
    template<std::size_t range>
    constexpr auto set_first_bits() {

        if constexpr (range <= (sizeof(uint8_t) * 8)) {

            uint8_t tmp{};

            for(std::size_t i{}; i < range; i++) {
                tmp |= (1 << i);
            }

            return tmp;

        } else if constexpr (range <= (sizeof(uint16_t) * 8)) {

            uint16_t tmp{};

            for(std::size_t i{}; i < range; i++) {
                tmp |= (1 << i);
            }

            return tmp;

        } else if constexpr (range <= (sizeof(uint32_t) * 8)) {

            uint32_t tmp{};

            for(std::size_t i{}; i < range; i++) {
                tmp |= (1 << i);
            }

            return tmp;

        } else {

            uint64_t tmp{};

            for(std::size_t i{}; i < range; i++) {
                tmp |= (1 << i);
            }

            return tmp;
        }
    }
} // namespace hal

#pragma GCC diagnostic pop

#endif //TEST_COMMON_H
