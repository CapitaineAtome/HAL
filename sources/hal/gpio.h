//
// Created by marmelade on 25/01/23.
//

#ifndef DRIVER_HAL_GPIO_H
#define DRIVER_HAL_GPIO_H

#include "common.h"
#include "memorymap.h"

#include <cstdint>
#include <utility>

/*
 * LD3 USER
 * The LD3 USER green LED is connected to the following STM32G031K8T6 I/O:
 * PB3, if the configuration is SB12 ON, and SB13 OFF
 * PC6, if the configuration is SB12 OFF, and SB13 ON (default configuration)
 */

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

class GPIOPort {
public:

    enum class [[nodiscard]] Port : uint32_t {
        A = PORTA_MEMORY,  ///< Port A
        B = PORTB_MEMORY,  ///< Port B
        C = PORTC_MEMORY,  ///< Port C
        D = PORTD_MEMORY,  ///< Port D
        E = PORTE_MEMORY,  ///< Port E
        F = PORTF_MEMORY,  ///< Port F
    };

    enum class [[nodiscard]] Pin : uint8_t {
        P0,    ///< Pin 0
        P1,    ///< Pin 1
        P2,    ///< Pin 2
        P3,    ///< Pin 3
        P4,    ///< Pin 4
        P5,    ///< Pin 5
        P6,    ///< Pin 6
        P7,    ///< Pin 7
        P8,    ///< Pin 8
        P9,    ///< Pin 9
        P10,   ///< Pin 10
        P11,   ///< Pin 11
        P12,   ///< Pin 12
        P13,   ///< Pin 13
        P14,   ///< Pin 14
        P15,   ///< Pin 15
    };

    enum class [[nodiscard]] Mode : uint8_t {
        INPUT = 0b00,       ///< Input mode
        OUTPUT = 0b01,      ///< General purpose output mode
        ALTERNATE = 0b10,   ///< Alternate function mode
        ANALOG = 0b11,      ///< Analog mode ( reset state )
    };

    enum class [[nodiscard]] OutputType : uint8_t {
        PUSH_PULL = 0,  ///< Output push-pull ( reset state )
        OPEN_DRAIN = 1, ///< Output open-drain
    };

    enum class [[nodiscard]] OutputSpeed : uint8_t {
        VERY_LOW_SPEED = 0b00,  ///< Very low speed
        LOW_SPEED = 0b01,       ///< Low speed
        HIGH_SPEED = 0b10,      ///< High speed
        VERY_HIGH_SPEED = 0b11, ///< Very high speed
    };

    enum class [[nodiscard]] PullMode : uint8_t {
        NO_PULL = 0,        ///< No pull-up or pull-down
        PULL_UP = 0b01,     ///< Pull-up
        PULL_DOWN = 0b10,   ///< Pull-down
    };

    enum class [[nodiscard]] BitReset : uint8_t {
        NO_ACTION,  ///< No action on the corresponding ODx bit
        RESET,      ///< Reset the corresponding ODx bit
    };

    enum class [[nodiscard]] Lock : uint8_t {
        NOT_LOCKED = 0, ///< Port configuration not locked
        LOCKED = 1,     ///< Port configuration locked
    };

    enum class [[nodiscard]] AlternateFunction : uint8_t {
        AF0 = 0b0000,   ///< Alternate Function 0
        AF1 = 0b0001,   ///< Alternate Function 1
        AF2 = 0b0010,   ///< Alternate Function 2
        AF3 = 0b0011,   ///< Alternate Function 3
        AF4 = 0b0100,   ///< Alternate Function 4
        AF5 = 0b0101,   ///< Alternate Function 5
        AF6 = 0b0110,   ///< Alternate Function 6
        AF7 = 0b0111,   ///< Alternate Function 7
    };

    explicit GPIOPort(const Port port) : gpio_reg{reinterpret_cast<gpio_registers *>(std::to_underlying(port))} {
    }

    ~GPIOPort() {

        gpio_reg = nullptr;
    }

    void Mode(const enum Pin pin, const enum Mode mode) {

        hal::set_bit_value( gpio_reg->mode_reg, std::to_underlying(pin) * 2,
                       std::to_underlying(mode) & 0b01 );

        hal::set_bit_value( gpio_reg->mode_reg, std::to_underlying(pin) * 2u + 1u,
                       std::to_underlying(mode) & 0b10 );
    }

    enum Mode Mode(const Pin pin) {

        return static_cast<enum Mode>(  hal::check_bit(gpio_reg->mode_reg, std::to_underlying(pin) * 2u + 1u) << 1u |
                                        hal::check_bit(gpio_reg->mode_reg, std::to_underlying(pin) * 2u));
    }

    void OutputType(const Pin pin, const OutputType otype) {

        hal::set_bit_value( gpio_reg->otype_reg, std::to_underlying(pin),
                       std::to_underlying(otype));
    }

    enum OutputType OutputType(const Pin pin) {

        return static_cast<enum OutputType>( hal::check_bit( gpio_reg->otype_reg,
                                                        std::to_underlying(pin)));
    }

    void OutputSpeed(const Pin pin, const enum OutputSpeed ospeed) {

        hal::set_bit_value( gpio_reg->ospeed_reg, std::to_underlying(pin) * 2,
                       std::to_underlying(ospeed) & 0b01 );

        hal::set_bit_value( gpio_reg->ospeed_reg, std::to_underlying(pin) * 2u + 1u,
                       std::to_underlying(ospeed) & 0b10 );
    }

    enum OutputSpeed OutputSpeed(const Pin pin) {

        return static_cast<enum OutputSpeed>( hal::check_bit(gpio_reg->ospeed_reg, std::to_underlying(pin) * 2u + 1u) << 1u |
                                              hal::check_bit(gpio_reg->ospeed_reg, std::to_underlying(pin) * 2u));
    }

    void PullMode(const Pin pin, const enum OutputSpeed ospeed) {

        hal::set_bit_value( gpio_reg->pupd_reg, std::to_underlying(pin) * 2,
                       std::to_underlying(ospeed) & 0b01 );

        hal::set_bit_value( gpio_reg->pupd_reg, std::to_underlying(pin) * 2u + 1u,
                       std::to_underlying(ospeed) & 0b10 );
    }

    enum PullMode PullMode(const Pin pin) {

        return static_cast<enum PullMode>( hal::check_bit(gpio_reg->pupd_reg, std::to_underlying(pin) * 2u + 1u) << 1u |
                                           hal::check_bit(gpio_reg->pupd_reg, std::to_underlying(pin) * 2u));
    }

    [[nodiscard]] bool Read(const Pin pin) {
        auto tmp = gpio_reg->id_reg;
        return hal::check_bit(tmp, std::to_underlying(pin));
    }

    void Write(const Pin pin, bool value) {

        if(value) {
            // set
            hal::set_bit(gpio_reg->bsr_reg, std::to_underlying(pin));

        } else {
            // reset
            hal::set_bit(gpio_reg->br_reg, std::to_underlying(pin));

        }
    }

    void Toggle(const Pin pin) {

        Write(pin, Read(pin));
    }

    void LockKey(const Pin pin) {

        /*
         * LOCK key write sequence:
         *
         * WR Lock[16] = '1' + pin
         * WR Lock[16] = '0' + pin
         * WR Lock[16] = '1' + pin
         * RD Lock[16]
         * RD Lock[16] = '1'
         */
        // TODO: A implémenter

        unused_param(pin);
    }

protected:

private:

    // TODO: Set read only registers to const ?
    struct gpio_registers {
        volatile std::uint32_t mode_reg;     ///< Mode register : These bits are written by software to configure the I/O mode
        volatile std::uint32_t otype_reg;    ///< Output Type register : These bits are written by software to configure the I/O output type
        volatile std::uint32_t ospeed_reg;   ///< Output Speed register : These bits are written by software to configure the I/O output speed @note efer to the device datasheet for the frequency specifications and the power supply and load conditions for each speed
        volatile std::uint32_t pupd_reg;     ///< Pull up/down register : These bits are written by software to configure the I/O pull-up or pull-down
        volatile std::uint32_t id_reg;       ///< Input Data register : Reserved, must be kept at reset value
        volatile std::uint32_t od_reg;       ///< Output Data register : These bits can be read and written by software @note For atomic bit set/reset, the OD bits can be individually set and/or reset by writing to the GPIOx_BSRR register (x = A..D, F)
        volatile std::uint32_t bsr_reg;      ///< Bit Set/Reset register : These bits can be read/written by software @note For atomic bit set/reset, the od_reg bits can be individually set and/or reset by writing to the GPIOx_BSRR register (x = A..D, F)
        volatile std::uint32_t lck_reg;      ///< Lock register : These bits are read/write but can only be written when the LCKK bit is 0 @note A specific write sequence is used to write to the GPIOx_LCKR register. Only word access (32-bit long) is allowed during this locking sequence. Each lock bit freezes a specific configuration register (control and alternate function
        volatile std::uint32_t afrl_reg;     ///< Alternate Function Low register : These bits are written by software to configure alternate function I/Os
        volatile std::uint32_t afrh_reg;     ///< Alternate Function High register : These bits are written by software to configure alternate function I/Os
        volatile std::uint32_t br_reg;       ///< Bit Reset Register : These bits are write-only. A read to these bits returns the value 0x0000
    };
    static_assert(sizeof(gpio_registers) == 44, "Error on gpio registers struct size");

    gpio_registers * gpio_reg;
};

#pragma GCC diagnostic pop

#endif // DRIVER_HAL_GPIO_H