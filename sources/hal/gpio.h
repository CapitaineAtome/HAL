//
// Created by marmelade on 25/01/23.
//

// TODO: Commenter tout ça

#ifndef DRIVER_HAL_GPIO_H
#define DRIVER_HAL_GPIO_H

#include "common.h"
#include "memorymap.h"

#include "rcc.h"

#include <cstdint>
#include <utility>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

namespace hal {

    class GPIOPort {
    public:

        enum class [[nodiscard]] BitReset : uint8_t {
            NO_ACTION,  ///< No action on the corresponding ODx bit
            RESET,      ///< Reset the corresponding ODx bit
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

        /**
         *
         * @param port
         */
        explicit GPIOPort(const Port port) {

            // Enable the clock for the GPIOPort of the port selected
            RCC rcc{};

            rcc.IOPortClockEnable(port, true);

            switch(port) {
                case Port::A:
                    gpio_reg = reinterpret_cast<volatile gpio_registers *>(PORTA_MEMORY);
                    break;
                case Port::B:
                    gpio_reg = reinterpret_cast<volatile gpio_registers *>(PORTB_MEMORY);
                    break;
                case Port::C:
                    gpio_reg = reinterpret_cast<volatile gpio_registers *>(PORTC_MEMORY);
                    break;
                case Port::D:
                    gpio_reg = reinterpret_cast<volatile gpio_registers *>(PORTD_MEMORY);
                    break;
                case Port::E:
                    gpio_reg = reinterpret_cast<volatile gpio_registers *>(PORTE_MEMORY);
                    break;
                case Port::F:
                    gpio_reg = reinterpret_cast<volatile gpio_registers *>(PORTF_MEMORY);
                    break;
                default:
                    break;
            }
        }

        /**
         *
         */
        ~GPIOPort() {

            gpio_reg = nullptr;
        }

        GPIOPort(const GPIOPort &other) : GPIOPort(Port::A) {

            gpio_reg = other.gpio_reg;
        }

        GPIOPort(GPIOPort &&other) noexcept : GPIOPort(Port::A) {

            gpio_reg = nullptr;

            swap(*this, other);
        }

        GPIOPort &operator=(GPIOPort other) {

            swap(*this, other);

            return *this;
        }

        GPIOPort &operator=(GPIOPort &&other) noexcept {

            swap(*this, other);

            return *this;
        }

        friend void swap(GPIOPort &first, GPIOPort &second) {

            using std::swap;

            swap(first.gpio_reg, second.gpio_reg);
        }

        enum class [[nodiscard]] Mode : uint8_t {
            INPUT = 0b00,       ///< Input mode
            OUTPUT = 0b01,      ///< General purpose output mode
            ALTERNATE = 0b10,   ///< Alternate function mode
            ANALOG = 0b11,      ///< Analog mode ( reset state )
        };

        /**
         *
         * @param pin
         * @param mode
         */
        void Mode(const enum Pin pin, const enum Mode mode) const noexcept {

            /*const uint32_t pin_pos = 1u << (static_cast<uint8_t>(pin) * 2u);

            hal::set_bit_value( gpio_reg->mode_reg, pin_pos,
                           static_cast<uint8_t>(mode) & 0b01 );

            hal::set_bit_value( gpio_reg->mode_reg, pin_pos + 1u,
                           static_cast<uint8_t>(mode) & 0b10 );*/

            hal::set_bits_in_range(gpio_reg->mode_reg, static_cast<uint8_t>(pin) * 2, static_cast<uint8_t>(mode), 2);
        }

        /**
         *
         * @param pin
         * @return
         */
        enum Mode Mode(const Pin pin) const noexcept {

            return static_cast<enum Mode>(  hal::get_masked_bytes_at(gpio_reg->mode_reg, static_cast<uint8_t>(pin) * 2,
                                                                     0b11));
        }

        enum class [[nodiscard]] OutputType : uint8_t {
            PUSH_PULL,  ///< Output push-pull ( reset state )
            OPEN_DRAIN, ///< Output open-drain
        };

        /**
         *
         * @param pin
         * @param otype
         */
        void OutputType(const Pin pin, const OutputType otype) noexcept {

            hal::set_bit_value(gpio_reg->otype_reg, static_cast<uint8_t>(pin), static_cast<uint8_t>(otype));
        }

        /**
         *
         * @param pin
         * @return
         */
        enum OutputType OutputType(const Pin pin) const noexcept {

            return static_cast<enum OutputType>( gpio_reg->otype_reg >> static_cast<uint8_t>(pin));
        }

        enum class [[nodiscard]] OutputSpeed : uint8_t {
            VERY_LOW_SPEED = 0b00,  ///< Very low speed
            LOW_SPEED = 0b01,       ///< Low speed
            HIGH_SPEED = 0b10,      ///< High speed
            VERY_HIGH_SPEED = 0b11, ///< Very high speed
        };

        /**
         *
         * @param pin
         * @param ospeed
         */
        void OutputSpeed(const Pin pin, const enum OutputSpeed ospeed) noexcept {

            hal::set_bits_in_range(gpio_reg->ospeed_reg, static_cast<uint8_t>(pin) * 2, static_cast<uint8_t>(ospeed),
                                   2);
        }

        /**
         *
         * @param pin
         * @return
         */
        enum OutputSpeed OutputSpeed(const Pin pin) const noexcept {

            return static_cast<enum OutputSpeed>( hal::get_masked_bytes_at(gpio_reg->ospeed_reg,
                                                                           static_cast<uint8_t>(pin) * 2, 0b11));
        }

        enum class [[nodiscard]] PullMode : uint8_t {
            NO_PULL = 0,        ///< No pull-up or pull-down
            PULL_UP = 0b01,     ///< Pull-up
            PULL_DOWN = 0b10,   ///< Pull-down
        };

        /**
         *
         * @param pin
         * @param pullm
         */
        void PullMode(const Pin pin, const enum PullMode pullm) noexcept {

            hal::set_bits_in_range(gpio_reg->pupd_reg, static_cast<uint8_t>(pin) * 2, static_cast<uint8_t>(pullm), 2);
        }

        /**
         *
         * @param pin
         * @return
         */
        enum PullMode PullMode(const Pin pin) const noexcept {

            return static_cast<enum PullMode>( hal::get_masked_bytes_at(gpio_reg->pupd_reg,
                                                                        static_cast<uint8_t>(pin) * 2, 0b11));
        }

        /**
         *
         * @param pin
         * @return
         */
        [[nodiscard]] bool Read(const Pin pin) const noexcept {

            return (gpio_reg->id_reg >> static_cast<uint8_t>(pin)) & 1u;
        }

        void Write(const Pin pin, bool value) noexcept {

            uint32_t pin_bit_pos = 1u << static_cast<uint8_t>(pin);

            gpio_reg->bsr_reg = static_cast<std::uint32_t>( pin_bit_pos << (16 * static_cast<uint8_t>(!value)));
        }

        /**
         *
         * @param pin
         */
        void Toggle(const Pin pin) noexcept {

            // It just works !

            Write(pin, !Read(pin));
        }

        enum class [[nodiscard]] Lock : uint8_t {
            NOT_LOCKED, ///< Port configuration not locked
            LOCKED,     ///< Port configuration locked
        };

        /**
         *
         * @param pin
         */
        void LockKey(const Pin pin) noexcept {

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

            unused(pin);
        }

        /**
         *
         * @param pin
         * @param mode
         * @param pull
         * @param type
         * @param speed
         * @return
         */
        bool InitPin( const enum Pin pin, const enum Mode mode, const enum PullMode pull, const enum OutputType type, const enum OutputSpeed speed) noexcept {

            OutputSpeed(pin, speed);
            OutputType(pin, type);
            PullMode(pin, pull);
            Mode(pin, mode);

            return false;
        }

        uint32_t InitPins(std::initializer_list<const enum Pin> pins, const enum Mode mode, const enum PullMode pull, const enum OutputType type, const enum OutputSpeed speed) noexcept {

            uint32_t ret{};
            uint8_t i{};

            for(const auto &pin : pins) {

                ret |= static_cast<uint32_t>((InitPin(pin, mode, pull, type, speed) ? 1 : 0) << i);
            }

            return ret;
        }

        /**
         *
         * @param pin
         * @param speed
         * @return
         */
        bool InitOutput(const enum Pin pin, const enum OutputSpeed speed = OutputSpeed::VERY_LOW_SPEED) noexcept {

            // Configure Output mode
            OutputSpeed(pin, speed);
            OutputType(pin, OutputType::PUSH_PULL);
            PullMode(pin, PullMode::NO_PULL);
            Mode(pin, Mode::OUTPUT);

            return false;
        }

        /**
         *
         * @param pins
         * @param speed
         * @return
         */
        bool InitOutputs(std::initializer_list<const enum Pin> pins, const enum OutputSpeed speed = OutputSpeed::VERY_LOW_SPEED) noexcept {

            uint32_t ret{};
            uint8_t i{};

            for(const auto &pin : pins) {

                ret |= static_cast<uint32_t>((InitOutput(pin, speed) ? 1 : 0) << i);
            }

            return ret;
        }

        /**
         *
         * @param pin
         * @return
         */
        bool InitInput(const enum Pin pin) noexcept {

            PullMode(pin, PullMode::NO_PULL);
            Mode(pin, Mode::INPUT);

            return false;
        }

        /**
         *
         * @param pins
         * @return
         */
        bool InitInputs(std::initializer_list<const enum Pin> pins) noexcept {

            uint32_t ret{};
            uint8_t i{};

            for(const auto &pin : pins) {

                ret |= static_cast<uint32_t>((InitInput(pin) ? 1 : 0) << i);
            }

            return ret;
        }

        /**
         *
         * @param pin
         * @return
         */
        bool InitAnalog(const enum Pin pin) noexcept {

            PullMode(pin, PullMode::NO_PULL);
            Mode(pin, Mode::ANALOG);

            return false;
        }

        /**
         *
         * @param pins
         * @return
         */
        bool InitAnalogs(std::initializer_list<const enum Pin> pins) noexcept {

            uint32_t ret{};
            uint8_t i{};

            for(const auto &pin : pins) {

                ret |= static_cast<uint32_t>((InitAnalog(pin) ? 1 : 0) << i);
            }

            return ret;
        }

    protected:

    private:

        // TODO: Set read only registers to const ?
        struct gpio_registers {
            std::uint32_t mode_reg;     ///< Mode register : These bits are written by software to configure the I/O mode
            std::uint32_t otype_reg;    ///< Output Type register : These bits are written by software to configure the I/O output type
            std::uint32_t ospeed_reg;   ///< Output Speed register : These bits are written by software to configure the I/O output speed @note Refer to the device datasheet for the frequency specifications and the power supply and load conditions for each speed
            std::uint32_t pupd_reg;     ///< Pull up/down register : These bits are written by software to configure the I/O pull-up or pull-down
            std::uint32_t id_reg; ///< Input Data register : Reserved, must be kept at reset value
            std::uint32_t od_reg;       ///< Output Data register : These bits can be read and written by software @note For atomic bit set/reset, the OD bits can be individually set and/or reset by writing to the GPIOx_BSRR register (x = A..D, F)
            std::uint32_t bsr_reg;      ///< Bit Set/Reset register : These bits can be read/written by software @note For atomic bit set/reset, the od_reg bits can be individually set and/or reset by writing to the GPIOx_BSRR register (x = A..D, F)
            std::uint32_t lck_reg;      ///< Lock register : These bits are read/write but can only be written when the LCKK bit is 0 @note A specific write sequence is used to write to the GPIOx_LCKR register. Only word access (32-bit long) is allowed during this locking sequence. Each lock bit freezes a specific configuration register (control and alternate function
            std::uint32_t afrl_reg;     ///< Alternate Function Low register : These bits are written by software to configure alternate function I/Os
            std::uint32_t afrh_reg;     ///< Alternate Function High register : These bits are written by software to configure alternate function I/Os
            std::uint32_t br_reg;       ///< Bit Reset Register : These bits are write-only. A read to these bits returns the value 0x0000
        };
        static_assert(sizeof(gpio_registers) == 44, "Error gpio registers struct size is not valid");

        /**
         *
         */
        volatile gpio_registers *gpio_reg;
    };
}

#pragma GCC diagnostic pop

#endif // DRIVER_HAL_GPIO_H
