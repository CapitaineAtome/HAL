//
// Created by marmelade on 27/01/23.
//

#ifndef DRIVER_HAL_RCC_H
#define DRIVER_HAL_RCC_H

#include <cstdint>
#include <cassert>

#include "memorymap.h"
#include "common.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wvolatile"

#define RCC_PLL_READ        25
#define RCC_PLL_WRITE       24

#define RCC_HSI48_READ      23
#define RCC_HSI48_WRITE     22

#define RCC_CSS_READ        19
#define RCC_CSS_WRITE       19

#define RCC_HSEByp_READ     18
#define RCC_HSEByp_WRITE    18

#define RCC_HSE_READ        17
#define RCC_HSE_WRITE       16

#define RCC_HSI16DIV_READ   10
#define RCC_HSI16DIV_WRITE  11

#define RCC_HSI16KER_READ   9
#define RCC_HSI16KER_WRITE  9

#define RCC_HSI16_READ      8
#define RCC_HSI16_WRITE     8

namespace hal {
    class RCC {
    public:
        RCC() {

            rcc_reg = reinterpret_cast<volatile struct rcc_registers *>(RCC_MEMORY);
        }

        ~RCC() {

            rcc_reg = nullptr;
        }

        // Clock Control register
        enum class [[nodiscard]] ClockControl : uint8_t {
            PLL,
            HSI48,
            CSS,
            HSEByp,
            HSE,
            HSI16DIV,
            HSI16KER,
            HSI16
        };

        /**
         *
         * @param cc
         * @param enabled
         */
        void ClockControl(const enum ClockControl cc, const uint8_t enabled) {

            switch (cc) {
                case ClockControl::PLL:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_PLL_WRITE, enabled);
                    break;
                case ClockControl::HSI48:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_HSI48_WRITE, enabled);
                    break;
                case ClockControl::CSS:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_CSS_WRITE, enabled);
                    break;
                case ClockControl::HSEByp:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_HSEByp_WRITE, enabled);
                    break;
                case ClockControl::HSE:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_HSE_WRITE, enabled);
                    break;
                case ClockControl::HSI16DIV:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_HSI16DIV_WRITE, enabled);
                    break;
                case ClockControl::HSI16KER:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_HSI16KER_WRITE, enabled);
                    break;
                case ClockControl::HSI16:
                    hal::set_bit_value(rcc_reg->cc_reg, RCC_HSI16_WRITE, enabled);
                    break;
            }
        }

        /**
         *
         * @param cc
         * @return
         */
        bool ClockControl(const enum ClockControl cc) {

            switch (cc) {
                case ClockControl::PLL:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_PLL_READ);
                case ClockControl::HSI48:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_HSI48_READ);
                case ClockControl::CSS:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_CSS_READ);
                case ClockControl::HSEByp:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_HSEByp_READ);
                case ClockControl::HSE:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_HSE_READ);
                case ClockControl::HSI16DIV:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_HSI16DIV_READ);
                case ClockControl::HSI16KER:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_HSI16KER_READ);
                case ClockControl::HSI16:
                    return hal::check_bit(rcc_reg->cc_reg, RCC_HSI16_READ);
            }
        }

        // Internal Clock Source Calibration register
        enum class [[nodiscard]] InternalClockSourceCalibration : uint8_t {

            CALIBRATION,
            TRIM,
        };

        void ClockSourceCalibration(const enum InternalClockSourceCalibration csc, const uint8_t value) const {

            if(csc == InternalClockSourceCalibration::CALIBRATION) {

                set_bits_in_range(rcc_reg->icsc_reg, 8, value, 7);
            }
        }

        [[nodiscard]] uint8_t ClockSourceCalibration(const enum InternalClockSourceCalibration csc) const noexcept {

            switch (csc) {
                case InternalClockSourceCalibration::CALIBRATION:
                    return rcc_reg->icsc_reg & 0xFF;

                case InternalClockSourceCalibration::TRIM:
                    return rcc_reg->icsc_reg & (0x7F << 8);
            }
        }

        enum class [[nodiscard]] ClocksConfiguration : uint8_t {

            SW,         ///< System clock switch
            SWS,        ///< System clock switch status
            HPRE,       ///< AHB prescaler
            PPRE,       ///< APB prescaler
            MCO2SEL,    ///< Microcontroller clock output 2 clock selector
            MCO2PRE,    ///< Microcontroller clock output 2 prescaler
            MCOSEL,     ///< Microcontroller clock output clock selector
            MCOPRE,     ///< Microcontroller clock output prescaler
        };

        void ClockConfiguration(const enum ClocksConfiguration clock, const uint8_t value) noexcept {

            switch(clock) {
                case ClocksConfiguration::SW:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 0, value, 3);
                    break;

                case ClocksConfiguration::SWS:
                    assert("READ ONLY");
                    break;

                case ClocksConfiguration::HPRE:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 8, value, 4);
                    break;

                case ClocksConfiguration::PPRE:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 12, value, 3);
                    break;

                case ClocksConfiguration::MCO2SEL:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 16, value, 4);
                    break;

                case ClocksConfiguration::MCO2PRE:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 20, value, 4);
                    break;

                case ClocksConfiguration::MCOSEL:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 24, value, 4);
                    break;

                case ClocksConfiguration::MCOPRE:
                    hal::set_bits_in_range(rcc_reg->cfg_reg, 28, value, 4);
                    break;
            }
        }

        /**
         * \verbatim
         * SW :
         *      000 HSISYS
         *      001 HSE
         *      010 PLLRCLK
         *      011 LSI
         *      100 LSE
         *
         * SWS :
         *      000 HSISYS
         *      001 HSE
         *      010 PLLRCLK
         *      011 LSI
         *      100 LSE
         *
         * HPRE :
         *      0xxx 1
         *      1000 2
         *      1001 4
         *      1010 8
         *      1011 16
         *      1100 64
         *      1101 128
         *      1110 256
         *      1111 512
         *
         * PPRE:
         *      0xx 1
         *      100 2
         *      101 4
         *      110 8
         *      111 16
         *
         * MCO2SEL:
         *      0000 No clock, MCO output disabled
         *      0001 SYSCLK
         *      0010 HSI48
         *      0011 HSI16
         *      0100 HSE
         *      0101 PLLRCLK
         *      0110 LSI
         *      0111 LSE
         *      1000 PLLPCLK
         *      1001 PLLQCLK
         *      1010 RTCCLK
         *      1011 RCTWAKEUP
         *
         * MCO2PRE:
         *      0000 1
         *      0001 2
         *      0010 4
         *      .... .
         *      0111 128
         *      1000 256
         *      1001 512
         *      1010 1024
         *
         * MCOSEL:
         *      0000 No clock, MCO output disabled
         *      0001 SYSCLK
         *      0010 HSI48
         *      0011 HSI16
         *      0100 HSE
         *      0101 PLLRCLK
         *      0110 LSI
         *      0111 LSE
         *      1000 PLLPCLK
         *      1001 PLLQCLK
         *      1010 RTCCLK
         *      1011 RCTWAKEUP
         *
         * MCOPRE:
         *      0000 1
         *      0001 2
         *      0010 4
         *      .... .
         *      0111 128
         *      1000 256
         *      1001 512
         *      1010 1024
         * \endverbatim
         * @param clock
         * @return
         */
        [[nodiscard]] uint8_t ClockConfiguration(const enum ClocksConfiguration clock) {

            switch(clock) {
                case ClocksConfiguration::SW:
                    return (rcc_reg->cfg_reg >> 0) & 0b111;

                case ClocksConfiguration::SWS:
                    return (rcc_reg->cfg_reg >> 3) & 0b111;

                case ClocksConfiguration::HPRE:
                    return (rcc_reg->cfg_reg >> 8) & 0b1111;

                case ClocksConfiguration::PPRE:
                    return (rcc_reg->cfg_reg >> 12) & 0b111;

                case ClocksConfiguration::MCO2SEL:
                    return (rcc_reg->cfg_reg >> 16) & 0b1111;

                case ClocksConfiguration::MCO2PRE:
                    return (rcc_reg->cfg_reg >> 20) & 0b1111;

                case ClocksConfiguration::MCOSEL:
                    return (rcc_reg->cfg_reg >> 24) & 0b1111;

                case ClocksConfiguration::MCOPRE:
                    return (rcc_reg->cfg_reg >> 28) & 0b1111;
            }
        }

        enum class [[nodiscard]] PLLConfiguration : uint8_t {

            PLLSRC, ///< PLL input clock source
            PLLM,   ///< Division factor M of the PLL input clock divider @warning The software must set these bits so that the PLL input frequency after the /M divider is between 2.66 and 16 MHz
            PLLN,   ///< PLL frequency multiplication factor N @warning The software must set these bits so that the VCO output frequency is between 64 and 344 MHz.
            PLLPEN, ///< PLLPCLK clock output enable
            PLLP,   ///< PLL VCO division factor P for PLLPCLK clock output @warning The software must set this bitfield so as not to exceed 122 MHz on this clock
            PLLQEN, ///< PLLQCLK clock output enable
            PLLQ,   ///< PLL VCO division factor Q for PLLQCLK clock output @warning The software must set this bitfield so as not to exceed 128 MHz on this clock
            PLLREN, ///< PLLRCLK clock output enable
            PLLR,   ///< PLL VCO division factor R for PLLRCLK clock output @warning The software must set this bitfield so as not to exceed 64 MHz on this clock
        };

        /**
         *
         * \verbatim
         * f_vco  = f_pllin * (N / M)
         * f_pllp = f_vco / P
         * f_pllq = f_vco / Q
         * f_pllr = f_vco / R
         *
         * PLLSRC:
         *      00 No clock
         *      01 Reserved
         *      10 HSI16
         *      11 HSE
         *
         * PLLM:
         *      000 1
         *      001 2
         *      010 3
         *      011 4
         *      100 5
         *      101 6
         *      110 7
         *      111 8
         *
         * PLLN:
         *      0000000 Invalid
         *      0000001 Reserved
         *      ...
         *      0000111 Reserved
         *      0001000 8
         *      0001001 9
         *      ...
         *      1010101 85
         *      1010110 86
         *      1010111 Reserved
         *      ...
         *      1111111 Reserved
         *
         * PLLPEN:
         *      0 Disable
         *      1 Enable
         *
         * PLLP:
         *      00000 Reserved
         *      00001 2
         *      ...
         *      11111 32
         *
         * PLLQEN:
         *      0 Disable
         *      1 Enable
         *
         * PLLQ:
         *      000 Reserved
         *      001 2
         *      010 3
         *      011 4
         *      100 5
         *      101 6
         *      110 7
         *      111 8
         *
         * PLLREN:
         *      0 Disable
         *      1 Enable
         *
         * PLLR:
         *      000 Reserved
         *      001 2
         *      010 3
         *      011 4
         *      100 5
         *      101 6
         *      110 7
         *      111 8
         *      The itf
         * \endverbatim
         *
         * @param pll
         * @param value
         */
        void PLLConfiguration(const enum PLLConfiguration pll, const uint8_t value) {

            switch(pll) {
                case PLLConfiguration::PLLSRC:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 0, value, 2);
                    break;

                case PLLConfiguration::PLLM:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 4, value, 3);
                    break;

                case PLLConfiguration::PLLN:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 8, value, 8);
                    break;

                case PLLConfiguration::PLLPEN:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 16, value, 1);
                    break;

                case PLLConfiguration::PLLP:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 17, value, 5);
                    break;

                case PLLConfiguration::PLLQEN:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 24, value, 1);
                    break;

                case PLLConfiguration::PLLQ:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 25, value, 3);
                    break;

                case PLLConfiguration::PLLREN:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 28, value, 1);
                    break;

                case PLLConfiguration::PLLR:
                    hal::set_bits_in_range(rcc_reg->pllcfg_reg, 29, value, 3);
                    break;
            }
        }

        /**
         *
         * @param pll
         * @return
         */
        [[nodiscard]] uint8_t PLLConfiguration(const enum PLLConfiguration pll) {

            switch(pll) {
                case PLLConfiguration::PLLSRC:
                    return (rcc_reg->pllcfg_reg >> 0) & 0b11;

                case PLLConfiguration::PLLM:
                    return (rcc_reg->pllcfg_reg >> 4) & 0b111;

                case PLLConfiguration::PLLN:
                    return (rcc_reg->pllcfg_reg >> 8) & 0xFF;

                case PLLConfiguration::PLLPEN:
                    return (rcc_reg->pllcfg_reg >> 16) & 0b1;

                case PLLConfiguration::PLLP:
                    return (rcc_reg->pllcfg_reg >> 17) & 0x1F;

                case PLLConfiguration::PLLQEN:
                    return (rcc_reg->pllcfg_reg >> 24) & 0b1;

                case PLLConfiguration::PLLQ:
                    return (rcc_reg->pllcfg_reg >> 25) & 0b111;

                case PLLConfiguration::PLLREN:
                    return (rcc_reg->pllcfg_reg >> 28) & 0b1;

                case PLLConfiguration::PLLR:
                    return (rcc_reg->pllcfg_reg >> 29) & 0b111;
            }
        }

#if defined(STM32G0B1) || defined(STM32G0C1xx)
        /**
         * @return
         */
        uint16_t ClockRecovery() {
            return rcc_reg rcc_reg->crrc_reg & 0x1FF;

            return 0;
        }
#endif

        enum class [[nodiscard]] ClockInterrupt : uint8_t {
            LSI,        ///< LSI interrupt
            LSE,        ///< LSE interrupt
            HIS48,      ///< HSI48 interrupt
            HSI,        ///< HSI16 interrupt
            HSE,        ///< HSE interrupt
            PLL,        ///< PLL interrupt
            CSS = 8,    ///< HSE security system interrupt
            LSECSS,     ///< LSE security system interrupt
        };

        /**
         *
         * @param cie
         * @param enable
         */
        void ClockInterruptEnable(const enum ClockInterrupt cie, const bool enable) noexcept {

            assert(static_cast<uint8_t>(cie) <= static_cast<uint8_t>(ClockInterrupt::PLL));

            hal::set_bit_value(rcc_reg->cie_reg, static_cast<uint8_t>(cie), enable);
        }

        /**
         *
         * @param cie
         * @return
         */
        [[nodiscard]] bool ClockInterruptEnable(const enum ClockInterrupt cie) const noexcept {

            assert(static_cast<uint8_t>(cie) <= static_cast<uint8_t>(ClockInterrupt::PLL));

            return hal::check_bit(rcc_reg->cie_reg, static_cast<uint8_t>(cie));
        }

        /**
         *
         * @param cif
         * @return
         */
        [[nodiscard]] bool ClockInterruptFlag(const enum ClockInterrupt cif) const noexcept {

            return hal::check_bit(rcc_reg->cif_reg, static_cast<uint8_t>(cif));
        }

        /**
         *
         * @param cic
         * @param clear
         */
        void ClockInterruptClear(const enum ClockInterrupt cic, bool clear) noexcept {

            hal::set_bit_value(rcc_reg->cic_reg, static_cast<uint8_t>(cic), clear);
        }

        /**
         *
         * @param port
         * @param reset
         */
        void IOPortReset(const enum Port port, const bool reset) noexcept {

            hal::set_bit_value(rcc_reg->ioprst_reg, static_cast<uint8_t>(port), reset);
        }

        /**
         *
         * @param port
         * @return
         */
        [[nodiscard]] bool IOPortReset(const enum Port port) const noexcept {

            return hal::check_bit(rcc_reg->ioprst_reg, static_cast<uint8_t>(port));
        }

        enum class [[nodiscard]] AHBPeripheral : uint8_t {
            DMA1 = 0,   ///< Direct Memory Access 1
            DMA2 = 1,   ///< Direct Memory Access 2
            FLASH = 8,  ///< Flash memory interface
            CRC = 12,   ///< CRC
            AES = 16,   ///< AES hardware accelerator
            RNG = 18,   ///< Random Number Generator
        };

        /**
         *
         * @param periph
         * @param reset
         */
        void AHBPeripheralReset(const enum AHBPeripheral periph, const bool reset) noexcept {

            hal::set_bit_value(rcc_reg->ahbrst_reg, static_cast<uint8_t>(periph), reset);
        }

        /**
         *
         * @param periph
         * @param reset
         * @return
         */
        [[nodiscard]] bool AHBPeripheralReset(const enum AHBPeripheral periph) const noexcept {

            return hal::check_bit(rcc_reg->ahbrst_reg, static_cast<uint8_t>(periph));
        }

        /**
         *
         * @param periph
         * @param enable
         */
        void AHBPeripheralClockEnable(const enum AHBPeripheral periph, const bool enable) noexcept {

            hal::set_bit_value(rcc_reg->ahben_reg, static_cast<uint8_t>(periph), enable);
        }

        /**
         *
         * @param periph
         * @return
         */
        [[nodiscard]] bool AHBPeripheralClockEnable(const enum AHBPeripheral periph) const noexcept {

            return hal::check_bit(rcc_reg->ahben_reg, static_cast<uint8_t>(periph));
        }

        /**
         *
         * @param periph
         * @param enable
         */
        void AHBPeripheralSleepClockEnable(const enum AHBPeripheral periph, const bool enable) noexcept {

            hal::set_bit_value(rcc_reg->ahbsmen_reg, static_cast<uint8_t>(periph), enable);
        }

        /**
         *
         * @param periph
         * @return
         */
        [[nodiscard]] bool AHBPeripheralSleepClockEnable(const enum AHBPeripheral periph) const noexcept {

            return hal::check_bit(rcc_reg->ahbsmen_reg, static_cast<uint8_t>(periph));
        }

        enum class [[nodiscard]] APBPeripheral : uint8_t {
            TIM2        = 0,                ///< TIM2
            TIM3        = 1,                ///< TIM3
            TIM4        = 2,                ///< TIM4
            TIM6        = 4,                ///< TIM6
            TIM7        = 5,                ///< TIM7
            LPUART2     = 7,                ///< LPUART2
            USART5      = 8,                ///< USART5
            USART6      = 9,                ///< USART6
            FDCAN       = 12,               ///< FDCAN
            USB         = 13,               ///< USB
            SPI2        = 14,               ///< SPI2
            SPI3        = 15,               ///< SPI3
            CRS         = 16,               ///< CRS
            USART2      = 17,               ///< USART2
            USART3      = 18,               ///< USART3
            USART4      = 19,               ///< USART4
            LPUSART1    = 20,               ///< LPUSART1
            I2C1        = 21,               ///< I2C1
            I2C2        = 22,               ///< I2C2
            I2C3        = 23,               ///< I2C3
            CEC         = 24,               ///< CEC
            UCPD1       = 25,               ///< UCPD1
            UCPD2       = 26,               ///< UCPD2
            DBG         = 27,               ///< Debug support
            PWR         = 28,               ///< Power interface
            DAC1        = 29,               ///< DAC1 interface
            LPTIM2      = 30,               ///< Low Power Timer 2
            LPTIM1      = 31,               ///< Low Power Timer 1
            SYSCFG      = 1  | (1 << 7),    ///< SYSCFG, COMP adn VREFBUF
            TIM1        = 11 | (1 << 7),    ///< TIM1
            SPI1        = 12 | (1 << 7),    ///< SPI1
            USART1      = 14 | (1 << 7),    ///< USART1
            TIM14       = 15 | (1 << 7),    ///< TIM14
            TIM15       = 16 | (1 << 7),    ///< TIM15
            TIM16       = 17 | (1 << 7),    ///< TIM16
            TIM17       = 18 | (1 << 7),    ///< TIM17
            ADC         = 20 | (1 << 7),    ///< ADC
        };

        /**
         *
         * @param periph
         * @param reset
         */
        void APBPeripheralReset(const enum APBPeripheral periph, const bool reset) noexcept {

            const auto periph_ = static_cast<uint8_t>(periph);
            const auto idx_ = hal::check_bit(periph_, 7); // if MSB is set, idx = 1

            hal::set_bit_value(rcc_reg->apbrst_reg[idx_], periph_ & 0x7F, reset);
        }

        /**
         *
         * @param periph
         * @return
         */
        [[nodiscard]] bool APBPeripheralReset(const enum APBPeripheral periph) const noexcept {

            const auto periph_ = static_cast<uint8_t>(periph);
            const auto idx_ = hal::check_bit(periph_, 7); // if MSB is set, idx = 1

            return hal::check_bit(rcc_reg->apbrst_reg[idx_], periph_ & 0x7F);
        }

        /**
         *
         * @param periph
         * @param reset
         */
        void APBPeripheralClockEnable(const enum APBPeripheral periph, const bool reset) noexcept {

            const auto periph_ = static_cast<uint8_t>(periph);
            const auto idx_ = hal::check_bit(periph_, 7); // if MSB is set, idx = 1

            hal::set_bit_value(rcc_reg->apben_reg[idx_], periph_ & 0x7F, reset);
        }

        /**
         *
         * @param periph
         * @return
         */
        [[nodiscard]] bool APBPeripheralClockEnable(const enum APBPeripheral periph) const noexcept {

            const auto periph_ = static_cast<uint8_t>(periph);
            const auto idx_ = hal::check_bit(periph_, 7); // if MSB is set, idx = 1

            return hal::check_bit(rcc_reg->apben_reg[idx_], periph_ & 0x7F);
        }

        /**
         *
         * @param periph
         * @param reset
         */
        void APBPeripheralSleepClockEnable(const enum APBPeripheral periph, const bool reset) noexcept {

            const auto periph_ = static_cast<uint8_t>(periph);
            const auto idx_ = hal::check_bit(periph_, 7); // if MSB is set, idx = 1

            hal::set_bit_value(rcc_reg->apbsmen_reg[idx_], periph_ & 0x7F, reset);
        }

        /**
         *
         * @param periph
         * @return
         */
        [[nodiscard]] bool APBPeripheralSleepClockEnable(const enum APBPeripheral periph) const noexcept {

            const auto periph_ = static_cast<uint8_t>(periph);
            const auto idx_ = hal::check_bit(periph_, 7); // if MSB is set, idx = 1

            return hal::check_bit(rcc_reg->apbsmen_reg[idx_], periph_ & 0x7F);
        }

        /**
         *
         * @param port
         * @param enable
         */
        void IOPortClockEnable(const enum Port port, const bool enable) {

            hal::set_bit_value(rcc_reg->iopen_reg, static_cast<uint8_t>(port), enable);

            // Dummy read to apply a delay after clock enabling
            rcc_reg->iopen_reg = 4;
            volatile bool tmp;
            for(std::size_t i{}; i < 2; i++) {
                tmp = IOPortClockEnable(port);
                unused(tmp);
            }
        }

        /**
         *
         * @param port
         * @return
         */
        [[nodiscard]] bool IOPortClockEnable(const enum Port port) {

            return hal::check_bit(rcc_reg->iopen_reg, static_cast<uint8_t>(port));
        }

        /**
         *
         * @param port
         * @param enable
         */
        void IOPortSleepModeClockEnable(const enum Port port, const bool enable) {

            hal::set_bit_value(rcc_reg->iospmen_reg, static_cast<uint8_t>(port), enable);
        }

        /**
         *
         * @param port
         * @return
         */
        [[nodiscard]] bool IOPortSleepModeClockEnable(const enum Port port) {

            return hal::check_bit(rcc_reg->iospmen_reg, static_cast<uint8_t>(port));
        }

        enum class [[nodiscard]] IndependentPeripheral : uint8_t {

            USART1      = 0,                ///< USART1
            USART2      = 2,                ///< USART2
            USART3      = 4,                ///< USART3
            CEC         = 6,                ///< CEC
            LPUART2     = 8,                ///< LPUART2
            LPUART1     = 10,               ///< LPUART1
            I2C1        = 12,               ///< I2C1
            I2C2I2S1    = 14,               ///< I2C2/I2S1
            LPTIM1      = 18,               ///< LPTIM1
            LPTIM2      = 20,               ///< LPTIM2
            TIM1        = 22,               ///< TIM1
            TIM15       = 24,               ///< TIM15
            RNG         = 26,               ///< RNG
            RNGDIV      = 28,               ///< RNGDIV
            ADC         = 30,               ///< ADC
            I2S1        = 0  | (1 << 7),    ///< I2S1
            I2S2        = 2  | (1 << 7),    ///< I2S2
            FDCAN       = 8  | (1 << 7),    ///< FDCAN
            USB         = 12 | (1 << 7),    ///< USB
        };

#define IndependentPeripheral_USART1_RANGE   2
#define IndependentPeripheral_USART2_RANGE   2
#define IndependentPeripheral_USART3_RANGE   2
#define IndependentPeripheral_CEC_RANGE      1
#define IndependentPeripheral_LPUART2_RANGE  2
#define IndependentPeripheral_LPUART1_RANGE  2
#define IndependentPeripheral_I2C1_RANGE     2
#define IndependentPeripheral_I2C2I2S1_RANGE 2
#define IndependentPeripheral_LPTIM1_RANGE   2
#define IndependentPeripheral_LPTIM2_RANGE   2
#define IndependentPeripheral_TIM1_RANGE     1
#define IndependentPeripheral_TIM15_RANGE    1
#define IndependentPeripheral_RNG_RANGE      2
#define IndependentPeripheral_RNGDIV_RANGE   2
#define IndependentPeripheral_ADC_RANGE      2
#define IndependentPeripheral_I2S1_RANGE     2
#define IndependentPeripheral_I2S2_RANGE     2
#define IndependentPeripheral_FDCAN_RANGE    2
#define IndependentPeripheral_USB_RANGE      2

        /**
         * \verbatim
         * USART1:
         *      00 PCLK
         *      01 SYSCLK
         *      10 HSI16
         *      11 LSE
         *
         *
         * USART2:
         *      00 PCLK
         *      01 SYSCLK
         *      10 HSI16
         *      11 LSE
         *
         *
         * USART3:
         *      00 PCLK
         *      01 SYSCLK
         *      10 HSI16
         *      11 LSE
         *
         * CEC:
         *      0 HSI16 divided by 488
         *      1 LSE
         *
         * LPUART2:
         *      00 PCLK
         *      01 SYSCLK
         *      10 HSI16
         *      11 LSE
         *
         * LPUART1:
         *      00 PCLK
         *      01 SYSCLK
         *      10 HSI16
         *      11 LSE
         *
         * I2C1:
         *      00 PCLK
         *      01 SYSCLK
         *      10 HSI16
         *      11 Reserved
         *
         * I2C2I2S1:
         *      00 PCLK/SYSCLK
         *      01 SYSCLK/PLLPCLK
         *      10 HSI16/HSI16
         *      11 Reserved/I2S_CKIN
         *
         * LPTIM1:
         *      00 PCLK
         *      01 LSI
         *      10 HSI16
         *      11 LSE
         *
         * LPTIM2:
         *      00 PCLK
         *      01 LSI
         *      10 HSI16
         *      11 LSE
         *
         * TIM1:
         *      0 TIMPCLK
         *      1 PLLQCLK
         *
         * TIM15:
         *      0 TIMPCLK
         *      1 PLLQCLK
         *
         * RNG:
         *      00 No clock
         *      01 HSI16
         *      10 SYSCLK
         *      11 PLLQCLK
         *
         * RNGDIV:
         *      00 1
         *      01 2
         *      10 4
         *      11 8
         *
         * ADC:
         *      00 System clock
         *      01 PLLPCLK
         *      10 HSI16
         *      11 Reserved
         *
         * I2S1:
         *      00 SYSCLK
         *      01 PLLPCLK
         *      10 HSI16
         *      11 External I2S clock selected as I2S1
         *
         *
         * I2S2:
         *      00 SYSCLK
         *      01 PLLPCLK
         *      10 HSI16
         *      11 External I2S clock selected as I2S2
         *
         *
         * FDCAN:
         *      00 PCLK
         *      01 PLLQCLK
         *      10 HSE
         *      11 Reserved
         *
         *
         * USB:
         *      00 HSI48
         *      01 PLLQCLK
         *      10 HSE
         *      11 Reserved
         * \endverbatim
         *
         * @param periph
         * @param value
         */
        void PeripheralIndependentClockConfiguration(const enum IndependentPeripheral periph, const uint8_t value) noexcept {

            const auto pos_ = static_cast<uint8_t>(periph);
            const auto idx_ = static_cast<uint8_t>(hal::check_bit(static_cast<uint8_t>(periph), 7));

            switch(periph) {
                case IndependentPeripheral::USART1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_USART1_RANGE);
                    break;
                case IndependentPeripheral::USART2:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_USART2_RANGE);
                    break;
                case IndependentPeripheral::USART3:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_USART3_RANGE);
                    break;
                case IndependentPeripheral::CEC:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_CEC_RANGE);
                    break;
                case IndependentPeripheral::LPUART2:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_LPUART2_RANGE);
                    break;
                case IndependentPeripheral::LPUART1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_LPUART1_RANGE);
                    break;
                case IndependentPeripheral::I2C1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_I2C1_RANGE);
                    break;
                case IndependentPeripheral::I2C2I2S1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_I2C2I2S1_RANGE);
                    break;
                case IndependentPeripheral::LPTIM1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_LPTIM1_RANGE);
                    break;
                case IndependentPeripheral::LPTIM2:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_LPTIM2_RANGE);
                    break;
                case IndependentPeripheral::TIM1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_TIM1_RANGE);
                    break;
                case IndependentPeripheral::TIM15:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_TIM15_RANGE);
                    break;
                case IndependentPeripheral::RNG:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_RNG_RANGE);
                    break;
                case IndependentPeripheral::RNGDIV:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_RNGDIV_RANGE);
                    break;
                case IndependentPeripheral::ADC:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_ADC_RANGE);
                    break;
                case IndependentPeripheral::I2S1:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_I2S1_RANGE);
                    break;
                case IndependentPeripheral::I2S2:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_I2S2_RANGE);
                    break;
                case IndependentPeripheral::FDCAN:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_FDCAN_RANGE);
                    break;
                case IndependentPeripheral::USB:
                    hal::set_bits_in_range(rcc_reg->ccip_reg[idx_], pos_, value, IndependentPeripheral_USB_RANGE);
                    break;
            }
        }

        /**
         *
         * @param periph
         * @return
         */
        [[nodiscard]] uint8_t PeripheralIndependentClockConfiguration(const enum IndependentPeripheral periph) {

            const auto pos_ = static_cast<uint8_t>(periph);
            const auto idx_ = static_cast<uint8_t>(hal::check_bit(static_cast<uint8_t>(periph), 7));

            switch(periph) {
                case IndependentPeripheral::USART1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_USART1_RANGE>();

                case IndependentPeripheral::USART2:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_USART2_RANGE>();

                case IndependentPeripheral::USART3:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_USART3_RANGE>();

                case IndependentPeripheral::CEC:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_CEC_RANGE>();

                case IndependentPeripheral::LPUART2:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_LPUART2_RANGE>();

                case IndependentPeripheral::LPUART1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_LPUART1_RANGE>();

                case IndependentPeripheral::I2C1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_I2C1_RANGE>();

                case IndependentPeripheral::I2C2I2S1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_I2C2I2S1_RANGE>();

                case IndependentPeripheral::LPTIM1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_LPTIM1_RANGE>();

                case IndependentPeripheral::LPTIM2:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_LPTIM2_RANGE>();

                case IndependentPeripheral::TIM1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_TIM1_RANGE>();

                case IndependentPeripheral::TIM15:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_TIM15_RANGE>();

                case IndependentPeripheral::RNG:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_RNG_RANGE>();

                case IndependentPeripheral::RNGDIV:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_RNGDIV_RANGE>();

                case IndependentPeripheral::ADC:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_ADC_RANGE>();

                case IndependentPeripheral::I2S1:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_I2S1_RANGE>();

                case IndependentPeripheral::I2S2:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_I2S2_RANGE>();

                case IndependentPeripheral::FDCAN:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_FDCAN_RANGE>();

                case IndependentPeripheral::USB:
                    return hal::check_bit(rcc_reg->ccip_reg[idx_], pos_) & hal::set_first_bits<IndependentPeripheral_USB_RANGE>();
            }
        }


    protected:

    private:

        struct rcc_registers {
            std::uint32_t cc_reg;          ///< Clock Control register
            std::uint32_t icsc_reg;        ///< Internal Clock Source Calibration register
            std::uint32_t cfg_reg;         ///< Clock Configuration register
            std::uint32_t pllcfg_reg;      ///< PLL Configuration register
            std::uint32_t crrc_reg;        ///< RCC clock recovery RC register
            std::uint32_t cie_reg;         ///< Clock interrupt enable register
            std::uint32_t cif_reg;         ///< Clock interrupt flag register
            std::uint32_t cic_reg;         ///< Clock interrupt clear register
            std::uint32_t ioprst_reg;      ///< I/O port reset register
            std::uint32_t ahbrst_reg;      ///< AHB peripheral reset register
            std::uint32_t apbrst_reg[2];   ///< APB peripheral reset registers
            std::uint32_t iopen_reg;       ///< I/O port clock enable register
            std::uint32_t ahben_reg;       ///< AHB peripheral clock enable register
            std::uint32_t apben_reg[2];    ///< APB peripheral clock enable registers
            std::uint32_t iospmen_reg;     ///< I/O port in sleep mode clock enable register
            std::uint32_t ahbsmen_reg;     ///< AHB peripheral clock enable in sleep/stop mode register
            std::uint32_t apbsmen_reg[2];  ///< APB peripheral clock enable in sleep/stop mode register
            std::uint32_t ccip_reg[2];     ///< Peripheral independent clock configuration register
            std::uint32_t bdc_reg;         ///< RTC domain control register
            std::uint32_t cs_reg;          ///< Control/Status register
        };
        static_assert(sizeof(rcc_registers) == 96, "Error rcc registers struct size is not valid");

        volatile rcc_registers *rcc_reg;
    };
}

#pragma GCC diagnostic pop

#endif // DRIVER_HAL_RCC_H
