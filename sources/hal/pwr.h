//
// Created by marmelade on 20/02/23.
//

#ifndef DRIVER_HAL_PWR_H
#define DRIVER_HAL_PWR_H

#include <cstdint>

#include "common.h"
#include "memorymap.h"

namespace hal {

    class PWR {
    public:
        PWR() {

            pwr_reg = reinterpret_cast<volatile pwr_registers *>(PWR_MEMORY);
        }

        ~PWR() {

            pwr_reg = nullptr;
        }

        enum class [[nodiscard]] StatusClearFlags {

            WakeUp1, ///< Clear Wake Up flag 1
            WakeUp2, ///< Clear Wake Up flag 2
            WakeUp3, ///< Clear Wake Up flag 3
            WakeUp4, ///< Clear Wake Up flag 4
            WakeUp5, ///< Clear Wake Up flag 5
            WakeUp6, ///< Clear Wake Up flag 6
            StandBy = 8, ///< Clear standby flag
        };

        constexpr void StatusClear(const enum StatusClearFlags flag, const bool clear) {

            set_bit_value(pwr_reg->scr, static_cast<uint8_t>(flag), clear);
        }

        constexpr void PullUpControl(const hal::Port port, const hal::Pin pin, const bool pull_up) {

            auto pin_nbr{static_cast<uint8_t>(pin)};

            switch(port) {
                case Port::A:
                    set_bit_value(pwr_reg->pucra, pin_nbr, pull_up);
                    break;
                case Port::B:
                    set_bit_value(pwr_reg->pucrb, pin_nbr, pull_up);
                    break;
                case Port::C:
                    set_bit_value(pwr_reg->pucrc, pin_nbr, pull_up);
                    break;
                case Port::D:
                    set_bit_value(pwr_reg->pucrd, pin_nbr, pull_up);
                    break;
                case Port::E:
                    if(pin_nbr < 13) {

                        set_bit_value(pwr_reg->pucre, pin_nbr, pull_up);
                    }
                    break;
                case Port::F:
                    if(pin_nbr < 13) {

                        set_bit_value(pwr_reg->pucrf, pin_nbr, pull_up);
                    }
                    break;
            }
        }

        bool PullUpControl(const hal::Port port, const hal::Pin pin) {

            auto pin_nbr{static_cast<uint8_t>(pin)};

            switch(port) {
                case Port::A:
                    return check_bit(pwr_reg->pucra, pin_nbr);
                    break;
                case Port::B:
                    return check_bit(pwr_reg->pucrb, pin_nbr);
                    break;
                case Port::C:
                    return check_bit(pwr_reg->pucrc, pin_nbr);
                    break;
                case Port::D:
                    return check_bit(pwr_reg->pucrd, pin_nbr);
                    break;
                case Port::E:
                    if(pin_nbr < 13) {

                        return check_bit(pwr_reg->pucre, pin_nbr);
                    }
                    break;
                case Port::F:
                    if(pin_nbr < 13) {

                        return check_bit(pwr_reg->pucrf, pin_nbr);
                    }
                    break;
            }

            return false;
        }

        constexpr void PullDownControl(const hal::Port port, const hal::Pin pin, const bool pull_down) {

            auto pin_nbr{static_cast<uint8_t>(pin)};

            switch(port) {
                case Port::A:
                    set_bit_value(pwr_reg->pdcra, pin_nbr, pull_down);
                    break;
                case Port::B:
                    set_bit_value(pwr_reg->pdcrb, pin_nbr, pull_down);
                    break;
                case Port::C:
                    set_bit_value(pwr_reg->pdcrc, pin_nbr, pull_down);
                    break;
                case Port::D:
                    set_bit_value(pwr_reg->pdcrd, pin_nbr, pull_down);
                    break;
                case Port::E:
                    if(pin_nbr < 13) {

                        set_bit_value(pwr_reg->pdcre, pin_nbr, pull_down);
                    }
                    break;
                case Port::F:
                    if(pin_nbr < 13) {

                        set_bit_value(pwr_reg->pdcrf, pin_nbr, pull_down);
                    }
                    break;
            }
        }

        bool PullDownControl(const hal::Port port, const hal::Pin pin) {

            auto pin_nbr{static_cast<uint8_t>(pin)};

            switch(port) {
                case Port::A:
                    return check_bit(pwr_reg->pdcra, pin_nbr);
                case Port::B:
                    return check_bit(pwr_reg->pdcrb, pin_nbr);
                case Port::C:
                    return check_bit(pwr_reg->pdcrc, pin_nbr);
                case Port::D:
                    return check_bit(pwr_reg->pdcrd, pin_nbr);
                case Port::E:
                    if(pin_nbr < 13) {

                        return check_bit(pwr_reg->pdcre, pin_nbr);
                    }
                    break;
                case Port::F:
                    if(pin_nbr < 13) {

                        return check_bit(pwr_reg->pdcrf, pin_nbr);
                    }
                    break;
            }

            return false;
        }

    protected:

    private:
        struct pwr_registers {
            uint32_t cr1;
            uint32_t cr2;
            uint32_t cr3;
            uint32_t cr4;
            uint32_t sr1;
            uint32_t sr2;
            uint32_t scr;
            uint32_t pucra;
            uint32_t pdcra;
            uint32_t pucrb;
            uint32_t pdcrb;
            uint32_t pucrc;
            uint32_t pdcrc;
            uint32_t pucrd;
            uint32_t pdcrd;
            uint32_t pucre;
            uint32_t pdcre;
            uint32_t pucrf;
            uint32_t pdcrf;
        };
        static_assert(sizeof(pwr_registers) == 76, "Error pwr registers struct size is not valid");

        volatile pwr_registers *pwr_reg;
    };
}

#endif // DRIVER_HAL_PWR_H
