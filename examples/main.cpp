//
// Created by marmelade on 23/01/23.
//
#include "hal/pwr.h"
#include "hal/rcc.h"
#include "hal/gpio.h"

/*
 * LD3 USER
 * The LD3 USER green LED is connected to the following STM32G031K8T6 I/O:
 * PB3, if the configuration is SB12 ON, and SB13 OFF
 * PC6, if the configuration is SB12 OFF, and SB13 ON (default configuration)
 */

const char magic[] = "Caca";

bool test_magic() {

    const char test_[] = "Caca";

    for(std::size_t i{0}; i < 5; i++) {

        if(magic[i] != test_[i]) {

            return true;
        }
    }

    return false;
}
[[noreturn]] int main() {

    volatile uint32_t i = test_magic();

    hal::RCC rcc{};
    hal::GPIOPort port{hal::Port::C};

    // GPIOE->MODER |= 1 << 26; // Enable PORTC pin 8 as digital output
    // GPIOE -> ODR |= 1<<13; // Turn GPIOC Pin 8 ON

    port.InitOutput(hal::Pin::P6);

    while(true) {

        i++;
        port.Toggle(hal::Pin::P6);
        for(unsigned int j=0; j < 1000000; j++)
            ;

    }

}
