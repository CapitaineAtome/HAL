//
// Created by marmelade on 23/01/23.
//
#include "hal/rcc.h"
#include "hal/gpio.h"

[[noreturn]] int main() {

    RCC rcc{};
    GPIOPort port{GPIOPort::Port::C};

    volatile int i = 0;
    i++;

    port.Mode(GPIOPort::Pin::P6, GPIOPort::Mode::OUTPUT);

    while(true) {

        i++;
        port.Toggle(GPIOPort::Pin::P6);

    }
}
