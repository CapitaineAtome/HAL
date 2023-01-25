#include <cstdint>
#include <algorithm>

// For a STM32G031K8
/*constexpr uint32_t*/ #define SRAM_START 0x20000000u
/*constexpr uint32_t*/ #define SRAM_SIZE (8u * 1024u)
/*constexpr uint32_t*/ #define SRAM_END (SRAM_START + SRAM_SIZE)
/*constexpr uint32_t*/ #define STACK_POINTER_INIT_ADDRESS SRAM_END

/*constexpr uint8_t*/ #define ISR_VECTOR_SIZE 47

#define endless_loop while(true) {}

using init_array_type = void(*)();

extern std::uint32_t _estack;
extern std::uint32_t _etext;
extern std::uint32_t _sdata;
extern std::uint32_t _edata;
extern std::uint32_t _sidata;
extern std::uint32_t _sbss;
extern std::uint32_t _ebss;

extern init_array_type __preinit_array_start [];
extern init_array_type __preinit_array_end   [];
extern init_array_type __init_array_start    [];
extern init_array_type __init_array_end      [];
extern init_array_type __fini_array_start    [];
extern init_array_type __fini_array_end      [];

int main();

static void init_bss() {

    std::fill(&_sbss, &_ebss, 0u);
}

static void init_data() {

    auto size_to_copy = static_cast<std::size_t>(&_edata - &_sdata);

    for(std::size_t i = 0; i < size_to_copy; i++) {

        ((uint8_t *)_sdata)[i] = ((uint8_t *)_sidata)[i];
    }
}

static void preinit_array() {

    std::for_each(__preinit_array_start, __preinit_array_end, [](const init_array_type func) {
        func();
    });
}

static void init_array() {

    std::for_each(__init_array_start, __init_array_end, [](const init_array_type func) {
        func();
    });
}

static void fini_array() {

    std::for_each(__fini_array_start, __fini_array_end, [](const init_array_type func) {
        func();
    });
}

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

[[maybe_unused, noreturn, gnu::nothrow]] void Default_Handler() {

    int i = 0;
    while(true) {

        i++;
    }
}

[[noreturn, gnu::nothrow]] void NMI_Handler()          __attribute__((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void HardFault_Handler()    __attribute__ ((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void SVC_Handler()          __attribute__ ((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void PendSV_Handler()       __attribute__ ((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void SysTick_Handler()      __attribute__ ((weak, alias("Default_Handler")));

void Reset_Handler() {

    /*asm(//".syntax unified\n"
        "ldr r0, =_estack\n"
        "mov sp, r0\n"
        ".align 4");*/



    // Flash to RAM
    init_data();

    // Fill BSS with 0
    init_bss();

    // Call ctors
    preinit_array();
    init_array();

    // Now we start !
    main();

    // Just in case, call dtors
    fini_array();

    // Stall
    endless_loop
}

// Define the vector table
std::uint32_t isr_vector[ISR_VECTOR_SIZE] __attribute__((section(".isr_vector"))) = {
        SRAM_END,
        (std::uint32_t) &Reset_Handler,
        (std::uint32_t) &NMI_Handler,
        (std::uint32_t) &HardFault_Handler,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        (std::uint32_t) &SVC_Handler,
        0,
        0,
        (std::uint32_t) &PendSV_Handler,
        (std::uint32_t) &SysTick_Handler,
        // Add more interrupt
};

#ifdef __cplusplus
}
#endif // __cplusplus

/*
 * LD3 USER
 * The LD3 USER green LED is connected to the following STM32G031K8T6 I/O:
 * PB3, if the configuration is SB12 ON, and SB13 OFF
 * PC6, if the configuration is SB12 OFF, and SB13 ON (default configuration)
 */
