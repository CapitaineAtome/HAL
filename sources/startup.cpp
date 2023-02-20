#include <cstdint>
#include <algorithm>

constexpr bool is_nullptr(void *ptr) {

    return ptr == nullptr;
}

volatile uint64_t system_tick{};

// For a STM32G031K8
constexpr uint32_t SRAM_START = 0x20000000u;
constexpr uint32_t SRAM_SIZE = (8u * 1024u);
constexpr uint32_t SRAM_END = (SRAM_START + SRAM_SIZE);
constexpr uint32_t STACK_POINTER_INIT_ADDRESS = SRAM_END;

constexpr uint8_t ISR_VECTOR_SIZE = 47;

using ptr_void_function = void(*)();

extern std::uint32_t _estack;
extern std::uint32_t _etext;
extern std::uint32_t _sdata;
extern std::uint32_t _edata;
extern std::uint32_t _sidata;
extern std::uint32_t _sbss;
extern std::uint32_t _ebss;

extern void (*__preinit_array_start [])(void)__attribute__((weak));
extern void (*__preinit_array_end   [])(void)__attribute__((weak));
extern void (*__init_array_start    [])(void)__attribute__((weak));
extern void (*__init_array_end      [])(void)__attribute__((weak));
extern void (*__fini_array_start    [])(void)__attribute__((weak));
extern void (*__fini_array_end      [])(void)__attribute__((weak));

int main();

static void init_bss() {

    std::fill(reinterpret_cast<uint8_t *>(&_sbss), reinterpret_cast<uint8_t *>(&_ebss), 0u);
}

static void init_data() {

    const auto size_data_to_copy = static_cast<std::size_t>(&_edata - &_sdata);

    // This code is 16 bytes less efficient
    std::copy(reinterpret_cast<uint8_t *>(&_etext), reinterpret_cast<uint8_t *>(&_etext) + size_data_to_copy, reinterpret_cast<uint8_t *>(&_sdata));

    // uint8_t *sram_data{reinterpret_cast<uint8_t *>(&_sdata)};
    // uint8_t *flash_data{reinterpret_cast<uint8_t *>(&_etext)};

    // for(std::size_t i = 0; i < size_data_to_copy; i++) {

    //     sram_data[i] = flash_data[i];
    // }

}

constexpr static void preinit_array() {

    std::for_each(__preinit_array_start, __preinit_array_end, [](ptr_void_function func) {
        func();
    });
}

constexpr static void init_array() {

    std::for_each(__init_array_start, __init_array_end, [](ptr_void_function func) {
        func();
    });
}

constexpr static void fini_array() {

    std::for_each(__fini_array_start, __fini_array_end, [](ptr_void_function func) {
        func();
    });
}

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

[[maybe_unused, noreturn, gnu::nothrow]] void Default_Handler() {

    uint8_t i{};
    while(true) {
        __asm volatile ("nop");
    }
}

[[noreturn, gnu::nothrow]] void NMI_Handler()          __attribute__((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void HardFault_Handler()    __attribute__ ((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void SVC_Handler()          __attribute__ ((weak, alias("Default_Handler")));
[[noreturn, gnu::nothrow]] void PendSV_Handler()       __attribute__ ((weak, alias("Default_Handler")));
// [[noreturn, gnu::nothrow]] void SysTick_Handler()      __attribute__ ((weak, alias("Default_Handler")));

void SysTick_Handler() {

    system_tick++;
}

[[noreturn]] void Reset_Handler() {

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
    while(true) {

    }
}

#ifdef __cplusplus
}

// Define the vector table
std::uint32_t isr_vector[ISR_VECTOR_SIZE] __attribute__((section(".isr_vector"))) = {
        STACK_POINTER_INIT_ADDRESS,
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(Reset_Handler)),
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(NMI_Handler)),
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(HardFault_Handler)),
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(SVC_Handler)),
        0,
        0,
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(PendSV_Handler)),
        static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(SysTick_Handler)),
        // Add more interrupt
};

#endif // __cplusplus
