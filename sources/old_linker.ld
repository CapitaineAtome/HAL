ENTRY(Reset_Handler) /* Where to start */

MEMORY {

    FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 64K /* Check rm0444 Figure 5 */
    RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 8K
}

SECTIONS {

    .isr_vector : {

        KEEP(* (.isr_vector))
    } >FLASH

    .text : {

        . = ALIGN(4);

        *(.text)
        *(.rodata)

        . = ALIGN(4);

        text_end__ = .;
    }

    .data : {

        . = ALIGN(4);

        data_start__ = .;

        *(.data)

        . = ALIGN(4);

        data_end__ = .;
    } >RAM AT> FLASH
    /*
     * RAM   :   Virtual memory address ( relocation address ), address the section will have
     *          when the program is executed.
     * FLASH :  Load memory address, address where the section is loaded in the target.
     */

    .bss : {

        . = ALIGN(4);

        bss_start__ = .;

        *(.bss)

        . = ALIGN(4);

        bss_end__ = .;
    } >RAM
}
