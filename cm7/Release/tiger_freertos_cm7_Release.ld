/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for MIMXRT1176xxxxx
 * Created from linkscript.ldt by FMCreateLinkLibraries
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.8.0 [Build 1165] [2023-07-26] on May 7, 2024, 4:39:19 PM
 */

INCLUDE "tiger_freertos_cm7_Release_library.ld"
INCLUDE "tiger_freertos_cm7_Release_memory.ld"

ENTRY(ResetISR)

SECTIONS
{
     /* Image Vector Table and Boot Data for booting from external flash */
    .boot_hdr : ALIGN(4)
    {
        FILL(0xff)
        . = 0x400 ;
        __boot_hdr_start__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.conf))
        . = 0x1000 ;
        __boot_hdr_ivt_loadaddr__  =  ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.ivt))
        . = 0x1020 ;
        __boot_hdr_boot_data_loadaddr__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.boot_data))
        . = 0x1030 ;
        __boot_hdr_dcd_loadaddr__ = ABSOLUTE(.) ;
        KEEP(*(.boot_hdr.dcd_data))
        __boot_hdr_end__ = ABSOLUTE(.) ;
        . = 0x2000 ;
    } >BOARD_FLASH


    /* MAIN TEXT SECTION */
    .text : ALIGN(4)
    {
        FILL(0xff)
        __vectors_start__ = ABSOLUTE(.) ;
        KEEP(*(.isr_vector))
        /* Global Section Table */
        . = ALIGN(4) ;
        __section_table_start = .;
        __data_section_table = .;
        LONG(LOADADDR(.data));
        LONG(    ADDR(.data));
        LONG(  SIZEOF(.data));
        LONG(LOADADDR(.data_RAM2_core_m4slave_text));
        LONG(    ADDR(.data_RAM2_core_m4slave_text));
        LONG(  SIZEOF(.data_RAM2_core_m4slave_text));
        LONG(LOADADDR(.data_RAM2_core_m4slave_ARM_extab));
        LONG(    ADDR(.data_RAM2_core_m4slave_ARM_extab));
        LONG(  SIZEOF(.data_RAM2_core_m4slave_ARM_extab));
        LONG(LOADADDR(.data_RAM2_core_m4slave_ARM_exidx));
        LONG(    ADDR(.data_RAM2_core_m4slave_ARM_exidx));
        LONG(  SIZEOF(.data_RAM2_core_m4slave_ARM_exidx));
        LONG(LOADADDR(.data_RAM2_core_m4slave_data));
        LONG(    ADDR(.data_RAM2_core_m4slave_data));
        LONG(  SIZEOF(.data_RAM2_core_m4slave_data));
        LONG(LOADADDR(.data_RAM2));
        LONG(    ADDR(.data_RAM2));
        LONG(  SIZEOF(.data_RAM2));
        LONG(LOADADDR(.data_RAM3));
        LONG(    ADDR(.data_RAM3));
        LONG(  SIZEOF(.data_RAM3));
        LONG(LOADADDR(.data_RAM4));
        LONG(    ADDR(.data_RAM4));
        LONG(  SIZEOF(.data_RAM4));
        LONG(LOADADDR(.data_RAM5));
        LONG(    ADDR(.data_RAM5));
        LONG(  SIZEOF(.data_RAM5));
        __data_section_table_end = .;
        __bss_section_table = .;
        LONG(    ADDR(.bss));
        LONG(  SIZEOF(.bss));
        LONG(    ADDR(.bss_RAM2));
        LONG(  SIZEOF(.bss_RAM2));
        LONG(    ADDR(.bss_RAM3));
        LONG(  SIZEOF(.bss_RAM3));
        LONG(    ADDR(.bss_RAM4));
        LONG(  SIZEOF(.bss_RAM4));
        LONG(    ADDR(.bss_RAM5));
        LONG(  SIZEOF(.bss_RAM5));
        __bss_section_table_end = .;
        __section_table_end = . ;
        /* End of Global Section Table */

        *(.after_vectors*)

       *(.text*)
       *(.rodata .rodata.* .constdata .constdata.*)
       . = ALIGN(4);
    } > BOARD_FLASH
    /*
     * for exception handling/unwind - some Newlib functions (in common
     * with C++ and STDC++) use this.
     */
    .ARM.extab : ALIGN(4)
    {
        *(.ARM.extab* .gnu.linkonce.armextab.*)
    } > BOARD_FLASH

    .ARM.exidx : ALIGN(4)
    {
        __exidx_start = .;
        *(.ARM.exidx* .gnu.linkonce.armexidx.*)
        __exidx_end = .;
    } > BOARD_FLASH
 
    _etext = .;
        
    /* DATA section for OCRAM_ITCM_ALIAS */
    .data_RAM2_core_m4slave_text : 
    {
       FILL(0xff)
       PROVIDE(__start_data_RAM2 = .) ;
       __core_m4slave_START__ = .; /* start of slave image */
       KEEP(*(.core_m4slave))
    } > OCRAM_ITCM_ALIAS AT>BOARD_FLASH

    /* M4SLAVE extab and exidx sections */
    .data_RAM2_core_m4slave_ARM_extab :
    {
        FILL(0xff)
        KEEP(*(.core_m4slave.ARM.extab))
    } > OCRAM_ITCM_ALIAS AT>BOARD_FLASH 

    .data_RAM2_core_m4slave_ARM_exidx :
    {
        FILL(0xff)
        KEEP(*(.core_m4slave.ARM.exidx))
    } > OCRAM_ITCM_ALIAS AT>BOARD_FLASH 

    /* M4SLAVE data section */
    .data_RAM2_core_m4slave_data :
    {
        FILL(0xff)
        KEEP(*(.core_m4slave.data_*)) KEEP(*(.core_m4slave.data))
        __core_m4slave_END__ = .; /* end of slave image */

        /* perform some simple sanity checks */
        ASSERT(!(__core_m4slave_START__ == __core_m4slave_END__), "No slave code for _core_m4slave");
        /* Skip check due to potential use of a memory range which is seen at different addresses by the Master and Slave cores 
        ASSERT( (ABSOLUTE(__core_m4slave_START__) == __vectors_start___core_m4slave), "M4SLAVE execute address differs from address provided in source image");
        */
    } > OCRAM_ITCM_ALIAS AT>BOARD_FLASH 

    .data_RAM2 : ALIGN(4)
    {
        FILL(0xff)
        *(.ramfunc.$RAM2)
        *(.ramfunc.$OCRAM_ITCM_ALIAS)
        *(.data.$RAM2)
        *(.data.$OCRAM_ITCM_ALIAS)
        *(.data.$RAM2.*)
        *(.data.$OCRAM_ITCM_ALIAS.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM2 = .) ;
        PROVIDE(__end_data_OCRAM_ITCM_ALIAS = .) ;
     } > OCRAM_ITCM_ALIAS AT>BOARD_FLASH

    /* DATA section for rpmsg_sh_mem */

    .data_RAM3 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM3 = .) ;
        PROVIDE(__start_data_rpmsg_sh_mem = .) ;
        *(.ramfunc.$RAM3)
        *(.ramfunc.$rpmsg_sh_mem)
       *(rpmsg_sh_mem_section)
        *(.data.$RAM3)
        *(.data.$rpmsg_sh_mem)
        *(.data.$RAM3.*)
        *(.data.$rpmsg_sh_mem.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM3 = .) ;
        PROVIDE(__end_data_rpmsg_sh_mem = .) ;
     } > rpmsg_sh_mem AT>BOARD_FLASH

    /* DATA section for NCACHE_REGION */

    .data_RAM4 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM4 = .) ;
        PROVIDE(__start_data_NCACHE_REGION = .) ;
        *(.ramfunc.$RAM4)
        *(.ramfunc.$NCACHE_REGION)
        *(.data.$RAM4)
        *(.data.$NCACHE_REGION)
        *(.data.$RAM4.*)
        *(.data.$NCACHE_REGION.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM4 = .) ;
        PROVIDE(__end_data_NCACHE_REGION = .) ;
     } > NCACHE_REGION AT>BOARD_FLASH

    /* DATA section for SRAM_DTC_cm7 */

    .data_RAM5 : ALIGN(4)
    {
        FILL(0xff)
        PROVIDE(__start_data_RAM5 = .) ;
        PROVIDE(__start_data_SRAM_DTC_cm7 = .) ;
        *(.ramfunc.$RAM5)
        *(.ramfunc.$SRAM_DTC_cm7)
        *(.data.$RAM5)
        *(.data.$SRAM_DTC_cm7)
        *(.data.$RAM5.*)
        *(.data.$SRAM_DTC_cm7.*)
        . = ALIGN(4) ;
        PROVIDE(__end_data_RAM5 = .) ;
        PROVIDE(__end_data_SRAM_DTC_cm7 = .) ;
     } > SRAM_DTC_cm7 AT>BOARD_FLASH

    /* MAIN DATA SECTION */
    .uninit_RESERVED (NOLOAD) : ALIGN(4)
    {
        _start_uninit_RESERVED = .;
        KEEP(*(.bss.$RESERVED*))
       . = ALIGN(4) ;
        _end_uninit_RESERVED = .;
    } > BOARD_SDRAM AT> BOARD_SDRAM

    /* Main DATA section (BOARD_SDRAM) */
    .data : ALIGN(4)
    {
       FILL(0xff)
       _data = . ;
       PROVIDE(__start_data_RAM = .) ;
       PROVIDE(__start_data_BOARD_SDRAM = .) ;
       *(vtable)
       *(.ramfunc*)
       KEEP(*(CodeQuickAccess))
       KEEP(*(DataQuickAccess))
       *(RamFunction)
       *(NonCacheable.init)
       *(.data*)
       . = ALIGN(4) ;
       _edata = . ;
       PROVIDE(__end_data_RAM = .) ;
       PROVIDE(__end_data_BOARD_SDRAM = .) ;
    } > BOARD_SDRAM AT>BOARD_FLASH

    /* BSS section for OCRAM_ITCM_ALIAS */
    .bss_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM2 = .) ;
       PROVIDE(__start_bss_OCRAM_ITCM_ALIAS = .) ;
       *(.bss.$RAM2)
       *(.bss.$OCRAM_ITCM_ALIAS)
       *(.bss.$RAM2.*)
       *(.bss.$OCRAM_ITCM_ALIAS.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM2 = .) ;
       PROVIDE(__end_bss_OCRAM_ITCM_ALIAS = .) ;
    } > OCRAM_ITCM_ALIAS AT> OCRAM_ITCM_ALIAS

    /* BSS section for rpmsg_sh_mem */
    .bss_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM3 = .) ;
       PROVIDE(__start_bss_rpmsg_sh_mem = .) ;
       *(.bss.$RAM3)
       *(.bss.$rpmsg_sh_mem)
       *(.bss.$RAM3.*)
       *(.bss.$rpmsg_sh_mem.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM3 = .) ;
       PROVIDE(__end_bss_rpmsg_sh_mem = .) ;
    } > rpmsg_sh_mem AT> rpmsg_sh_mem

    /* BSS section for NCACHE_REGION */
    .bss_RAM4 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM4 = .) ;
       PROVIDE(__start_bss_NCACHE_REGION = .) ;
       *(.bss.$RAM4)
       *(.bss.$NCACHE_REGION)
       *(.bss.$RAM4.*)
       *(.bss.$NCACHE_REGION.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM4 = .) ;
       PROVIDE(__end_bss_NCACHE_REGION = .) ;
    } > NCACHE_REGION AT> NCACHE_REGION

    /* BSS section for SRAM_DTC_cm7 */
    .bss_RAM5 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_bss_RAM5 = .) ;
       PROVIDE(__start_bss_SRAM_DTC_cm7 = .) ;
       *(.bss.$RAM5)
       *(.bss.$SRAM_DTC_cm7)
       *(.bss.$RAM5.*)
       *(.bss.$SRAM_DTC_cm7.*)
       . = ALIGN (. != 0 ? 4 : 1) ; /* avoid empty segment */
       PROVIDE(__end_bss_RAM5 = .) ;
       PROVIDE(__end_bss_SRAM_DTC_cm7 = .) ;
    } > SRAM_DTC_cm7 AT> SRAM_DTC_cm7

    /* MAIN BSS SECTION */
    .bss (NOLOAD) : ALIGN(4)
    {
        _bss = .;
        PROVIDE(__start_bss_RAM = .) ;
        PROVIDE(__start_bss_BOARD_SDRAM = .) ;
       *(NonCacheable)
        *(.bss*)
        *(COMMON)
        . = ALIGN(4) ;
        _ebss = .;
        PROVIDE(__end_bss_RAM = .) ;
        PROVIDE(__end_bss_BOARD_SDRAM = .) ;
        PROVIDE(end = .);
    } > BOARD_SDRAM AT> BOARD_SDRAM

    /* NOINIT section for OCRAM_ITCM_ALIAS */
    .noinit_RAM2 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM2 = .) ;
       PROVIDE(__start_noinit_OCRAM_ITCM_ALIAS = .) ;
       *(.noinit.$RAM2)
       *(.noinit.$OCRAM_ITCM_ALIAS)
       *(.noinit.$RAM2.*)
       *(.noinit.$OCRAM_ITCM_ALIAS.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM2 = .) ;
       PROVIDE(__end_noinit_OCRAM_ITCM_ALIAS = .) ;
    } > OCRAM_ITCM_ALIAS AT> OCRAM_ITCM_ALIAS

    /* NOINIT section for rpmsg_sh_mem */
    .noinit_RAM3 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM3 = .) ;
       PROVIDE(__start_noinit_rpmsg_sh_mem = .) ;
       *(.noinit.$RAM3)
       *(.noinit.$rpmsg_sh_mem)
       *(.noinit.$RAM3.*)
       *(.noinit.$rpmsg_sh_mem.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM3 = .) ;
       PROVIDE(__end_noinit_rpmsg_sh_mem = .) ;
    } > rpmsg_sh_mem AT> rpmsg_sh_mem

    /* NOINIT section for NCACHE_REGION */
    .noinit_RAM4 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM4 = .) ;
       PROVIDE(__start_noinit_NCACHE_REGION = .) ;
       *(.noinit.$RAM4)
       *(.noinit.$NCACHE_REGION)
       *(.noinit.$RAM4.*)
       *(.noinit.$NCACHE_REGION.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM4 = .) ;
       PROVIDE(__end_noinit_NCACHE_REGION = .) ;
    } > NCACHE_REGION AT> NCACHE_REGION

    /* NOINIT section for SRAM_DTC_cm7 */
    .noinit_RAM5 (NOLOAD) : ALIGN(4)
    {
       PROVIDE(__start_noinit_RAM5 = .) ;
       PROVIDE(__start_noinit_SRAM_DTC_cm7 = .) ;
       *(.noinit.$RAM5)
       *(.noinit.$SRAM_DTC_cm7)
       *(.noinit.$RAM5.*)
       *(.noinit.$SRAM_DTC_cm7.*)
       . = ALIGN(4) ;
       PROVIDE(__end_noinit_RAM5 = .) ;
       PROVIDE(__end_noinit_SRAM_DTC_cm7 = .) ;
    } > SRAM_DTC_cm7 AT> SRAM_DTC_cm7

    /* DEFAULT NOINIT SECTION */
    .noinit (NOLOAD): ALIGN(4)
    {
        _noinit = .;
        PROVIDE(__start_noinit_RAM = .) ;
        PROVIDE(__start_noinit_BOARD_SDRAM = .) ;
        *(.noinit*)
         . = ALIGN(4) ;
        _end_noinit = .;
       PROVIDE(__end_noinit_RAM = .) ;
       PROVIDE(__end_noinit_BOARD_SDRAM = .) ;        
    } > BOARD_SDRAM AT> BOARD_SDRAM

    /* Reserve and place Heap within memory map */
    _HeapSize = 0x1000;
    .heap (NOLOAD) :  ALIGN(4)
    {
        _pvHeapStart = .;
        . += _HeapSize;
        . = ALIGN(4);
        _pvHeapLimit = .;
    } > BOARD_SDRAM

     _StackSize = 0x1000;
     /* Reserve space in memory for Stack */
    .heap2stackfill (NOLOAD) :
    {
        . += _StackSize;
    } > BOARD_SDRAM
    /* Locate actual Stack in memory map */
    .stack ORIGIN(BOARD_SDRAM) + LENGTH(BOARD_SDRAM) - _StackSize - 0 (NOLOAD) :  ALIGN(4)
    {
        _vStackBase = .;
        . = ALIGN(4);
        _vStackTop = . + _StackSize;
    } > BOARD_SDRAM

    /* Provide basic symbols giving location and size of main text
     * block, including initial values of RW data sections. Note that
     * these will need extending to give a complete picture with
     * complex images (e.g multiple Flash banks).
     */
    _image_start = LOADADDR(.text);
    _image_end = LOADADDR(.data) + SIZEOF(.data);
    _image_size = _image_end - _image_start;
    /* Provide symbols for MIMXRT1170 parts for boot header generation code
     * to set image to be plain load image or XIP.
     * Config : Plain load image = false
     */
    _boot_loadaddr = ORIGIN(BOARD_FLASH);
    _boot_size = LENGTH(BOARD_FLASH);
}