/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for MIMXRT1176xxxxx
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.8.0 [Build 1165] [2023-07-26] on May 7, 2024, 4:39:39 PM
 */

MEMORY
{
  /* Define each memory region */
  SRAM_ITC_INT_cm4 (rwx) : ORIGIN = 0x1ffe0000, LENGTH = 0x20000 /* 128K bytes (alias RAM) */  
  DTCM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000 /* 128K bytes (alias RAM2) */  
  rpmsg_sh_mem (rwx) : ORIGIN = 0x202c0000, LENGTH = 0x2000 /* 8K bytes (alias RAM3) */  
  NCACHE_REGION (rwx) : ORIGIN = 0x20280000, LENGTH = 0x40000 /* 256K bytes (alias RAM4) */  
  SDRAM (rwx) : ORIGIN = 0x80000000, LENGTH = 0x4000000 /* 64M bytes (alias RAM5) */  
}

  /* Define a symbol for the top of each memory region */
  __base_SRAM_ITC_INT_cm4 = 0x1ffe0000  ; /* SRAM_ITC_INT_cm4 */  
  __base_RAM = 0x1ffe0000 ; /* RAM */  
  __top_SRAM_ITC_INT_cm4 = 0x1ffe0000 + 0x20000 ; /* 128K bytes */  
  __top_RAM = 0x1ffe0000 + 0x20000 ; /* 128K bytes */  
  __base_DTCM = 0x20000000  ; /* DTCM */  
  __base_RAM2 = 0x20000000 ; /* RAM2 */  
  __top_DTCM = 0x20000000 + 0x20000 ; /* 128K bytes */  
  __top_RAM2 = 0x20000000 + 0x20000 ; /* 128K bytes */  
  __base_rpmsg_sh_mem = 0x202c0000  ; /* rpmsg_sh_mem */  
  __base_RAM3 = 0x202c0000 ; /* RAM3 */  
  __top_rpmsg_sh_mem = 0x202c0000 + 0x2000 ; /* 8K bytes */  
  __top_RAM3 = 0x202c0000 + 0x2000 ; /* 8K bytes */  
  __base_NCACHE_REGION = 0x20280000  ; /* NCACHE_REGION */  
  __base_RAM4 = 0x20280000 ; /* RAM4 */  
  __top_NCACHE_REGION = 0x20280000 + 0x40000 ; /* 256K bytes */  
  __top_RAM4 = 0x20280000 + 0x40000 ; /* 256K bytes */  
  __base_SDRAM = 0x80000000  ; /* SDRAM */  
  __base_RAM5 = 0x80000000 ; /* RAM5 */  
  __top_SDRAM = 0x80000000 + 0x4000000 ; /* 64M bytes */  
  __top_RAM5 = 0x80000000 + 0x4000000 ; /* 64M bytes */  
