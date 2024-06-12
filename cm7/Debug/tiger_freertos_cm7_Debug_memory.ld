/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2024 NXP
 * Generated linker script file for MIMXRT1176xxxxx
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v11.8.0 [Build 1165] [2023-07-26] on May 7, 2024, 4:37:59 PM
 */

MEMORY
{
  /* Define each memory region */
  BOARD_FLASH (rx) : ORIGIN = 0x30000000, LENGTH = 0x4000000 /* 64M bytes (alias Flash) */  
  BOARD_SDRAM (rwx) : ORIGIN = 0x80000000, LENGTH = 0x2000000 /* 32M bytes (alias RAM) */  
  OCRAM_ITCM_ALIAS (rwx) : ORIGIN = 0x20200000, LENGTH = 0x20000 /* 128K bytes (alias RAM2) */  
  rpmsg_sh_mem (rwx) : ORIGIN = 0x20220000, LENGTH = 0x2000 /* 8K bytes (alias RAM3) */  
  NCACHE_REGION (rwx) : ORIGIN = 0x82000000, LENGTH = 0x2000000 /* 32M bytes (alias RAM4) */  
  SRAM_DTC_cm7 (rwx) : ORIGIN = 0x20000000, LENGTH = 0x40000 /* 256K bytes (alias RAM5) */  
}

  /* Define a symbol for the top of each memory region */
  __base_BOARD_FLASH = 0x30000000  ; /* BOARD_FLASH */  
  __base_Flash = 0x30000000 ; /* Flash */  
  __top_BOARD_FLASH = 0x30000000 + 0x4000000 ; /* 64M bytes */  
  __top_Flash = 0x30000000 + 0x4000000 ; /* 64M bytes */  
  __base_BOARD_SDRAM = 0x80000000  ; /* BOARD_SDRAM */  
  __base_RAM = 0x80000000 ; /* RAM */  
  __top_BOARD_SDRAM = 0x80000000 + 0x2000000 ; /* 32M bytes */  
  __top_RAM = 0x80000000 + 0x2000000 ; /* 32M bytes */  
  __base_OCRAM_ITCM_ALIAS = 0x20200000  ; /* OCRAM_ITCM_ALIAS */  
  __base_RAM2 = 0x20200000 ; /* RAM2 */  
  __top_OCRAM_ITCM_ALIAS = 0x20200000 + 0x20000 ; /* 128K bytes */  
  __top_RAM2 = 0x20200000 + 0x20000 ; /* 128K bytes */  
  __base_rpmsg_sh_mem = 0x20220000  ; /* rpmsg_sh_mem */  
  __base_RAM3 = 0x20220000 ; /* RAM3 */  
  __top_rpmsg_sh_mem = 0x20220000 + 0x2000 ; /* 8K bytes */  
  __top_RAM3 = 0x20220000 + 0x2000 ; /* 8K bytes */  
  __base_NCACHE_REGION = 0x82000000  ; /* NCACHE_REGION */  
  __base_RAM4 = 0x82000000 ; /* RAM4 */  
  __top_NCACHE_REGION = 0x82000000 + 0x2000000 ; /* 32M bytes */  
  __top_RAM4 = 0x82000000 + 0x2000000 ; /* 32M bytes */  
  __base_SRAM_DTC_cm7 = 0x20000000  ; /* SRAM_DTC_cm7 */  
  __base_RAM5 = 0x20000000 ; /* RAM5 */  
  __top_SRAM_DTC_cm7 = 0x20000000 + 0x40000 ; /* 256K bytes */  
  __top_RAM5 = 0x20000000 + 0x40000 ; /* 256K bytes */  
