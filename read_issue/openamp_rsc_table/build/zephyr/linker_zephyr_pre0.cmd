OUTPUT_ARCH(xtensa)
PROVIDE(__memctl_default = 0x00000000);
PROVIDE(_MemErrorHandler = 0x00000000);
MEMORY
{
  vector_reset_text :
        org = 0x3B6F8000,
        len = 0x2E0
  vector_reset_lit :
        org = 0x3B6F8000 + 0x2E0,
        len = 0x120
  vector_base_text :
 org = (0x3B6F8000 + 0x400),
        len = 0x178
  vector_int2_lit :
 org = ((0x3B6F8000 + 0x400) + 0x17C) - 0x4,
        len = 0x4
  vector_int2_text :
 org = ((0x3B6F8000 + 0x400) + 0x17C),
        len = 0x1C
  vector_int3_lit :
 org = ((0x3B6F8000 + 0x400) + 0x19C) - 0x4,
        len = 0x4
  vector_int3_text :
 org = ((0x3B6F8000 + 0x400) + 0x19C),
        len = 0x1C
  vector_int4_lit :
 org = ((0x3B6F8000 + 0x400) + 0x1BC) - 0x4,
        len = 0x4
  vector_int4_text :
 org = ((0x3B6F8000 + 0x400) + 0x1BC),
        len = 0x1C
  vector_int5_lit :
 org = ((0x3B6F8000 + 0x400) + 0x1DC) - 0x4,
        len = 0x4
  vector_int5_text :
 org = ((0x3B6F8000 + 0x400) + 0x1DC),
        len = 0x1C
  vector_kernel_lit :
 org = ((0x3B6F8000 + 0x400) + 0x1FC) - 0x4,
        len = 0x4
  vector_kernel_text :
 org = ((0x3B6F8000 + 0x400) + 0x1FC),
        len = 0x1C
  vector_user_lit :
 org = ((0x3B6F8000 + 0x400) + 0x21C) - 0x4,
        len = 0x4
  vector_user_text :
 org = ((0x3B6F8000 + 0x400) + 0x21C),
        len = 0x1C
  vector_double_lit :
 org = ((0x3B6F8000 + 0x400) + 0x23C) - 0x4,
        len = 0x4
  vector_double_text :
 org = ((0x3B6F8000 + 0x400) + 0x23C),
        len = 0x1C
  iram_text_start :
 org = ((0x3B6F8000 + 0x400) + 0x23C) + 0x1C,
        len = (0x3B6F8000 + 0x800) - (0x3B6F863C + 0x1C)
  sdram0 :
 org = 0x92400000,
        len = 0x800000
  sdram1 :
 org = 0x92C00000 + (0x1000 + 0x1000 + 0x800 + 0x800 + 0x1000 + 0x1000),
        len = 0x800000 - (0x1000 + 0x1000 + 0x800 + 0x800 + 0x1000 + 0x1000)
  IDT_LIST :
 org = (0x3B6F8000 + 0x800),
        len = 0x2000
  static_uuid_entries_seg (!ari) :
        org = 0x1FFFA000,
        len = 0x6000
  static_log_entries_seg (!ari) :
        org = 0x20000000,
        len = 0x2000000
  fw_metadata_seg (!ari) :
        org = (0x20000000 + 0x2000000),
        len = 0x2000000
}
PHDRS
{
  vector_reset_text_phdr PT_LOAD;
  vector_reset_lit_phdr PT_LOAD;
  vector_base_text_phdr PT_LOAD;
  vector_base_lit_phdr PT_LOAD;
  vector_int2_text_phdr PT_LOAD;
  vector_int2_lit_phdr PT_LOAD;
  vector_int3_text_phdr PT_LOAD;
  vector_int3_lit_phdr PT_LOAD;
  vector_int4_text_phdr PT_LOAD;
  vector_int4_lit_phdr PT_LOAD;
  vector_int5_text_phdr PT_LOAD;
  vector_int5_lit_phdr PT_LOAD;
  vector_kernel_text_phdr PT_LOAD;
  vector_kernel_lit_phdr PT_LOAD;
  vector_user_text_phdr PT_LOAD;
  vector_user_lit_phdr PT_LOAD;
  vector_double_text_phdr PT_LOAD;
  vector_double_lit_phdr PT_LOAD;
  iram_text_start_phdr PT_LOAD;
  sdram0_phdr PT_LOAD;
  sdram1_phdr PT_LOAD;
  static_uuid_entries_phdr PT_NOTE;
  static_log_entries_phdr PT_NOTE;
  metadata_entries_phdr PT_NOTE;
}
_rom_store_table = 0;
PROVIDE(_memmap_vecbase_reset = 0x3B6F8400);
ENTRY("__start")
_memmap_cacheattr_wb_base = 0x44024000;
_memmap_cacheattr_wt_base = 0x11021000;
_memmap_cacheattr_bp_base = 0x22022000;
_memmap_cacheattr_unused_mask = 0x00F00FFF;
_memmap_cacheattr_wb_trapnull = 0x4422422F;
_memmap_cacheattr_wba_trapnull = 0x4422422F;
_memmap_cacheattr_wbna_trapnull = 0x25222222;
_memmap_cacheattr_wt_trapnull = 0x1122122F;
_memmap_cacheattr_bp_trapnull = 0x2222222F;
_memmap_cacheattr_wb_strict = 0x44F24FFF;
_memmap_cacheattr_wt_strict = 0x11F21FFF;
_memmap_cacheattr_bp_strict = 0x22F22FFF;
_memmap_cacheattr_wb_allvalid = 0x44224222;
_memmap_cacheattr_wt_allvalid = 0x11221222;
_memmap_cacheattr_bp_allvalid = 0x22222222;
_memmap_cacheattr_imx8_wt_allvalid = 0x22212222;
PROVIDE(_memmap_cacheattr_reset = _memmap_cacheattr_imx8_wt_allvalid);
_EXT_MAN_ALIGN_ = 16;
EXTERN(ext_man_fw_ver)
SECTIONS
{
    .resource_table : SUBALIGN(4)
    {
      KEEP(*(.resource_table*))
    } > sdram0 :sdram0_phdr
 .rel.plt :
 {
 *(.rel.plt)
 PROVIDE_HIDDEN (__rel_iplt_start = .);
 *(.rel.iplt)
 PROVIDE_HIDDEN (__rel_iplt_end = .);
 }
 .rela.plt :
 {
 *(.rela.plt)
 PROVIDE_HIDDEN (__rela_iplt_start = .);
 *(.rela.iplt)
 PROVIDE_HIDDEN (__rela_iplt_end = .);
 }
 .rel.dyn :
 {
 *(.rel.*)
 }
 .rela.dyn :
 {
 *(.rela.*)
 }
  .ResetVector.text : ALIGN(4)
  {
    _ResetVector_text_start = ABSOLUTE(.);
    KEEP (*(.ResetVector.text))
    _ResetVector_text_end = ABSOLUTE(.);
  } >vector_reset_text :vector_reset_text_phdr
  .ResetVector.literal : ALIGN(4)
  {
    _ResetVector_literal_start = ABSOLUTE(.);
    *(.ResetVector.literal)
    _ResetVector_literal_end = ABSOLUTE(.);
  } >vector_reset_lit :vector_reset_lit_phdr
  .WindowVectors.text : ALIGN(4)
  {
    _WindowVectors_text_start = ABSOLUTE(.);
    KEEP (*(.WindowVectors.text))
    _WindowVectors_text_end = ABSOLUTE(.);
  } >vector_base_text :vector_base_text_phdr
  .Level2InterruptVector.literal : ALIGN(4)
  {
    _Level2InterruptVector_literal_start = ABSOLUTE(.);
    *(.Level2InterruptVector.literal)
    _Level2InterruptVector_literal_end = ABSOLUTE(.);
  } >vector_int2_lit :vector_int2_lit_phdr
  .Level2InterruptVector.text : ALIGN(4)
  {
    _Level2InterruptVector_text_start = ABSOLUTE(.);
    KEEP (*(.Level2InterruptVector.text))
    _Level2InterruptVector_text_end = ABSOLUTE(.);
  } >vector_int2_text :vector_int2_text_phdr
  .Level3InterruptVector.literal : ALIGN(4)
  {
    _Level3InterruptVector_literal_start = ABSOLUTE(.);
    *(.Level3InterruptVector.literal)
    _Level3InterruptVector_literal_end = ABSOLUTE(.);
  } >vector_int3_lit :vector_int3_lit_phdr
  .Level3InterruptVector.text : ALIGN(4)
  {
    _Level3InterruptVector_text_start = ABSOLUTE(.);
    KEEP (*(.Level3InterruptVector.text))
    _Level3InterruptVector_text_end = ABSOLUTE(.);
  } >vector_int3_text :vector_int3_text_phdr
  .DebugExceptionVector.literal : ALIGN(4)
  {
    _DebugExceptionVector_literal_start = ABSOLUTE(.);
    *(.DebugExceptionVector.literal)
    _DebugExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_int4_lit :vector_int4_lit_phdr
  .DebugExceptionVector.text : ALIGN(4)
  {
    _DebugExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.DebugExceptionVector.text))
    _DebugExceptionVector_text_end = ABSOLUTE(.);
  } >vector_int4_text :vector_int4_text_phdr
  .NMIExceptionVector.literal : ALIGN(4)
  {
    _NMIExceptionVector_literal_start = ABSOLUTE(.);
    *(.NMIExceptionVector.literal)
    _NMIExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_int5_lit :vector_int5_lit_phdr
  .NMIExceptionVector.text : ALIGN(4)
  {
    _NMIExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.NMIExceptionVector.text))
    _NMIExceptionVector_text_end = ABSOLUTE(.);
  } >vector_int5_text :vector_int5_text_phdr
  .KernelExceptionVector.literal : ALIGN(4)
  {
    _KernelExceptionVector_literal_start = ABSOLUTE(.);
    *(.KernelExceptionVector.literal)
    _KernelExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_kernel_lit :vector_kernel_lit_phdr
  .KernelExceptionVector.text : ALIGN(4)
  {
    _KernelExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.KernelExceptionVector.text))
    _KernelExceptionVector_text_end = ABSOLUTE(.);
  } >vector_kernel_text :vector_kernel_text_phdr
  .UserExceptionVector.literal : ALIGN(4)
  {
    _UserExceptionVector_literal_start = ABSOLUTE(.);
    *(.UserExceptionVector.literal)
    _UserExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_user_lit :vector_user_lit_phdr
  .UserExceptionVector.text : ALIGN(4)
  {
    _UserExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.UserExceptionVector.text))
    _UserExceptionVector_text_end = ABSOLUTE(.);
  } >vector_user_text :vector_user_text_phdr
  .DoubleExceptionVector.literal : ALIGN(4)
  {
    _DoubleExceptionVector_literal_start = ABSOLUTE(.);
    *(.DoubleExceptionVector.literal)
    _DoubleExceptionVector_literal_end = ABSOLUTE(.);
  } >vector_double_lit :vector_double_lit_phdr
  .DoubleExceptionVector.text : ALIGN(4)
  {
    _DoubleExceptionVector_text_start = ABSOLUTE(.);
    KEEP (*(.DoubleExceptionVector.text))
    _DoubleExceptionVector_text_end = ABSOLUTE(.);
  } >vector_double_text :vector_double_text_phdr
  .iram.text : ALIGN(4)
  {
    _stext = .;
    _iram_text_start = ABSOLUTE(.);
    *(.iram0.literal .iram.literal .iram.text.literal .iram0.text .iram.text)
    _iram_text_end = ABSOLUTE(.);
  } >iram_text_start :iram_text_start_phdr
  .rodata : ALIGN(4)
  {
    __rodata_region_start = ABSOLUTE(.);
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    __XT_EXCEPTION_TABLE__ = ABSOLUTE(.);
    KEEP (*(.xt_except_table))
    KEEP (*(.gcc_except_table .gcc_except_table.*))
    *(.gnu.linkonce.e.*)
    *(.gnu.version_r)
    KEEP (*(.eh_frame))
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*(.ctors))
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*(.dtors))
    __XT_EXCEPTION_DESCS__ = ABSOLUTE(.);
    *(.xt_except_desc)
    *(.gnu.linkonce.h.*)
    __XT_EXCEPTION_DESCS_END__ = ABSOLUTE(.);
    *(.xt_except_desc_end)
    *(.dynamic)
    *(.gnu.version_d)
    . = ALIGN(4);
    _bss_table_start = ABSOLUTE(.);
    LONG(_bss_start)
    LONG(_bss_end)
    _bss_table_end = ABSOLUTE(.);
    __rodata_region_end = ABSOLUTE(.);
  } >sdram0 :sdram0_phdr
  .module_init : ALIGN(4)
  {
   _module_init_start = ABSOLUTE(.);
    *(*.initcall)
    _module_init_end = ABSOLUTE(.);
  } >sdram0 :sdram0_phdr
  .text : ALIGN(4)
  {
    _stext = .;
    __text_region_start = ABSOLUTE(.);
    KEEP (*(.ResetVector.text))
    *(.ResetVector.literal)
    *(.entry.text)
    *(.init.literal)
    KEEP(*(.init))
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.fini.literal)
    KEEP(*(.fini))
    *(.gnu.version)
    __text_region_end = ABSOLUTE(.);
    _etext = .;
  } >sdram0 :sdram0_phdr
 initlevel :
 {
  __init_start = .;
  __init_EARLY_start = .; KEEP(*(SORT(.z_init_EARLY?_*))); KEEP(*(SORT(.z_init_EARLY??_*)));
  __init_PRE_KERNEL_1_start = .; KEEP(*(SORT(.z_init_PRE_KERNEL_1?_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_1??_*)));
  __init_PRE_KERNEL_2_start = .; KEEP(*(SORT(.z_init_PRE_KERNEL_2?_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_2??_*)));
  __init_POST_KERNEL_start = .; KEEP(*(SORT(.z_init_POST_KERNEL?_*))); KEEP(*(SORT(.z_init_POST_KERNEL??_*)));
  __init_APPLICATION_start = .; KEEP(*(SORT(.z_init_APPLICATION?_*))); KEEP(*(SORT(.z_init_APPLICATION??_*)));
  __init_SMP_start = .; KEEP(*(SORT(.z_init_SMP?_*))); KEEP(*(SORT(.z_init_SMP??_*)));
  __init_end = .;
  __deferred_init_list_start = .;
  KEEP(*(.z_deferred_init*))
  __deferred_init_list_end = .;
 } > sdram0 :sdram0_phdr
 device_area : SUBALIGN(4) { _device_list_start = .; KEEP(*(SORT(._device.static.*_?_*))); KEEP(*(SORT(._device.static.*_??_*))); KEEP(*(SORT(._device.static.*_???_*))); KEEP(*(SORT(._device.static.*_????_*))); KEEP(*(SORT(._device.static.*_?????_*))); _device_list_end = .; } > sdram0 :sdram0_phdr
 initlevel_error :
 {
  KEEP(*(SORT(.z_init_[_A-Z0-9]*)))
 }
 ASSERT(SIZEOF(initlevel_error) == 0, "Undefined initialization levels used.")
 app_shmem_regions :
 {
  __app_shmem_regions_start = .;
  KEEP(*(SORT(.app_regions.*)));
  __app_shmem_regions_end = .;
 } > sdram0 :sdram0_phdr
 k_p4wq_initparam_area : SUBALIGN(4) { _k_p4wq_initparam_list_start = .; KEEP(*(SORT_BY_NAME(._k_p4wq_initparam.static.*))); _k_p4wq_initparam_list_end = .; } > sdram0 :sdram0_phdr
 _static_thread_data_area : SUBALIGN(4) { __static_thread_data_list_start = .; KEEP(*(SORT_BY_NAME(.__static_thread_data.static.*))); __static_thread_data_list_end = .; } > sdram0 :sdram0_phdr
 device_deps :
 {
__device_deps_start = .;
KEEP(*(SORT(.__device_deps_pass2*)));
__device_deps_end = .;
 } > sdram0 :sdram0_phdr
ipm_driver_api_area : SUBALIGN(4) { _ipm_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ipm_driver_api.static.*))); _ipm_driver_api_list_end = .; } > sdram0 :sdram0_phdr
shared_irq_driver_api_area : SUBALIGN(4) { _shared_irq_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._shared_irq_driver_api.static.*))); _shared_irq_driver_api_list_end = .; } > sdram0 :sdram0_phdr
crypto_driver_api_area : SUBALIGN(4) { _crypto_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._crypto_driver_api.static.*))); _crypto_driver_api_list_end = .; } > sdram0 :sdram0_phdr
adc_driver_api_area : SUBALIGN(4) { _adc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._adc_driver_api.static.*))); _adc_driver_api_list_end = .; } > sdram0 :sdram0_phdr
auxdisplay_driver_api_area : SUBALIGN(4) { _auxdisplay_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._auxdisplay_driver_api.static.*))); _auxdisplay_driver_api_list_end = .; } > sdram0 :sdram0_phdr
bbram_driver_api_area : SUBALIGN(4) { _bbram_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bbram_driver_api.static.*))); _bbram_driver_api_list_end = .; } > sdram0 :sdram0_phdr
bt_hci_driver_api_area : SUBALIGN(4) { _bt_hci_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bt_hci_driver_api.static.*))); _bt_hci_driver_api_list_end = .; } > sdram0 :sdram0_phdr
can_driver_api_area : SUBALIGN(4) { _can_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._can_driver_api.static.*))); _can_driver_api_list_end = .; } > sdram0 :sdram0_phdr
cellular_driver_api_area : SUBALIGN(4) { _cellular_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._cellular_driver_api.static.*))); _cellular_driver_api_list_end = .; } > sdram0 :sdram0_phdr
charger_driver_api_area : SUBALIGN(4) { _charger_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._charger_driver_api.static.*))); _charger_driver_api_list_end = .; } > sdram0 :sdram0_phdr
clock_control_driver_api_area : SUBALIGN(4) { _clock_control_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._clock_control_driver_api.static.*))); _clock_control_driver_api_list_end = .; } > sdram0 :sdram0_phdr
comparator_driver_api_area : SUBALIGN(4) { _comparator_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._comparator_driver_api.static.*))); _comparator_driver_api_list_end = .; } > sdram0 :sdram0_phdr
coredump_driver_api_area : SUBALIGN(4) { _coredump_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._coredump_driver_api.static.*))); _coredump_driver_api_list_end = .; } > sdram0 :sdram0_phdr
counter_driver_api_area : SUBALIGN(4) { _counter_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._counter_driver_api.static.*))); _counter_driver_api_list_end = .; } > sdram0 :sdram0_phdr
dac_driver_api_area : SUBALIGN(4) { _dac_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dac_driver_api.static.*))); _dac_driver_api_list_end = .; } > sdram0 :sdram0_phdr
dai_driver_api_area : SUBALIGN(4) { _dai_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dai_driver_api.static.*))); _dai_driver_api_list_end = .; } > sdram0 :sdram0_phdr
display_driver_api_area : SUBALIGN(4) { _display_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._display_driver_api.static.*))); _display_driver_api_list_end = .; } > sdram0 :sdram0_phdr
dma_driver_api_area : SUBALIGN(4) { _dma_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dma_driver_api.static.*))); _dma_driver_api_list_end = .; } > sdram0 :sdram0_phdr
edac_driver_api_area : SUBALIGN(4) { _edac_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._edac_driver_api.static.*))); _edac_driver_api_list_end = .; } > sdram0 :sdram0_phdr
eeprom_driver_api_area : SUBALIGN(4) { _eeprom_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._eeprom_driver_api.static.*))); _eeprom_driver_api_list_end = .; } > sdram0 :sdram0_phdr
emul_bbram_driver_api_area : SUBALIGN(4) { _emul_bbram_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._emul_bbram_driver_api.static.*))); _emul_bbram_driver_api_list_end = .; } > sdram0 :sdram0_phdr
fuel_gauge_emul_driver_api_area : SUBALIGN(4) { _fuel_gauge_emul_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fuel_gauge_emul_driver_api.static.*))); _fuel_gauge_emul_driver_api_list_end = .; } > sdram0 :sdram0_phdr
emul_sensor_driver_api_area : SUBALIGN(4) { _emul_sensor_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._emul_sensor_driver_api.static.*))); _emul_sensor_driver_api_list_end = .; } > sdram0 :sdram0_phdr
entropy_driver_api_area : SUBALIGN(4) { _entropy_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._entropy_driver_api.static.*))); _entropy_driver_api_list_end = .; } > sdram0 :sdram0_phdr
espi_driver_api_area : SUBALIGN(4) { _espi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._espi_driver_api.static.*))); _espi_driver_api_list_end = .; } > sdram0 :sdram0_phdr
espi_saf_driver_api_area : SUBALIGN(4) { _espi_saf_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._espi_saf_driver_api.static.*))); _espi_saf_driver_api_list_end = .; } > sdram0 :sdram0_phdr
flash_driver_api_area : SUBALIGN(4) { _flash_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._flash_driver_api.static.*))); _flash_driver_api_list_end = .; } > sdram0 :sdram0_phdr
fpga_driver_api_area : SUBALIGN(4) { _fpga_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fpga_driver_api.static.*))); _fpga_driver_api_list_end = .; } > sdram0 :sdram0_phdr
fuel_gauge_driver_api_area : SUBALIGN(4) { _fuel_gauge_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fuel_gauge_driver_api.static.*))); _fuel_gauge_driver_api_list_end = .; } > sdram0 :sdram0_phdr
gnss_driver_api_area : SUBALIGN(4) { _gnss_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._gnss_driver_api.static.*))); _gnss_driver_api_list_end = .; } > sdram0 :sdram0_phdr
gpio_driver_api_area : SUBALIGN(4) { _gpio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._gpio_driver_api.static.*))); _gpio_driver_api_list_end = .; } > sdram0 :sdram0_phdr
haptics_driver_api_area : SUBALIGN(4) { _haptics_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._haptics_driver_api.static.*))); _haptics_driver_api_list_end = .; } > sdram0 :sdram0_phdr
hwspinlock_driver_api_area : SUBALIGN(4) { _hwspinlock_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._hwspinlock_driver_api.static.*))); _hwspinlock_driver_api_list_end = .; } > sdram0 :sdram0_phdr
i2c_driver_api_area : SUBALIGN(4) { _i2c_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2c_driver_api.static.*))); _i2c_driver_api_list_end = .; } > sdram0 :sdram0_phdr
i2c_target_driver_api_area : SUBALIGN(4) { _i2c_target_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2c_target_driver_api.static.*))); _i2c_target_driver_api_list_end = .; } > sdram0 :sdram0_phdr
i2s_driver_api_area : SUBALIGN(4) { _i2s_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2s_driver_api.static.*))); _i2s_driver_api_list_end = .; } > sdram0 :sdram0_phdr
i3c_driver_api_area : SUBALIGN(4) { _i3c_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i3c_driver_api.static.*))); _i3c_driver_api_list_end = .; } > sdram0 :sdram0_phdr
kscan_driver_api_area : SUBALIGN(4) { _kscan_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._kscan_driver_api.static.*))); _kscan_driver_api_list_end = .; } > sdram0 :sdram0_phdr
led_driver_api_area : SUBALIGN(4) { _led_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._led_driver_api.static.*))); _led_driver_api_list_end = .; } > sdram0 :sdram0_phdr
led_strip_driver_api_area : SUBALIGN(4) { _led_strip_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._led_strip_driver_api.static.*))); _led_strip_driver_api_list_end = .; } > sdram0 :sdram0_phdr
lora_driver_api_area : SUBALIGN(4) { _lora_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._lora_driver_api.static.*))); _lora_driver_api_list_end = .; } > sdram0 :sdram0_phdr
mbox_driver_api_area : SUBALIGN(4) { _mbox_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mbox_driver_api.static.*))); _mbox_driver_api_list_end = .; } > sdram0 :sdram0_phdr
mdio_driver_api_area : SUBALIGN(4) { _mdio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mdio_driver_api.static.*))); _mdio_driver_api_list_end = .; } > sdram0 :sdram0_phdr
mipi_dbi_driver_api_area : SUBALIGN(4) { _mipi_dbi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mipi_dbi_driver_api.static.*))); _mipi_dbi_driver_api_list_end = .; } > sdram0 :sdram0_phdr
mipi_dsi_driver_api_area : SUBALIGN(4) { _mipi_dsi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mipi_dsi_driver_api.static.*))); _mipi_dsi_driver_api_list_end = .; } > sdram0 :sdram0_phdr
mspi_driver_api_area : SUBALIGN(4) { _mspi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mspi_driver_api.static.*))); _mspi_driver_api_list_end = .; } > sdram0 :sdram0_phdr
peci_driver_api_area : SUBALIGN(4) { _peci_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._peci_driver_api.static.*))); _peci_driver_api_list_end = .; } > sdram0 :sdram0_phdr
ps2_driver_api_area : SUBALIGN(4) { _ps2_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ps2_driver_api.static.*))); _ps2_driver_api_list_end = .; } > sdram0 :sdram0_phdr
ptp_clock_driver_api_area : SUBALIGN(4) { _ptp_clock_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ptp_clock_driver_api.static.*))); _ptp_clock_driver_api_list_end = .; } > sdram0 :sdram0_phdr
pwm_driver_api_area : SUBALIGN(4) { _pwm_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pwm_driver_api.static.*))); _pwm_driver_api_list_end = .; } > sdram0 :sdram0_phdr
regulator_parent_driver_api_area : SUBALIGN(4) { _regulator_parent_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._regulator_parent_driver_api.static.*))); _regulator_parent_driver_api_list_end = .; } > sdram0 :sdram0_phdr
regulator_driver_api_area : SUBALIGN(4) { _regulator_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._regulator_driver_api.static.*))); _regulator_driver_api_list_end = .; } > sdram0 :sdram0_phdr
reset_driver_api_area : SUBALIGN(4) { _reset_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._reset_driver_api.static.*))); _reset_driver_api_list_end = .; } > sdram0 :sdram0_phdr
retained_mem_driver_api_area : SUBALIGN(4) { _retained_mem_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._retained_mem_driver_api.static.*))); _retained_mem_driver_api_list_end = .; } > sdram0 :sdram0_phdr
rtc_driver_api_area : SUBALIGN(4) { _rtc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._rtc_driver_api.static.*))); _rtc_driver_api_list_end = .; } > sdram0 :sdram0_phdr
sdhc_driver_api_area : SUBALIGN(4) { _sdhc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sdhc_driver_api.static.*))); _sdhc_driver_api_list_end = .; } > sdram0 :sdram0_phdr
sensor_driver_api_area : SUBALIGN(4) { _sensor_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sensor_driver_api.static.*))); _sensor_driver_api_list_end = .; } > sdram0 :sdram0_phdr
smbus_driver_api_area : SUBALIGN(4) { _smbus_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._smbus_driver_api.static.*))); _smbus_driver_api_list_end = .; } > sdram0 :sdram0_phdr
spi_driver_api_area : SUBALIGN(4) { _spi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._spi_driver_api.static.*))); _spi_driver_api_list_end = .; } > sdram0 :sdram0_phdr
stepper_driver_api_area : SUBALIGN(4) { _stepper_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._stepper_driver_api.static.*))); _stepper_driver_api_list_end = .; } > sdram0 :sdram0_phdr
syscon_driver_api_area : SUBALIGN(4) { _syscon_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._syscon_driver_api.static.*))); _syscon_driver_api_list_end = .; } > sdram0 :sdram0_phdr
tee_driver_api_area : SUBALIGN(4) { _tee_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tee_driver_api.static.*))); _tee_driver_api_list_end = .; } > sdram0 :sdram0_phdr
video_driver_api_area : SUBALIGN(4) { _video_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._video_driver_api.static.*))); _video_driver_api_list_end = .; } > sdram0 :sdram0_phdr
w1_driver_api_area : SUBALIGN(4) { _w1_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._w1_driver_api.static.*))); _w1_driver_api_list_end = .; } > sdram0 :sdram0_phdr
wdt_driver_api_area : SUBALIGN(4) { _wdt_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._wdt_driver_api.static.*))); _wdt_driver_api_list_end = .; } > sdram0 :sdram0_phdr
can_transceiver_driver_api_area : SUBALIGN(4) { _can_transceiver_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._can_transceiver_driver_api.static.*))); _can_transceiver_driver_api_list_end = .; } > sdram0 :sdram0_phdr
nrf_clock_control_driver_api_area : SUBALIGN(4) { _nrf_clock_control_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._nrf_clock_control_driver_api.static.*))); _nrf_clock_control_driver_api_list_end = .; } > sdram0 :sdram0_phdr
i3c_target_driver_api_area : SUBALIGN(4) { _i3c_target_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i3c_target_driver_api.static.*))); _i3c_target_driver_api_list_end = .; } > sdram0 :sdram0_phdr
its_driver_api_area : SUBALIGN(4) { _its_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._its_driver_api.static.*))); _its_driver_api_list_end = .; } > sdram0 :sdram0_phdr
vtd_driver_api_area : SUBALIGN(4) { _vtd_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._vtd_driver_api.static.*))); _vtd_driver_api_list_end = .; } > sdram0 :sdram0_phdr
tgpio_driver_api_area : SUBALIGN(4) { _tgpio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tgpio_driver_api.static.*))); _tgpio_driver_api_list_end = .; } > sdram0 :sdram0_phdr
pcie_ctrl_driver_api_area : SUBALIGN(4) { _pcie_ctrl_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pcie_ctrl_driver_api.static.*))); _pcie_ctrl_driver_api_list_end = .; } > sdram0 :sdram0_phdr
pcie_ep_driver_api_area : SUBALIGN(4) { _pcie_ep_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pcie_ep_driver_api.static.*))); _pcie_ep_driver_api_list_end = .; } > sdram0 :sdram0_phdr
svc_driver_api_area : SUBALIGN(4) { _svc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._svc_driver_api.static.*))); _svc_driver_api_list_end = .; } > sdram0 :sdram0_phdr
uart_driver_api_area : SUBALIGN(4) { _uart_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._uart_driver_api.static.*))); _uart_driver_api_list_end = .; } > sdram0 :sdram0_phdr
bc12_emul_driver_api_area : SUBALIGN(4) { _bc12_emul_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bc12_emul_driver_api.static.*))); _bc12_emul_driver_api_list_end = .; } > sdram0 :sdram0_phdr
bc12_driver_api_area : SUBALIGN(4) { _bc12_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bc12_driver_api.static.*))); _bc12_driver_api_list_end = .; } > sdram0 :sdram0_phdr
usbc_ppc_driver_api_area : SUBALIGN(4) { _usbc_ppc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._usbc_ppc_driver_api.static.*))); _usbc_ppc_driver_api_list_end = .; } > sdram0 :sdram0_phdr
tcpc_driver_api_area : SUBALIGN(4) { _tcpc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tcpc_driver_api.static.*))); _tcpc_driver_api_list_end = .; } > sdram0 :sdram0_phdr
usbc_vbus_driver_api_area : SUBALIGN(4) { _usbc_vbus_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._usbc_vbus_driver_api.static.*))); _usbc_vbus_driver_api_list_end = .; } > sdram0 :sdram0_phdr
ivshmem_driver_api_area : SUBALIGN(4) { _ivshmem_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ivshmem_driver_api.static.*))); _ivshmem_driver_api_list_end = .; } > sdram0 :sdram0_phdr
ethphy_driver_api_area : SUBALIGN(4) { _ethphy_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ethphy_driver_api.static.*))); _ethphy_driver_api_list_end = .; } > sdram0 :sdram0_phdr
ztest :
{
 _ztest_expected_result_entry_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_expected_result_entry.static.*))); _ztest_expected_result_entry_list_end = .;
 _ztest_suite_node_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_suite_node.static.*))); _ztest_suite_node_list_end = .;
 _ztest_unit_test_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_unit_test.static.*))); _ztest_unit_test_list_end = .;
 _ztest_test_rule_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_test_rule.static.*))); _ztest_test_rule_list_end = .;
} > sdram0 :sdram0_phdr
 init_array :
 {
  KEEP(*(SORT_BY_NAME(".ctors*")))
  KEEP(*(SORT_BY_NAME(".init_array*")))
 } > sdram0 :sdram0_phdr
 ASSERT (SIZEOF(init_array) == 0,
  "GNU-style constructors required but STATIC_INIT_GNU not enabled")
 bt_l2cap_fixed_chan_area : SUBALIGN(4) { _bt_l2cap_fixed_chan_list_start = .; KEEP(*(SORT_BY_NAME(._bt_l2cap_fixed_chan.static.*))); _bt_l2cap_fixed_chan_list_end = .; } > sdram0 :sdram0_phdr
 bt_gatt_service_static_area : SUBALIGN(4) { _bt_gatt_service_static_list_start = .; KEEP(*(SORT_BY_NAME(._bt_gatt_service_static.static.*))); _bt_gatt_service_static_list_end = .; } > sdram0 :sdram0_phdr
 log_strings_area : SUBALIGN(4) { _log_strings_list_start = .; KEEP(*(SORT_BY_NAME(._log_strings.static.*))); _log_strings_list_end = .; } > sdram0 :sdram0_phdr
 log_stmesp_ptr_area : SUBALIGN(4) { _log_stmesp_ptr_list_start = .; KEEP(*(SORT_BY_NAME(._log_stmesp_ptr.static.*))); _log_stmesp_ptr_list_end = .; } > sdram0 :sdram0_phdr
 log_stmesp_str_area : SUBALIGN(4) { _log_stmesp_str_list_start = .; KEEP(*(SORT_BY_NAME(._log_stmesp_str.static.*))); _log_stmesp_str_list_end = .; } > sdram0 :sdram0_phdr
 log_const_area : SUBALIGN(4) { _log_const_list_start = .; KEEP(*(SORT_BY_NAME(._log_const.static.*))); _log_const_list_end = .; } > sdram0 :sdram0_phdr
 log_backend_area : SUBALIGN(4) { _log_backend_list_start = .; KEEP(*(SORT_BY_NAME(._log_backend.static.*))); _log_backend_list_end = .; } > sdram0 :sdram0_phdr
 log_link_area : SUBALIGN(4) { _log_link_list_start = .; KEEP(*(SORT_BY_NAME(._log_link.static.*))); _log_link_list_end = .; } > sdram0 :sdram0_phdr
 tracing_backend_area : SUBALIGN(4) { _tracing_backend_list_start = .; KEEP(*(SORT_BY_NAME(._tracing_backend.static.*))); _tracing_backend_list_end = .; } > sdram0 :sdram0_phdr
 zephyr_dbg_info :
 {
  KEEP(*(".dbg_thread_info"));
 } > sdram0 :sdram0_phdr
 intc_table_area : SUBALIGN(4) { _intc_table_list_start = .; KEEP(*(SORT_BY_NAME(._intc_table.static.*))); _intc_table_list_end = .; } > sdram0 :sdram0_phdr
 symbol_to_keep :
 {
  __symbol_to_keep_start = .;
  KEEP(*(SORT(.symbol_to_keep*)));
  __symbol_to_keep_end = .;
 } > sdram0 :sdram0_phdr
 shell_area : SUBALIGN(4) { _shell_list_start = .; KEEP(*(SORT_BY_NAME(._shell.static.*))); _shell_list_end = .; } > sdram0 :sdram0_phdr
 shell_root_cmds_area : SUBALIGN(4) { _shell_root_cmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_root_cmds.static.*))); _shell_root_cmds_list_end = .; } > sdram0 :sdram0_phdr
 shell_subcmds_area : SUBALIGN(4) { _shell_subcmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_subcmds.static.*))); _shell_subcmds_list_end = .; } > sdram0 :sdram0_phdr
 shell_dynamic_subcmds_area : SUBALIGN(4) { _shell_dynamic_subcmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_dynamic_subcmds.static.*))); _shell_dynamic_subcmds_list_end = .; } > sdram0 :sdram0_phdr
 cfb_font_area : SUBALIGN(4) { _cfb_font_list_start = .; KEEP(*(SORT_BY_NAME(._cfb_font.static.*))); _cfb_font_list_end = .; } > sdram0 :sdram0_phdr
  .fw_ready : ALIGN(4)
  {
    KEEP(*(".fw_ready"));
    KEEP (*(.fw_ready_metadata))
  } >sdram0 :sdram0_phdr
  .noinit : ALIGN(4)
  {
    *(.noinit)
    *(.noinit.*)
  } >sdram0 :sdram0_phdr
  .data : ALIGN(4)
  {
    __data_start = ABSOLUTE(.);
    *(.data)
    *(.data.*)
    *(.gnu.linkonce.d.*)
    KEEP(*(.gnu.linkonce.d.*personality*))
    *(.data1)
    *(.sdata)
    *(.sdata.*)
    *(.gnu.linkonce.s.*)
    *(.sdata2)
    *(.sdata2.*)
    *(.gnu.linkonce.s2.*)
    KEEP(*(.jcr))
    _trace_ctx_start = ABSOLUTE(.);
    *(.trace_ctx)
    _trace_ctx_end = ABSOLUTE(.);
    . = ALIGN(4);
    *(.gna_model)
    __data_end = ABSOLUTE(.);
    . = ALIGN(4096);
  } >sdram0 :sdram0_phdr
  .lit4 : ALIGN(4)
  {
    _lit4_start = ABSOLUTE(.);
    *(*.lit4)
    *(.lit4.*)
    *(.gnu.linkonce.lit4.*)
    _lit4_end = ABSOLUTE(.);
  } >sdram0 :sdram0_phdr
 sw_isr_table :
 {
  . = ALIGN(4);
  *(.gnu.linkonce.sw_isr_table*)
 } > sdram0 :sdram0_phdr
        device_states :
        {
                __device_states_start = .;
  KEEP(*(".z_devstate"));
  KEEP(*(".z_devstate.*"));
                __device_states_end = .;
        } > sdram0 :sdram0_phdr
 log_mpsc_pbuf_area : SUBALIGN(4) { _log_mpsc_pbuf_list_start = .; *(SORT_BY_NAME(._log_mpsc_pbuf.static.*)); _log_mpsc_pbuf_list_end = .; } > sdram0 :sdram0_phdr
 log_msg_ptr_area : SUBALIGN(4) { _log_msg_ptr_list_start = .; KEEP(*(SORT_BY_NAME(._log_msg_ptr.static.*))); _log_msg_ptr_list_end = .; } > sdram0 :sdram0_phdr
 log_dynamic_area : SUBALIGN(4) { _log_dynamic_list_start = .; KEEP(*(SORT_BY_NAME(._log_dynamic.static.*))); _log_dynamic_list_end = .; } > sdram0 :sdram0_phdr
 k_timer_area : SUBALIGN(4) { _k_timer_list_start = .; *(SORT_BY_NAME(._k_timer.static.*)); _k_timer_list_end = .; } > sdram0 :sdram0_phdr
 k_mem_slab_area : SUBALIGN(4) { _k_mem_slab_list_start = .; *(SORT_BY_NAME(._k_mem_slab.static.*)); _k_mem_slab_list_end = .; } > sdram0 :sdram0_phdr
 k_heap_area : SUBALIGN(4) { _k_heap_list_start = .; *(SORT_BY_NAME(._k_heap.static.*)); _k_heap_list_end = .; } > sdram0 :sdram0_phdr
 k_mutex_area : SUBALIGN(4) { _k_mutex_list_start = .; *(SORT_BY_NAME(._k_mutex.static.*)); _k_mutex_list_end = .; } > sdram0 :sdram0_phdr
 k_stack_area : SUBALIGN(4) { _k_stack_list_start = .; *(SORT_BY_NAME(._k_stack.static.*)); _k_stack_list_end = .; } > sdram0 :sdram0_phdr
 k_msgq_area : SUBALIGN(4) { _k_msgq_list_start = .; *(SORT_BY_NAME(._k_msgq.static.*)); _k_msgq_list_end = .; } > sdram0 :sdram0_phdr
 k_mbox_area : SUBALIGN(4) { _k_mbox_list_start = .; *(SORT_BY_NAME(._k_mbox.static.*)); _k_mbox_list_end = .; } > sdram0 :sdram0_phdr
 k_pipe_area : SUBALIGN(4) { _k_pipe_list_start = .; *(SORT_BY_NAME(._k_pipe.static.*)); _k_pipe_list_end = .; } > sdram0 :sdram0_phdr
 k_sem_area : SUBALIGN(4) { _k_sem_list_start = .; *(SORT_BY_NAME(._k_sem.static.*)); _k_sem_list_end = .; } > sdram0 :sdram0_phdr
 k_event_area : SUBALIGN(4) { _k_event_list_start = .; *(SORT_BY_NAME(._k_event.static.*)); _k_event_list_end = .; } > sdram0 :sdram0_phdr
 k_queue_area : SUBALIGN(4) { _k_queue_list_start = .; *(SORT_BY_NAME(._k_queue.static.*)); _k_queue_list_end = .; } > sdram0 :sdram0_phdr
 k_fifo_area : SUBALIGN(4) { _k_fifo_list_start = .; *(SORT_BY_NAME(._k_fifo.static.*)); _k_fifo_list_end = .; } > sdram0 :sdram0_phdr
 k_lifo_area : SUBALIGN(4) { _k_lifo_list_start = .; *(SORT_BY_NAME(._k_lifo.static.*)); _k_lifo_list_end = .; } > sdram0 :sdram0_phdr
 k_condvar_area : SUBALIGN(4) { _k_condvar_list_start = .; *(SORT_BY_NAME(._k_condvar.static.*)); _k_condvar_list_end = .; } > sdram0 :sdram0_phdr
 sys_mem_blocks_ptr_area : SUBALIGN(4) { _sys_mem_blocks_ptr_list_start = .; *(SORT_BY_NAME(._sys_mem_blocks_ptr.static.*)); _sys_mem_blocks_ptr_list_end = .; } > sdram0 :sdram0_phdr
 net_buf_pool_area : SUBALIGN(4) { _net_buf_pool_list_start = .; KEEP(*(SORT_BY_NAME(._net_buf_pool.static.*))); _net_buf_pool_list_end = .; } > sdram0 :sdram0_phdr
  .tm_clone_table : {
    *(.tm_clone_table)
  } >sdram0 :sdram0_phdr
  .bss (NOLOAD) : ALIGN(8)
  {
    . = ALIGN (8);
    _bss_start = ABSOLUTE(.);
    *(.dynsbss)
    *(.sbss)
    *(.sbss.*)
    *(.gnu.linkonce.sb.*)
    *(.scommon)
    *(.sbss2)
    *(.sbss2.*)
    *(.gnu.linkonce.sb2.*)
    *(.dynbss)
    *(.bss)
    *(.bss.*)
    *(.gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN (8);
    _bss_end = ABSOLUTE(.);
  } >sdram0 :sdram0_phdr
  .heap_mem (NOLOAD) : ALIGN(8)
  {
    . = ALIGN (8);
    _heap_mem_start = ABSOLUTE(.);
    *(*.heap_mem)
    _heap_mem_end = ABSOLUTE(.);
  } >sdram1 :sdram1_phdr
  _end = ALIGN (8);
  PROVIDE(end = ALIGN (8));
  _heap_sentry = .;
  __stack = 0x92C00000 + 0x800000;
  .comment 0 : { *(.comment) }
  .debug 0 : { *(.debug) }
  .line 0 : { *(.line) }
  .debug_srcinfo 0 : { *(.debug_srcinfo) }
  .debug_sfnames 0 : { *(.debug_sfnames) }
  .debug_aranges 0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  .debug_info 0 : { *(.debug_info) }
  .debug_abbrev 0 : { *(.debug_abbrev) }
  .debug_line 0 : { *(.debug_line) }
  .debug_frame 0 : { *(.debug_frame) }
  .debug_str 0 : { *(.debug_str) }
  .debug_loc 0 : { *(.debug_loc) }
  .debug_macinfo 0 : { *(.debug_macinfo) }
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames 0 : { *(.debug_varnames) }
  .debug_ranges 0 : { *(.debug_ranges) }
  .debug_addr 0 : { *(.debug_addr) }
  .debug_line_str 0 : { *(.debug_line_str) }
  .debug_loclists 0 : { *(.debug_loclists) }
  .debug_macro 0 : { *(.debug_macro) }
  .debug_names 0 : { *(.debug_names) }
  .debug_rnglists 0 : { *(.debug_rnglists) }
  .debug_str_offsets 0 : { *(.debug_str_offsets) }
  .debug_sup 0 : { *(.debug_sup) }
  .xtensa.info 0 : { *(.xtensa.info) }
  .xt.insn 0 :
  {
    KEEP (*(.xt.insn))
    KEEP (*(.gnu.linkonce.x.*))
  }
  .xt.prop 0 :
  {
    KEEP (*(.xt.prop))
    KEEP (*(.xt.prop.*))
    KEEP (*(.gnu.linkonce.prop.*))
  }
  .xt.lit 0 :
  {
    KEEP (*(.xt.lit))
    KEEP (*(.xt.lit.*))
    KEEP (*(.gnu.linkonce.p.*))
  }
  .xt.profile_range 0 :
  {
    KEEP (*(.xt.profile_range))
    KEEP (*(.gnu.linkonce.profile_range.*))
  }
  .xt.profile_ranges 0 :
  {
    KEEP (*(.xt.profile_ranges))
    KEEP (*(.gnu.linkonce.xt.profile_ranges.*))
  }
  .xt.profile_files 0 :
  {
    KEEP (*(.xt.profile_files))
    KEEP (*(.gnu.linkonce.xt.profile_files.*))
  }
.intList :
{
 KEEP(*(.irq_info*))
 KEEP(*(.intList*))
} > IDT_LIST
  .static_uuid_entries (COPY) : ALIGN(1024)
  {
    *(*.static_uuids)
  } > static_uuid_entries_seg :static_uuid_entries_phdr
  .static_log_entries (COPY) : ALIGN(1024)
  {
    *(*.static_log*)
  } > static_log_entries_seg :static_log_entries_phdr
  .fw_metadata (COPY) : ALIGN(1024)
  {
    KEEP (*(.fw_metadata))
    . = ALIGN(_EXT_MAN_ALIGN_);
  } >fw_metadata_seg :metadata_entries_phdr
.intList :
{
 KEEP(*(.irq_info*))
 KEEP(*(.intList*))
} > IDT_LIST
  /DISCARD/ : { *(.note.GNU-stack) }
}
