MEMORY
{
  vectors : org = 0x00002000, len = 0x2400
  RAM : org = (((0)) + 0x2400), len = (((16777216)) - 0x2400)
  rom0_seg : org = ((4261412864)), len = ((16384))
  IDT_LIST : org = 0x00000000, len = 0x2000
}
PHDRS
{
  vectors_phdr PT_LOAD;
  rom0_phdr PT_LOAD;
  sram0_phdr PT_LOAD;
  sram0_bss_phdr PT_LOAD;
}
ENTRY("__start")
_rom_store_table = 0;
PROVIDE(_memmap_vecbase_reset = 0x00002000);
PROVIDE(_memmap_reset_vector = 0xFE000000);
_memmap_cacheattr_wb_base = 0x70000007;
_memmap_cacheattr_wt_base = 0xB000000B;
_memmap_cacheattr_bp_base = 0x30000003;
_memmap_cacheattr_unused_mask = 0x0FFFFFF0;
_memmap_cacheattr_wb_strict = 0x7FFFFFF7;
_memmap_cacheattr_wt_strict = 0xBFFFFFFB;
_memmap_cacheattr_bp_strict = 0x3FFFFFF3;
_memmap_cacheattr_wb_allvalid = 0x73333337;
_memmap_cacheattr_wt_allvalid = 0xB333333B;
_memmap_cacheattr_bp_allvalid = 0x33333333;
PROVIDE(_memmap_cacheattr_reset = _memmap_cacheattr_wb_allvalid);
SECTIONS
{
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
/DISCARD/ :
{
 KEEP(*(.irq_info*))
 KEEP(*(.intList*))
}
  .z_xtensa_vectors : ALIGN(1024) {
    z_xtensa_vecbase = .;
    KEEP(*(.WindowVectors.text));
    KEEP(*(.Level2InterruptVector.literal));
    . = 0x180;
    KEEP(*(.Level2InterruptVector.text));
    KEEP(*(.Level3InterruptVector.literal));
    . = 0x1c0;
    KEEP(*(.Level3InterruptVector.text));
    KEEP(*(.Level4InterruptVector.literal));
    . = 0x200;
    KEEP(*(.Level4InterruptVector.text));
    KEEP(*(.Level5InterruptVector.literal));
    . = 0x240;
    KEEP(*(.Level5InterruptVector.text));
    KEEP(*(.DebugExceptionVector.literal));
    . = 0x280;
    KEEP(*(.DebugExceptionVector.text));
    KEEP(*(.NMIExceptionVector.literal));
    . = 0x2c0;
    KEEP(*(.NMIExceptionVector.text));
    KEEP(*(.KernelExceptionVector.literal));
    . = 0x300;
    KEEP(*(.KernelExceptionVector.text));
    KEEP(*(.UserExceptionVector.literal));
    . = 0x340;
    KEEP(*(.UserExceptionVector.text));
    KEEP(*(.DoubleExceptionVector.literal));
    . = 0x3c0;
    KEEP(*(.DoubleExceptionVector.text));
  }
 >vectors :vectors_phdr
  .ResetVector.text : ALIGN(4)
  {
    __rom_region_start = ABSOLUTE(.);
    _ResetVector_text_start = ABSOLUTE(.);
    KEEP (*(.ResetVector.text))
    _ResetVector_text_end = ABSOLUTE(.);
  } >rom0_seg :rom0_phdr
  .text : ALIGN(4)
  {
    _stext = .;
    __text_region_start = .;
    z_mapped_start = .;
    _text_start = ABSOLUTE(.);
    *(.entry.text)
    *(.init.literal)
    *(.iram0.literal .iram.literal .iram.text.literal .iram0.text .iram.text)
    *(.iram1.literal .iram1)
    KEEP(*(.init))
    *(.literal .text .literal.* .text.* .stub .gnu.warning .gnu.linkonce.literal.* .gnu.linkonce.t.*.literal .gnu.linkonce.t.*)
    *(.fini.literal)
    KEEP(*(.fini))
    *(.gnu.version)
    . = ALIGN(4);
    _text_end = ABSOLUTE(.);
    _etext = .;
  } >RAM :sram0_phdr
  __text_region_end = .;
  .rodata : ALIGN(4)
  {
    __rodata_region_start = ABSOLUTE(.);
    *(.rodata)
    *(.rodata.*)
    *(.gnu.linkonce.r.*)
    *(.rodata1)
    . = ALIGN(4);
  } >RAM :sram0_phdr
 initlevel :
 {
  __init_start = .;
  __init_EARLY_start = .; KEEP(*(SORT(.z_init_EARLY_P_?_*))); KEEP(*(SORT(.z_init_EARLY_P_??_*))); KEEP(*(SORT(.z_init_EARLY_P_???_*)));
  __init_PRE_KERNEL_1_start = .; KEEP(*(SORT(.z_init_PRE_KERNEL_1_P_?_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_1_P_??_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_1_P_???_*)));
  __init_PRE_KERNEL_2_start = .; KEEP(*(SORT(.z_init_PRE_KERNEL_2_P_?_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_2_P_??_*))); KEEP(*(SORT(.z_init_PRE_KERNEL_2_P_???_*)));
  __init_POST_KERNEL_start = .; KEEP(*(SORT(.z_init_POST_KERNEL_P_?_*))); KEEP(*(SORT(.z_init_POST_KERNEL_P_??_*))); KEEP(*(SORT(.z_init_POST_KERNEL_P_???_*)));
  __init_APPLICATION_start = .; KEEP(*(SORT(.z_init_APPLICATION_P_?_*))); KEEP(*(SORT(.z_init_APPLICATION_P_??_*))); KEEP(*(SORT(.z_init_APPLICATION_P_???_*)));
  __init_SMP_start = .; KEEP(*(SORT(.z_init_SMP_P_?_*))); KEEP(*(SORT(.z_init_SMP_P_??_*))); KEEP(*(SORT(.z_init_SMP_P_???_*)));
  __init_end = .;
 } > RAM :sram0_phdr
 device_area : SUBALIGN(4) { _device_list_start = .; KEEP(*(SORT(._device.static.*_?_*))); KEEP(*(SORT(._device.static.*_??_*))); KEEP(*(SORT(._device.static.*_???_*))); KEEP(*(SORT(._device.static.*_????_*))); KEEP(*(SORT(._device.static.*_?????_*))); _device_list_end = .; } > RAM :sram0_phdr
 sw_isr_table :
 {
  . = ALIGN(4);
  *(.gnu.linkonce.sw_isr_table*)
 } > RAM :sram0_phdr
 initlevel_error :
 {
  KEEP(*(SORT(.z_init_*)))
 }
 ASSERT(SIZEOF(initlevel_error) == 0, "Undefined initialization levels used.")
 app_shmem_regions :
 {
  __app_shmem_regions_start = .;
  KEEP(*(SORT(.app_regions.*)));
  __app_shmem_regions_end = .;
 } > RAM :sram0_phdr
 k_p4wq_initparam_area : SUBALIGN(4) { _k_p4wq_initparam_list_start = .; KEEP(*(SORT_BY_NAME(._k_p4wq_initparam.static.*))); _k_p4wq_initparam_list_end = .; } > RAM :sram0_phdr
 _static_thread_data_area : SUBALIGN(4) { __static_thread_data_list_start = .; KEEP(*(SORT_BY_NAME(.__static_thread_data.static.*))); __static_thread_data_list_end = .; } > RAM :sram0_phdr
 device_deps :
 {
__device_deps_start = .;
KEEP(*(SORT(.__device_deps_pass2*)));
__device_deps_end = .;
 } > RAM :sram0_phdr
shared_irq_driver_api_area : SUBALIGN(4) { _shared_irq_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._shared_irq_driver_api.static.*))); _shared_irq_driver_api_list_end = .; } > RAM :sram0_phdr
crypto_driver_api_area : SUBALIGN(4) { _crypto_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._crypto_driver_api.static.*))); _crypto_driver_api_list_end = .; } > RAM :sram0_phdr
adc_driver_api_area : SUBALIGN(4) { _adc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._adc_driver_api.static.*))); _adc_driver_api_list_end = .; } > RAM :sram0_phdr
auxdisplay_driver_api_area : SUBALIGN(4) { _auxdisplay_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._auxdisplay_driver_api.static.*))); _auxdisplay_driver_api_list_end = .; } > RAM :sram0_phdr
bbram_driver_api_area : SUBALIGN(4) { _bbram_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bbram_driver_api.static.*))); _bbram_driver_api_list_end = .; } > RAM :sram0_phdr
bt_hci_driver_api_area : SUBALIGN(4) { _bt_hci_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bt_hci_driver_api.static.*))); _bt_hci_driver_api_list_end = .; } > RAM :sram0_phdr
can_driver_api_area : SUBALIGN(4) { _can_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._can_driver_api.static.*))); _can_driver_api_list_end = .; } > RAM :sram0_phdr
cellular_driver_api_area : SUBALIGN(4) { _cellular_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._cellular_driver_api.static.*))); _cellular_driver_api_list_end = .; } > RAM :sram0_phdr
charger_driver_api_area : SUBALIGN(4) { _charger_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._charger_driver_api.static.*))); _charger_driver_api_list_end = .; } > RAM :sram0_phdr
clock_control_driver_api_area : SUBALIGN(4) { _clock_control_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._clock_control_driver_api.static.*))); _clock_control_driver_api_list_end = .; } > RAM :sram0_phdr
comparator_driver_api_area : SUBALIGN(4) { _comparator_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._comparator_driver_api.static.*))); _comparator_driver_api_list_end = .; } > RAM :sram0_phdr
coredump_driver_api_area : SUBALIGN(4) { _coredump_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._coredump_driver_api.static.*))); _coredump_driver_api_list_end = .; } > RAM :sram0_phdr
counter_driver_api_area : SUBALIGN(4) { _counter_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._counter_driver_api.static.*))); _counter_driver_api_list_end = .; } > RAM :sram0_phdr
dac_driver_api_area : SUBALIGN(4) { _dac_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dac_driver_api.static.*))); _dac_driver_api_list_end = .; } > RAM :sram0_phdr
dai_driver_api_area : SUBALIGN(4) { _dai_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dai_driver_api.static.*))); _dai_driver_api_list_end = .; } > RAM :sram0_phdr
display_driver_api_area : SUBALIGN(4) { _display_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._display_driver_api.static.*))); _display_driver_api_list_end = .; } > RAM :sram0_phdr
dma_driver_api_area : SUBALIGN(4) { _dma_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._dma_driver_api.static.*))); _dma_driver_api_list_end = .; } > RAM :sram0_phdr
edac_driver_api_area : SUBALIGN(4) { _edac_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._edac_driver_api.static.*))); _edac_driver_api_list_end = .; } > RAM :sram0_phdr
eeprom_driver_api_area : SUBALIGN(4) { _eeprom_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._eeprom_driver_api.static.*))); _eeprom_driver_api_list_end = .; } > RAM :sram0_phdr
emul_bbram_driver_api_area : SUBALIGN(4) { _emul_bbram_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._emul_bbram_driver_api.static.*))); _emul_bbram_driver_api_list_end = .; } > RAM :sram0_phdr
fuel_gauge_emul_driver_api_area : SUBALIGN(4) { _fuel_gauge_emul_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fuel_gauge_emul_driver_api.static.*))); _fuel_gauge_emul_driver_api_list_end = .; } > RAM :sram0_phdr
emul_sensor_driver_api_area : SUBALIGN(4) { _emul_sensor_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._emul_sensor_driver_api.static.*))); _emul_sensor_driver_api_list_end = .; } > RAM :sram0_phdr
entropy_driver_api_area : SUBALIGN(4) { _entropy_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._entropy_driver_api.static.*))); _entropy_driver_api_list_end = .; } > RAM :sram0_phdr
espi_driver_api_area : SUBALIGN(4) { _espi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._espi_driver_api.static.*))); _espi_driver_api_list_end = .; } > RAM :sram0_phdr
espi_saf_driver_api_area : SUBALIGN(4) { _espi_saf_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._espi_saf_driver_api.static.*))); _espi_saf_driver_api_list_end = .; } > RAM :sram0_phdr
flash_driver_api_area : SUBALIGN(4) { _flash_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._flash_driver_api.static.*))); _flash_driver_api_list_end = .; } > RAM :sram0_phdr
fpga_driver_api_area : SUBALIGN(4) { _fpga_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fpga_driver_api.static.*))); _fpga_driver_api_list_end = .; } > RAM :sram0_phdr
fuel_gauge_driver_api_area : SUBALIGN(4) { _fuel_gauge_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._fuel_gauge_driver_api.static.*))); _fuel_gauge_driver_api_list_end = .; } > RAM :sram0_phdr
gnss_driver_api_area : SUBALIGN(4) { _gnss_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._gnss_driver_api.static.*))); _gnss_driver_api_list_end = .; } > RAM :sram0_phdr
gpio_driver_api_area : SUBALIGN(4) { _gpio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._gpio_driver_api.static.*))); _gpio_driver_api_list_end = .; } > RAM :sram0_phdr
haptics_driver_api_area : SUBALIGN(4) { _haptics_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._haptics_driver_api.static.*))); _haptics_driver_api_list_end = .; } > RAM :sram0_phdr
hwspinlock_driver_api_area : SUBALIGN(4) { _hwspinlock_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._hwspinlock_driver_api.static.*))); _hwspinlock_driver_api_list_end = .; } > RAM :sram0_phdr
i2c_driver_api_area : SUBALIGN(4) { _i2c_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2c_driver_api.static.*))); _i2c_driver_api_list_end = .; } > RAM :sram0_phdr
i2c_target_driver_api_area : SUBALIGN(4) { _i2c_target_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2c_target_driver_api.static.*))); _i2c_target_driver_api_list_end = .; } > RAM :sram0_phdr
i2s_driver_api_area : SUBALIGN(4) { _i2s_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i2s_driver_api.static.*))); _i2s_driver_api_list_end = .; } > RAM :sram0_phdr
i3c_driver_api_area : SUBALIGN(4) { _i3c_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i3c_driver_api.static.*))); _i3c_driver_api_list_end = .; } > RAM :sram0_phdr
ipm_driver_api_area : SUBALIGN(4) { _ipm_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ipm_driver_api.static.*))); _ipm_driver_api_list_end = .; } > RAM :sram0_phdr
led_driver_api_area : SUBALIGN(4) { _led_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._led_driver_api.static.*))); _led_driver_api_list_end = .; } > RAM :sram0_phdr
led_strip_driver_api_area : SUBALIGN(4) { _led_strip_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._led_strip_driver_api.static.*))); _led_strip_driver_api_list_end = .; } > RAM :sram0_phdr
lora_driver_api_area : SUBALIGN(4) { _lora_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._lora_driver_api.static.*))); _lora_driver_api_list_end = .; } > RAM :sram0_phdr
mbox_driver_api_area : SUBALIGN(4) { _mbox_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mbox_driver_api.static.*))); _mbox_driver_api_list_end = .; } > RAM :sram0_phdr
mdio_driver_api_area : SUBALIGN(4) { _mdio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mdio_driver_api.static.*))); _mdio_driver_api_list_end = .; } > RAM :sram0_phdr
mipi_dbi_driver_api_area : SUBALIGN(4) { _mipi_dbi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mipi_dbi_driver_api.static.*))); _mipi_dbi_driver_api_list_end = .; } > RAM :sram0_phdr
mipi_dsi_driver_api_area : SUBALIGN(4) { _mipi_dsi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mipi_dsi_driver_api.static.*))); _mipi_dsi_driver_api_list_end = .; } > RAM :sram0_phdr
mspi_driver_api_area : SUBALIGN(4) { _mspi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._mspi_driver_api.static.*))); _mspi_driver_api_list_end = .; } > RAM :sram0_phdr
peci_driver_api_area : SUBALIGN(4) { _peci_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._peci_driver_api.static.*))); _peci_driver_api_list_end = .; } > RAM :sram0_phdr
ps2_driver_api_area : SUBALIGN(4) { _ps2_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ps2_driver_api.static.*))); _ps2_driver_api_list_end = .; } > RAM :sram0_phdr
ptp_clock_driver_api_area : SUBALIGN(4) { _ptp_clock_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ptp_clock_driver_api.static.*))); _ptp_clock_driver_api_list_end = .; } > RAM :sram0_phdr
pwm_driver_api_area : SUBALIGN(4) { _pwm_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pwm_driver_api.static.*))); _pwm_driver_api_list_end = .; } > RAM :sram0_phdr
regulator_parent_driver_api_area : SUBALIGN(4) { _regulator_parent_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._regulator_parent_driver_api.static.*))); _regulator_parent_driver_api_list_end = .; } > RAM :sram0_phdr
regulator_driver_api_area : SUBALIGN(4) { _regulator_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._regulator_driver_api.static.*))); _regulator_driver_api_list_end = .; } > RAM :sram0_phdr
reset_driver_api_area : SUBALIGN(4) { _reset_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._reset_driver_api.static.*))); _reset_driver_api_list_end = .; } > RAM :sram0_phdr
retained_mem_driver_api_area : SUBALIGN(4) { _retained_mem_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._retained_mem_driver_api.static.*))); _retained_mem_driver_api_list_end = .; } > RAM :sram0_phdr
rtc_driver_api_area : SUBALIGN(4) { _rtc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._rtc_driver_api.static.*))); _rtc_driver_api_list_end = .; } > RAM :sram0_phdr
sdhc_driver_api_area : SUBALIGN(4) { _sdhc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sdhc_driver_api.static.*))); _sdhc_driver_api_list_end = .; } > RAM :sram0_phdr
sensor_driver_api_area : SUBALIGN(4) { _sensor_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._sensor_driver_api.static.*))); _sensor_driver_api_list_end = .; } > RAM :sram0_phdr
smbus_driver_api_area : SUBALIGN(4) { _smbus_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._smbus_driver_api.static.*))); _smbus_driver_api_list_end = .; } > RAM :sram0_phdr
spi_driver_api_area : SUBALIGN(4) { _spi_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._spi_driver_api.static.*))); _spi_driver_api_list_end = .; } > RAM :sram0_phdr
stepper_driver_api_area : SUBALIGN(4) { _stepper_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._stepper_driver_api.static.*))); _stepper_driver_api_list_end = .; } > RAM :sram0_phdr
syscon_driver_api_area : SUBALIGN(4) { _syscon_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._syscon_driver_api.static.*))); _syscon_driver_api_list_end = .; } > RAM :sram0_phdr
tee_driver_api_area : SUBALIGN(4) { _tee_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tee_driver_api.static.*))); _tee_driver_api_list_end = .; } > RAM :sram0_phdr
video_driver_api_area : SUBALIGN(4) { _video_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._video_driver_api.static.*))); _video_driver_api_list_end = .; } > RAM :sram0_phdr
w1_driver_api_area : SUBALIGN(4) { _w1_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._w1_driver_api.static.*))); _w1_driver_api_list_end = .; } > RAM :sram0_phdr
wdt_driver_api_area : SUBALIGN(4) { _wdt_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._wdt_driver_api.static.*))); _wdt_driver_api_list_end = .; } > RAM :sram0_phdr
can_transceiver_driver_api_area : SUBALIGN(4) { _can_transceiver_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._can_transceiver_driver_api.static.*))); _can_transceiver_driver_api_list_end = .; } > RAM :sram0_phdr
nrf_clock_control_driver_api_area : SUBALIGN(4) { _nrf_clock_control_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._nrf_clock_control_driver_api.static.*))); _nrf_clock_control_driver_api_list_end = .; } > RAM :sram0_phdr
i3c_target_driver_api_area : SUBALIGN(4) { _i3c_target_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._i3c_target_driver_api.static.*))); _i3c_target_driver_api_list_end = .; } > RAM :sram0_phdr
its_driver_api_area : SUBALIGN(4) { _its_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._its_driver_api.static.*))); _its_driver_api_list_end = .; } > RAM :sram0_phdr
vtd_driver_api_area : SUBALIGN(4) { _vtd_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._vtd_driver_api.static.*))); _vtd_driver_api_list_end = .; } > RAM :sram0_phdr
tgpio_driver_api_area : SUBALIGN(4) { _tgpio_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tgpio_driver_api.static.*))); _tgpio_driver_api_list_end = .; } > RAM :sram0_phdr
pcie_ctrl_driver_api_area : SUBALIGN(4) { _pcie_ctrl_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pcie_ctrl_driver_api.static.*))); _pcie_ctrl_driver_api_list_end = .; } > RAM :sram0_phdr
pcie_ep_driver_api_area : SUBALIGN(4) { _pcie_ep_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._pcie_ep_driver_api.static.*))); _pcie_ep_driver_api_list_end = .; } > RAM :sram0_phdr
svc_driver_api_area : SUBALIGN(4) { _svc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._svc_driver_api.static.*))); _svc_driver_api_list_end = .; } > RAM :sram0_phdr
uart_driver_api_area : SUBALIGN(4) { _uart_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._uart_driver_api.static.*))); _uart_driver_api_list_end = .; } > RAM :sram0_phdr
bc12_emul_driver_api_area : SUBALIGN(4) { _bc12_emul_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bc12_emul_driver_api.static.*))); _bc12_emul_driver_api_list_end = .; } > RAM :sram0_phdr
bc12_driver_api_area : SUBALIGN(4) { _bc12_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._bc12_driver_api.static.*))); _bc12_driver_api_list_end = .; } > RAM :sram0_phdr
usbc_ppc_driver_api_area : SUBALIGN(4) { _usbc_ppc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._usbc_ppc_driver_api.static.*))); _usbc_ppc_driver_api_list_end = .; } > RAM :sram0_phdr
tcpc_driver_api_area : SUBALIGN(4) { _tcpc_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._tcpc_driver_api.static.*))); _tcpc_driver_api_list_end = .; } > RAM :sram0_phdr
usbc_vbus_driver_api_area : SUBALIGN(4) { _usbc_vbus_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._usbc_vbus_driver_api.static.*))); _usbc_vbus_driver_api_list_end = .; } > RAM :sram0_phdr
ivshmem_driver_api_area : SUBALIGN(4) { _ivshmem_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ivshmem_driver_api.static.*))); _ivshmem_driver_api_list_end = .; } > RAM :sram0_phdr
ethphy_driver_api_area : SUBALIGN(4) { _ethphy_driver_api_list_start = .; KEEP(*(SORT_BY_NAME(._ethphy_driver_api.static.*))); _ethphy_driver_api_list_end = .; } > RAM :sram0_phdr
ztest :
{
 _ztest_expected_result_entry_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_expected_result_entry.static.*))); _ztest_expected_result_entry_list_end = .;
 _ztest_suite_node_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_suite_node.static.*))); _ztest_suite_node_list_end = .;
 _ztest_unit_test_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_unit_test.static.*))); _ztest_unit_test_list_end = .;
 _ztest_test_rule_list_start = .; KEEP(*(SORT_BY_NAME(._ztest_test_rule.static.*))); _ztest_test_rule_list_end = .;
} > RAM :sram0_phdr
 ctors :
 {
  . = ALIGN(4);
  __ZEPHYR_CTOR_LIST__ = .;
  LONG((__ZEPHYR_CTOR_END__ - __ZEPHYR_CTOR_LIST__) / 4 - 2)
  KEEP(*(SORT_BY_NAME(".ctors*")))
  __CTOR_LIST__ = .;
  LONG(0)
  __ZEPHYR_CTOR_END__ = .;
  LONG(0)
  __CTOR_END__ = .;
 } > RAM :sram0_phdr
 init_array :
 {
  . = ALIGN(4);
  __init_array_start = .;
  __init_array_end = .;
  __zephyr_init_array_start = .;
  KEEP(*(SORT_BY_NAME(".init_array*")))
  __zephyr_init_array_end = .;
 } > RAM :sram0_phdr
 bt_l2cap_fixed_chan_area : SUBALIGN(4) { _bt_l2cap_fixed_chan_list_start = .; KEEP(*(SORT_BY_NAME(._bt_l2cap_fixed_chan.static.*))); _bt_l2cap_fixed_chan_list_end = .; } > RAM :sram0_phdr
 bt_gatt_service_static_area : SUBALIGN(4) { _bt_gatt_service_static_list_start = .; KEEP(*(SORT_BY_NAME(._bt_gatt_service_static.static.*))); _bt_gatt_service_static_list_end = .; } > RAM :sram0_phdr
 log_strings_area : SUBALIGN(4) { _log_strings_list_start = .; KEEP(*(SORT_BY_NAME(._log_strings.static.*))); _log_strings_list_end = .; } > RAM :sram0_phdr
 log_stmesp_ptr_area : SUBALIGN(4) { _log_stmesp_ptr_list_start = .; KEEP(*(SORT_BY_NAME(._log_stmesp_ptr.static.*))); _log_stmesp_ptr_list_end = .; } > RAM :sram0_phdr
 log_stmesp_str_area : SUBALIGN(4) { _log_stmesp_str_list_start = .; KEEP(*(SORT_BY_NAME(._log_stmesp_str.static.*))); _log_stmesp_str_list_end = .; } > RAM :sram0_phdr
 log_const_area : SUBALIGN(4) { _log_const_list_start = .; KEEP(*(SORT_BY_NAME(._log_const.static.*))); _log_const_list_end = .; } > RAM :sram0_phdr
 log_backend_area : SUBALIGN(4) { _log_backend_list_start = .; KEEP(*(SORT_BY_NAME(._log_backend.static.*))); _log_backend_list_end = .; } > RAM :sram0_phdr
 log_link_area : SUBALIGN(4) { _log_link_list_start = .; KEEP(*(SORT_BY_NAME(._log_link.static.*))); _log_link_list_end = .; } > RAM :sram0_phdr
 tracing_backend_area : SUBALIGN(4) { _tracing_backend_list_start = .; KEEP(*(SORT_BY_NAME(._tracing_backend.static.*))); _tracing_backend_list_end = .; } > RAM :sram0_phdr
 zephyr_dbg_info :
 {
  KEEP(*(".dbg_thread_info"));
 } > RAM :sram0_phdr
 intc_table_area : SUBALIGN(4) { _intc_table_list_start = .; KEEP(*(SORT_BY_NAME(._intc_table.static.*))); _intc_table_list_end = .; } > RAM :sram0_phdr
 symbol_to_keep :
 {
  __symbol_to_keep_start = .;
  KEEP(*(SORT(.symbol_to_keep*)));
  __symbol_to_keep_end = .;
 } > RAM :sram0_phdr
 shell_area : SUBALIGN(4) { _shell_list_start = .; KEEP(*(SORT_BY_NAME(._shell.static.*))); _shell_list_end = .; } > RAM :sram0_phdr
 shell_root_cmds_area : SUBALIGN(4) { _shell_root_cmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_root_cmds.static.*))); _shell_root_cmds_list_end = .; } > RAM :sram0_phdr
 shell_subcmds_area : SUBALIGN(4) { _shell_subcmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_subcmds.static.*))); _shell_subcmds_list_end = .; } > RAM :sram0_phdr
 shell_dynamic_subcmds_area : SUBALIGN(4) { _shell_dynamic_subcmds_list_start = .; KEEP(*(SORT_BY_NAME(._shell_dynamic_subcmds.static.*))); _shell_dynamic_subcmds_list_end = .; } > RAM :sram0_phdr
 cfb_font_area : SUBALIGN(4) { _cfb_font_list_start = .; KEEP(*(SORT_BY_NAME(._cfb_font.static.*))); _cfb_font_list_end = .; } > RAM :sram0_phdr
 tdata :
 {
  *(.tdata .tdata.* .gnu.linkonce.td.*);
 } > RAM :sram0_phdr
 tbss :
 {
  *(.tbss .tbss.* .gnu.linkonce.tb.* .tcommon);
 } > RAM :sram0_phdr
 PROVIDE(__tdata_start = ADDR(tdata));
 PROVIDE(__tdata_align = ALIGNOF(tdata));
 PROVIDE(__tdata_size = (SIZEOF(tdata) + __tdata_align - 1) & ~(__tdata_align - 1));
 PROVIDE(__tdata_end = __tdata_start + __tdata_size);
 PROVIDE(__tbss_align = ALIGNOF(tbss));
 PROVIDE(__tbss_start = ADDR(tbss));
 PROVIDE(__tbss_size = (SIZEOF(tbss) + __tbss_align - 1) & ~(__tbss_align - 1));
 PROVIDE(__tbss_end = __tbss_start + __tbss_size);
 PROVIDE(__tls_start = __tdata_start);
 PROVIDE(__tls_end = __tbss_end);
 PROVIDE(__tls_size = __tbss_end - __tdata_start);
 .gcc_except_table : ONLY_IF_RO
 {
 *(.gcc_except_table .gcc_except_table.*)
 } > RAM :sram0_phdr
  .rodata_end : ALIGN(4)
  {
    . = ALIGN(4);
    _bss_table_start = ABSOLUTE(.);
    LONG(_bss_start)
    LONG(_bss_end)
    _bss_table_end = ABSOLUTE(.);
    . = ALIGN(4);
    __rodata_region_end = ABSOLUTE(.);
  } >RAM :sram0_phdr
  .data : ALIGN(4)
  {
    _image_ram_start = ABSOLUTE(.);
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
    . = ALIGN(4);
    . = ALIGN(4);
    . = ALIGN(4);
    __data_end = ABSOLUTE(.);
  } >RAM :sram0_phdr
/DISCARD/ :
{
 KEEP(*(.irq_info*))
 KEEP(*(.intList*))
}
        device_states :
        {
                __device_states_start = .;
  KEEP(*(".z_devstate"));
  KEEP(*(".z_devstate.*"));
                __device_states_end = .;
        } > RAM :sram0_phdr
 log_mpsc_pbuf_area : SUBALIGN(4) { _log_mpsc_pbuf_list_start = .; *(SORT_BY_NAME(._log_mpsc_pbuf.static.*)); _log_mpsc_pbuf_list_end = .; } > RAM :sram0_phdr
 log_msg_ptr_area : SUBALIGN(4) { _log_msg_ptr_list_start = .; KEEP(*(SORT_BY_NAME(._log_msg_ptr.static.*))); _log_msg_ptr_list_end = .; } > RAM :sram0_phdr
 log_dynamic_area : SUBALIGN(4) { _log_dynamic_list_start = .; KEEP(*(SORT_BY_NAME(._log_dynamic.static.*))); _log_dynamic_list_end = .; } > RAM :sram0_phdr
 k_timer_area : SUBALIGN(4) { _k_timer_list_start = .; *(SORT_BY_NAME(._k_timer.static.*)); _k_timer_list_end = .; } > RAM :sram0_phdr
 k_mem_slab_area : SUBALIGN(4) { _k_mem_slab_list_start = .; *(SORT_BY_NAME(._k_mem_slab.static.*)); _k_mem_slab_list_end = .; } > RAM :sram0_phdr
 k_heap_area : SUBALIGN(4) { _k_heap_list_start = .; *(SORT_BY_NAME(._k_heap.static.*)); _k_heap_list_end = .; } > RAM :sram0_phdr
 k_mutex_area : SUBALIGN(4) { _k_mutex_list_start = .; *(SORT_BY_NAME(._k_mutex.static.*)); _k_mutex_list_end = .; } > RAM :sram0_phdr
 k_stack_area : SUBALIGN(4) { _k_stack_list_start = .; *(SORT_BY_NAME(._k_stack.static.*)); _k_stack_list_end = .; } > RAM :sram0_phdr
 k_msgq_area : SUBALIGN(4) { _k_msgq_list_start = .; *(SORT_BY_NAME(._k_msgq.static.*)); _k_msgq_list_end = .; } > RAM :sram0_phdr
 k_mbox_area : SUBALIGN(4) { _k_mbox_list_start = .; *(SORT_BY_NAME(._k_mbox.static.*)); _k_mbox_list_end = .; } > RAM :sram0_phdr
 k_pipe_area : SUBALIGN(4) { _k_pipe_list_start = .; *(SORT_BY_NAME(._k_pipe.static.*)); _k_pipe_list_end = .; } > RAM :sram0_phdr
 k_sem_area : SUBALIGN(4) { _k_sem_list_start = .; *(SORT_BY_NAME(._k_sem.static.*)); _k_sem_list_end = .; } > RAM :sram0_phdr
 k_event_area : SUBALIGN(4) { _k_event_list_start = .; *(SORT_BY_NAME(._k_event.static.*)); _k_event_list_end = .; } > RAM :sram0_phdr
 k_queue_area : SUBALIGN(4) { _k_queue_list_start = .; *(SORT_BY_NAME(._k_queue.static.*)); _k_queue_list_end = .; } > RAM :sram0_phdr
 k_fifo_area : SUBALIGN(4) { _k_fifo_list_start = .; *(SORT_BY_NAME(._k_fifo.static.*)); _k_fifo_list_end = .; } > RAM :sram0_phdr
 k_lifo_area : SUBALIGN(4) { _k_lifo_list_start = .; *(SORT_BY_NAME(._k_lifo.static.*)); _k_lifo_list_end = .; } > RAM :sram0_phdr
 k_condvar_area : SUBALIGN(4) { _k_condvar_list_start = .; *(SORT_BY_NAME(._k_condvar.static.*)); _k_condvar_list_end = .; } > RAM :sram0_phdr
 sys_mem_blocks_ptr_area : SUBALIGN(4) { _sys_mem_blocks_ptr_list_start = .; *(SORT_BY_NAME(._sys_mem_blocks_ptr.static.*)); _sys_mem_blocks_ptr_list_end = .; } > RAM :sram0_phdr
 net_buf_pool_area : SUBALIGN(4) { _net_buf_pool_list_start = .; KEEP(*(SORT_BY_NAME(._net_buf_pool.static.*))); _net_buf_pool_list_end = .; } > RAM :sram0_phdr
 .gcc_except_table : ONLY_IF_RW
 {
 *(.gcc_except_table .gcc_except_table.*)
 } > RAM :sram0_phdr
  .bss (NOLOAD) : ALIGN(4)
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
    *(.sram.bss)
    . = ALIGN (8);
    _bss_end = ABSOLUTE(.);
    . = ALIGN(4);
  } >RAM :sram0_bss_phdr
noinit (NOLOAD) :
{
        *(.noinit)
        *(".noinit.*")
} > RAM :sram0_phdr
    .last_ram_section (NOLOAD) :
    {
 _image_ram_end = .;
 _image_ram_size = _image_ram_end - _image_ram_start;
 _end = .;
 z_mapped_end = .;
    } > RAM :sram0_phdr
  _heap_start = .;
  PROVIDE(_heap_sentry = ORIGIN(RAM) + LENGTH(RAM));
  PROVIDE(_heap_end = ORIGIN(RAM) + LENGTH(RAM));
  PROVIDE(__stack = z_interrupt_stacks + 2048);
 .stab 0 : { *(.stab) }
 .stabstr 0 : { *(.stabstr) }
 .stab.excl 0 : { *(.stab.excl) }
 .stab.exclstr 0 : { *(.stab.exclstr) }
 .stab.index 0 : { *(.stab.index) }
 .stab.indexstr 0 : { *(.stab.indexstr) }
 .gnu.build.attributes 0 : { *(.gnu.build.attributes .gnu.build.attributes.*) }
 .comment 0 : { *(.comment) }
 .debug 0 : { *(.debug) }
 .line 0 : { *(.line) }
 .debug_srcinfo 0 : { *(.debug_srcinfo) }
 .debug_sfnames 0 : { *(.debug_sfnames) }
 .debug_aranges 0 : { *(.debug_aranges) }
 .debug_pubnames 0 : { *(.debug_pubnames) }
 .debug_info 0 : { *(.debug_info .gnu.linkonce.wi.*) }
 .debug_abbrev 0 : { *(.debug_abbrev) }
 .debug_line 0 : { *(.debug_line .debug_line.* .debug_line_end ) }
 .debug_frame 0 : { *(.debug_frame) }
 .debug_str 0 : { *(.debug_str) }
 .debug_loc 0 : { *(.debug_loc) }
 .debug_macinfo 0 : { *(.debug_macinfo) }
 .debug_weaknames 0 : { *(.debug_weaknames) }
 .debug_funcnames 0 : { *(.debug_funcnames) }
 .debug_typenames 0 : { *(.debug_typenames) }
 .debug_varnames 0 : { *(.debug_varnames) }
 .debug_pubtypes 0 : { *(.debug_pubtypes) }
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
  .debug.xt.callgraph 0 :
  {
    KEEP (*(.debug.xt.callgraph .debug.xt.callgraph.* .gnu.linkonce.xt.callgraph.*))
  }
}
