/* link.ld - the swarm drone's own copy of _startup/c031c6/link.ld
 * FLASH 32K @ 0x08000000, RAM 12K @ 0x20000000
 *
 * The build engine takes a lesson's own link.ld before the shared one.  This
 * copy differs in two numbers, both RAM this firmware does not use:
 *
 *   _Min_Heap_Size  0x200 -> 0     nothing here calls malloc.  The C library's
 *                                  malloc is still LINKED - crt0 -> exit() ->
 *                                  stdio cleanup -> malloc - but main() never
 *                                  returns, so exit() never runs.  (Main.map
 *                                  shows the chain.)
 *   _Min_Stack_Size 0x400 -> 0x300 the main stack serves main() until
 *                                  os_start(), then only the interrupts; each
 *                                  task has its own stack.  main()'s deepest
 *                                  call (agent_init -> say -> fmt) is ~450 B.
 */
ENTRY(Reset_Handler)

MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 32K
  RAM   (rw)  : ORIGIN = 0x20000000, LENGTH = 12K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size  = 0;
_Min_Stack_Size = 0x300;

SECTIONS
{
  .isr_vector : { . = ALIGN(4); KEEP(*(.isr_vector)) . = ALIGN(4); } >FLASH

  .text : {
    . = ALIGN(4);
    *(.text) *(.text*) *(.glue_7) *(.glue_7t) *(.eh_frame)
    KEEP(*(.init)) KEEP(*(.fini))
    . = ALIGN(4);
    _etext = .;
  } >FLASH

  .rodata : { . = ALIGN(4); *(.rodata) *(.rodata*) . = ALIGN(4); } >FLASH

  .ARM.extab : { *(.ARM.extab* .gnu.linkonce.armextab.*) } >FLASH
  .ARM       : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH

  .preinit_array (READONLY) : { PROVIDE_HIDDEN(__preinit_array_start = .); KEEP(*(.preinit_array*)) PROVIDE_HIDDEN(__preinit_array_end = .); } >FLASH
  .init_array    (READONLY) : { PROVIDE_HIDDEN(__init_array_start = .); KEEP(*(SORT(.init_array.*))) KEEP(*(.init_array*)) PROVIDE_HIDDEN(__init_array_end = .); } >FLASH
  .fini_array    (READONLY) : { PROVIDE_HIDDEN(__fini_array_start = .); KEEP(*(SORT(.fini_array.*))) KEEP(*(.fini_array*)) PROVIDE_HIDDEN(__fini_array_end = .); } >FLASH

  /* _sidata is where .data lives in FLASH; startup.c copies from here */
  _sidata = LOADADDR(.data);

  .data : {
    . = ALIGN(4);
    _sdata = .;
    *(.data) *(.data*)
    . = ALIGN(4);
    _edata = .;
  } >RAM AT> FLASH

  . = ALIGN(4);
  .bss : {
    _sbss = .;
    __bss_start__ = _sbss;
    *(.bss) *(.bss*) *(COMMON)
    . = ALIGN(4);
    _ebss = .;
    __bss_end__ = _ebss;
  } >RAM

  ._user_heap_stack : {
    . = ALIGN(8);
    PROVIDE(end = .);
    PROVIDE(_end = .);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM

  .ARM.attributes 0 : { *(.ARM.attributes) }
}
