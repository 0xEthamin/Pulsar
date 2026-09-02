/* STM32H743VIT6 memory map.
 *
 * Sizes and base addresses come from RM0433 section 2.4 and from the flash
 * description of section 4. Table 7 of the datasheet describes the STM32H742
 * instead, with 384 Kbytes of AXI SRAM and 32 of SRAM1, and does not apply
 * here.
 *
 * STACK and RAM are the two halves of the DTCM, which the Cortex-M7 reaches
 * with no wait state and no cache in the way. That is what the audio path and
 * the interrupt stack need.
 *
 * The call stack owns STACK alone and grows down from the boundary at
 * 0x20004000, away from the statics that RAM holds, so an overflow leaves
 * `.data` and `.bss` untouched. That much the split delivers on its own, and
 * statics that outgrow RAM fail the link.
 *
 * What lies under the overflow is weaker. Table 7 of RM0433, the memory map,
 * marks 0x1FF20000 to 0x1FFFFFFF, immediately below STACK, as Reserved, and
 * attaches no access behaviour to it. The one bus error RM0433 states for a
 * reserved area is footnote 1 of its table 8, which covers the peripheral
 * region. A store below STACK is therefore EXPECTED to pend a bus fault, not
 * known to.
 *
 * DMA buffers do NOT belong in RAM. RM0433 section 2.1.6 gives DMA1 and DMA2
 * every internal memory except ITCM and DTCM, which only the MDMA reaches
 * through the AHBS port of the CPU (section 2.1.2). A transfer buffer in RAM
 * reads as zeros, with no diagnostic.
 *
 * BDMA is confined to D3, where it reaches SRAM4, the backup RAM and the AHB4
 * and APB4 peripherals. Section 2.4 gives AXI SRAM, SRAM1, SRAM2 and SRAM3 to
 * every system master EXCEPT BDMA, so a BDMA buffer belongs in `.sram4` and
 * nowhere else declared here. SAI4 and LPUART1 sit in D3 and are served by
 * BDMA, so the case is reachable on this board.
 *
 * The SECTIONS block below makes the placement explicit, and a region that
 * overflows fails the build. Without it an annotated buffer becomes an orphan
 * section that the linker drops next to RAM.
 */

MEMORY
{
  FLASH   (rx)  : ORIGIN = 0x08000000, LENGTH = 2048K
  ITCM    (rx)  : ORIGIN = 0x00000000, LENGTH = 64K
  STACK   (rw)  : ORIGIN = 0x20000000, LENGTH = 16K
  RAM     (rw)  : ORIGIN = 0x20004000, LENGTH = 112K
  AXISRAM (rw)  : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM1   (rw)  : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2   (rw)  : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3   (rw)  : ORIGIN = 0x30040000, LENGTH = 32K
  SRAM4   (rw)  : ORIGIN = 0x38000000, LENGTH = 64K
  BKPSRAM (rw)  : ORIGIN = 0x38800000, LENGTH = 4K
}

/* Word 0 of the vector table takes `_stack_start` as the initial main stack
 * pointer, and link.x defaults it to the end of RAM with `_stack_end` after the
 * statics. Both defaults put the stack and the statics in one region, so both
 * are overridden here. The ASSERT of link.x requires `_stack_start` at or above
 * `_stack_end`, which the default `_stack_end` breaks the moment `.bss` extends
 * past the top of STACK.
 */
_stack_start = ORIGIN(STACK) + LENGTH(STACK);
_stack_end = ORIGIN(STACK);

/* Uninitialised placement for the memories outside the TCMs. Reset leaves their
 * contents undefined, so anything landing here is zeroed or filled by the code
 * that owns it, never by the startup sequence.
 *
 * ITCM carries no section. Code placed in it has to be copied from flash at
 * startup, which needs a load region and a copy loop, not a placement.
 */
SECTIONS
{
  .axisram (NOLOAD) : ALIGN(8)
  {
    *(.axisram .axisram.*);
    . = ALIGN(8);
  } > AXISRAM

  .sram1 (NOLOAD) : ALIGN(4)
  {
    *(.sram1 .sram1.*);
    . = ALIGN(4);
  } > SRAM1

  .sram2 (NOLOAD) : ALIGN(4)
  {
    *(.sram2 .sram2.*);
    . = ALIGN(4);
  } > SRAM2

  .sram3 (NOLOAD) : ALIGN(4)
  {
    *(.sram3 .sram3.*);
    . = ALIGN(4);
  } > SRAM3

  .sram4 (NOLOAD) : ALIGN(4)
  {
    *(.sram4 .sram4.*);
    . = ALIGN(4);
  } > SRAM4

  .bkpsram (NOLOAD) : ALIGN(4)
  {
    *(.bkpsram .bkpsram.*);
    . = ALIGN(4);
  } > BKPSRAM
} INSERT AFTER .got;

/* The anchor is `.got` and not `.bss`. link.x assigns `__ebss` in a statement
 * of its own after the `.bss` output region, so a block inserted after `.bss`
 * lands between the two and carries `__ebss` to the end of the last inserted
 * region.
 * The startup code zero-fills from `__sbss` to `__ebss`, which then walks off
 * the DTCM and the board never reaches main. Nothing follows `.got`, so the
 * symbols keep their values.
 */
