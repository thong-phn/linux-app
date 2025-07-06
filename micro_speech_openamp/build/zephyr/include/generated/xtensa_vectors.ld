/* Automatically Generated Code - Do Not Edit */
/* See arch/xtensa/core/gen_vector.py */

  .z_xtensa_vectors : ALIGN(1024) {
    z_xtensa_vecbase = .;
    KEEP(*(.WindowVectors.text));
    KEEP(*(.Level2InterruptVector.literal));
    . = 0x17c;
    KEEP(*(.Level2InterruptVector.text));
    KEEP(*(.Level3InterruptVector.literal));
    . = 0x19c;
    KEEP(*(.Level3InterruptVector.text));
    KEEP(*(.DebugExceptionVector.literal));
    . = 0x1bc;
    KEEP(*(.DebugExceptionVector.text));
    KEEP(*(.NMIExceptionVector.literal));
    . = 0x1dc;
    KEEP(*(.NMIExceptionVector.text));
    KEEP(*(.KernelExceptionVector.literal));
    . = 0x1fc;
    KEEP(*(.KernelExceptionVector.text));
    KEEP(*(.UserExceptionVector.literal));
    . = 0x21c;
    KEEP(*(.UserExceptionVector.text));
    KEEP(*(.DoubleExceptionVector.literal));
    . = 0x23c;
    KEEP(*(.DoubleExceptionVector.text));
  }
