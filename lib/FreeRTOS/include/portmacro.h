/* Picks the right Cortex-M port header for the compiler FPU setting. */
#if defined(__ARM_FP)
  #include "portmacro_cm4f.h"
#else
  #include "portmacro_cm3.h"
#endif
