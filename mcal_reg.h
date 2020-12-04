#ifndef __MCAL_REG_H
#define __MCAL_REG_H

#include "reg_access.h"
// #include <cstdint>

using namespace std;

namespace mcal{
	namespace reg{
		const uint32_t scs_base       = 0xE000E000UL;   // System control space
		const uint32_t systick_offset = 0x0010UL;
		const uint32_t systick_base   = scs_base + systick_offset;

		const uint32_t systick_ctrl_offset  = 0x000;  // SysTick Control and Status Register
		const uint32_t systick_load_offset  = 0x004;  // SysTick Reload Value Register
		const uint32_t systick_val_offset   = 0x008;   // SysTick Current Value Register
		const uint32_t systick_calib_offset = 0x00c; // SysTick Calibration Register

		// SysTick registers
		const uint32_t systick_ctrl  = systick_base  + systick_ctrl_offset;
		const uint32_t systick_load  = systick_base  + systick_load_offset;
		const uint32_t systick_val   = systick_base  + systick_val_offset;
		const uint32_t systick_calib = systick_base  + systick_calib_offset;

		typedef struct
		{
			volatile uint32_t CTRL;  // 0x000
			volatile uint32_t LOAD;  // 0x004
			volatile uint32_t VAL;   // 0x008
			volatile uint32_t CALIB; // 0x00c
		} SysTick_Type;
	}
}

// Typecast systick_base address
#define SysTick ((mcal::reg::SysTick_Type*) mcal::reg::systick_base)
// System Handler to set systick priority - using system handler priority register 3
#define SYSPRI3 (*reinterpret_cast<volatile uint32_t *>(0xE000ED20))



#endif
