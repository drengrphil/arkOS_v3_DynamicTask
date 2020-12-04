// Prevent multiple inclusion of the same header file.
#ifndef _REG_ACCESS_H
#define _REG_ACCESS_H

#include <cstdint>

namespace mcal{
	namespace reg{
		// Micro controller register access template
		// addr_type: 32-bit
		// addr: register address
		// val: value to write to address
		// Works on other type of micro controller
		// For 8-bit (addr_type is set to 8-bit) - portability
		template<typename addr_type,
				 typename reg_type,
				 const addr_type addr,
				 const reg_type val>

		class reg_access
		{
		public:
			/* Function that can return a register */
			static reg_type reg_get(){
				// Return content at a particular address.
				return *reinterpret_cast<volatile addr_type*>(addr);
			}

			static void reg_set(){
				// Set the entire register to a value
				*reinterpret_cast<volatile addr_type*>(addr) = val;
			}

			// OR operation
			static void reg_or(){
				*reinterpret_cast<volatile addr_type*>(addr)
						|= static_cast<reg_type>(val);
			}

			// AND operation
			static void reg_and(){
				*reinterpret_cast<volatile addr_type*>(addr)
						&= static_cast<reg_type>(val);
			}

			// NOT operation (disable a bit)
			static void reg_not(){
				*reinterpret_cast<volatile addr_type*>(addr)
						&= static_cast<reg_type>(~val);
			}

			/* Set single-bit in register and not entire register */
			static void bit_set(){
				// Shift by one-bit (1U<<val)
				// 1U - U means unsigned bit
				*reinterpret_cast<volatile addr_type*>(addr)
						|= static_cast<reg_type>(1U<<val);
			}

			/* Clear bits set in register */
			static void bit_clr(){
				*reinterpret_cast<volatile addr_type*>(addr)
						&= static_cast<reg_type>(~static_cast<reg_type>(1U<<val));
			}

			/* Not the single-bit set in register */
			static void bit_not(){
				*reinterpret_cast<volatile addr_type*>(addr)
						^= static_cast<reg_type>(1U<<val);
			}

			/* Get a bit from register */
			static bool bit_get(){
				// val is always the position of the bit
				return ( (reg_get(addr)& static_cast<reg_type>(1U<<val))
						!= static_cast<reg_type>(0U) );
			}
		};
	}
}

#endif
