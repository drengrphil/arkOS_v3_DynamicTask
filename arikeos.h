#ifndef _ARIKEOS_H
#define _ARIKEOS_H

#include <cstdint>

using namespace std;

typedef bool osflag_type;
typedef void (*function_type)();
typedef uint32_t TaskId_T; // Task ID
typedef int32_t StackType_t;
typedef void (*TaskFunction_t) (void);

class ArikeOS{
	private:
		uint32_t bus_frequency;	        // Microcontroller frequency attribute
		uint32_t round_robin_quanta;	// Hold time qunata for scheduler
		uint32_t millis_prescaler;	    // To divide our clock freq to get appropriate value to sys interrupt
		TaskId_T taskCounter;			// Total task created so far.
	
	public:
		ArikeOS();
		ArikeOS(uint32_t bus_frequency, uint32_t round_robin_qunata);

		/* Facilitators - stack initializer */
		TaskId_T osCreateThread(TaskFunction_t newTaskID, StackType_t taskStackSize);
		void osKernelStackInit(uint32_t i);
		// Add threads to OS
		osflag_type osKernelAddThreads(
				function_type task0,
				function_type task1,
				function_type task2);
		// Initialize the Kernel
		void osKernelInit(void);
		// Launch the kernel
		void osKernelLaunch(void);

		// Mutators (or setters)
		void setOS_sysBusFrequency(uint32_t freq);
		void setOS_rrQuanta(uint32_t quanta);

		// Accessors (or getters)
		uint32_t getOS_sysBusFrequency() const;
		uint32_t getOS_rrQuanta() const;
		uint32_t getOS_millisPrescaler() const;

		~ArikeOS();
};

#endif
