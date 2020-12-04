#include "arikeos.h"

using namespace std;

#define BUS_FREQ 16000000
#define QUANTA   5 // Run each thread for 5millsec

uint32_t led_task_counter;
uint32_t dsp_task_counter;
uint32_t diagnostic_task_counter;
uint32_t colorsensing_task_counter;
uint32_t soilmoisture_task_counter;

void led_task(void);
void dsp_task(void);
void diagnostic_task(void);
void colorsensing_task(void);
void soilmoisture_task(void);

int main()
{
    ArikeOS sysOS(BUS_FREQ, QUANTA);
    sysOS.osCreateThread(&led_task, 50);
    sysOS.osCreateThread(&dsp_task,50);
    sysOS.osCreateThread(&diagnostic_task, 50);
    sysOS.osCreateThread(&colorsensing_task, 50);
    sysOS.osCreateThread(&soilmoisture_task, 50);
    sysOS.osKernelInit();
    sysOS.osKernelLaunch();
}

void led_task(void){
	// Thread runs until time quanta is up
	while(1){
		// Do some computation
		led_task_counter++;
	}
}

void dsp_task(void){
	while(1){
		// Do some computation
		dsp_task_counter++;
	}
}

void diagnostic_task(void){
	while(1){
		diagnostic_task_counter++;
	}
}

void colorsensing_task(void){
    while(1){
        colorsensing_task_counter++;
    }
}

void soilmoisture_task(void){
    while(1){
        soilmoisture_task_counter++;
    }
}

