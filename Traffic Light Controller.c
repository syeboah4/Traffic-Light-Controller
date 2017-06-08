/*------------------------------------------------------------------------------------------------------------
Name: Seth Yeboah
Date: 11/3/2015
Traffic Light Controller
-------------------------------------------------------------------------------------------------------------*/
//-----------------------------------------------------------------------------
// BIOS header files
// ----------------------------------------------------------------------------
#include "C:\TI\TivaWare_C_Series-2.1.1.71\inc\tm4c123gh6pm.h"
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <xdc/runtime/Log.h>				//needed for any Log_info() call
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles
//----------------------------------------------------------------------------
// TivaWare Header Files
//----------------------------------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
#include "C:\TI\TivaWare_C_Series-2.1.1.71\inc/hw_gpio.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\inc/hw_memmap.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\inc/hw_types.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\driverlib/gpio.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\driverlib/pin_map.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\driverlib/rom.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\driverlib/rom_map.h"
#include "C:\TI\TivaWare_C_Series-2.1.1.71\driverlib/sysctl.h"
#include "pinout.h"
//-----------------------------------------------------------------------------
// Globals
//-----------------------------------------------------------------------------
volatile unsigned long FallingEdges = 0;

//-----------------------------------------------------------------------------
// Prototypes
//-----------------------------------------------------------------------------
void PinoutSet(void);
void PortCInt_Init(void);
void goNorth_Init(void);
void goEast_Init(void);
void GPIOPortC_Handler(void);
void delay(int n);
void NormalSequence(void);
void goNorth(void);
void goEast(void);

//---------------------------------------------------------------------------
// main()
//---------------------------------------------------------------------------
void main(void)
{
	PinoutSet();
	PortCInt_Init();
	goNorth_Init();
	goEast_Init();
	BIOS_start();							// start BIOS scheduler

}


void PinoutSet(void)
{
	// Enable Peripheral Clocks
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	// Configure GPIO Inputs
	MAP_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_4);

	// Configure GPIO Outputs
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_0);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_1);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_1);
	MAP_GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);
}

void delay(int n)
{
	 SysCtlDelay(n);
}

void NormalSequence(void)
{
	PinoutSet();
	int i = 0;
	while(1)
	{
		if(i == 0){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
			delay(30000000); // ~3seconds delay
			i++;
		}
		if(i == 1){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 4); // D is GREEN
			delay(150000000); // ~15seconds delay
			i++;
		}
		if(i == 2){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2); // D is YELLOW
			delay(60000000); // ~6seconds delay
			i++;
		}
		if(i == 3){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
			delay(30000000); //~3seconds delay
			i++;
		}
		if(i == 4){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 4); // B is GREEN
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
			delay(150000000);
			i++;
		}
		if(i == 5){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 2); // B is YELLOW
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
			delay(60000000);
		}
		i = 0;
}

}

//---------------------------------------------------------------------------
// GPIO_portC_init() for Pedestrian Crossing
//---------------------------------------------------------------------------
void PortCInt_Init(void)
{
SYSCTL_RCGC2_R |= 0x00000004;							// (a) activate port C
FallingEdges = 0;										// (b) initialize the counter
GPIO_PORTC_DIR_R &= ~0x10;								// (c) make PC4 input
GPIO_PORTC_DEN_R |=0x10;								// ( ) enable digital pin
GPIO_PORTC_IS_R &= ~0x10;								// (d) PC4 is edge sensitive
GPIO_PORTC_IBE_R &= ~0x10;								// ( ) PC4 not both edge  sensitive
GPIO_PORTC_IEV_R &= ~0x10;								// ( ) PC4 falling edge event
GPIO_PORTC_ICR_R = 0x10;								// (e) Clear flag4
GPIO_PORTC_IM_R |= 0x10;								// (f) ARM Interrupt on PC4
NVIC_PRI0_R = (NVIC_PRI0_R &0xFF00FFFF) |0x00A00000;	// (g) Priority 5
NVIC_EN0_R = 4;											// (h) Enable interrupt 2 in NVIC
}

void North_South_Pedestrian_Interrupt(void)
{
	PinoutSet();
	int i;
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
	GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
	for(i = 0; i < 10; i++)
	{
		GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_3, 0);
		delay(3000000);
		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 8);
		delay(3000000);
	}
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
	GPIO_PORTC_ICR_R |= 0x11; /* clear the interrupt flag before return */
}

//---------------------------------------------------------------------------
// GPIO_PortA Initialization for North-South Traffic
//---------------------------------------------------------------------------
void goNorth_Init(void)
{
	SYSCTL_RCGC2_R |= 0x00000001;							// (a) activate port C
	FallingEdges = 0;										// (b) initialize the counter
	GPIO_PORTA_DIR_R &= ~0x10;								// (c) make PA4 input
	GPIO_PORTA_DEN_R |=0x10;								// ( ) enable digital pin
	GPIO_PORTA_IS_R &= ~0x10;								// (d) PA4 is edge sensitive
	GPIO_PORTA_IBE_R &= ~0x10;								// ( ) PA4 not both edge  sensitive
	GPIO_PORTA_IEV_R &= ~0x10;								// ( ) PA4 falling edge event
	GPIO_PORTA_ICR_R = 0x10;								// (e) Clear flag4
	GPIO_PORTA_IM_R |= 0x10;								// (f) ARM Interrupt on PC4
	NVIC_PRI0_R = (NVIC_PRI0_R &0xFF00FFFF) |0x00A00000;	// (g) Priority 5
	NVIC_EN0_R = 4;
}


void goNorth(void)
{
	PinoutSet();
	int i = 0;
	for(i = 0; i < 3; i++)
	{
		if(i == 0){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
			delay(30000000); // ~3seconds delay
			i++;
		}
		if(i == 1){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_2, 4); // D is GREEN
			delay(50000000); // ~5seconds delay
			i++;
		}
		if(i == 2){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_1, 2); // D is YELLOW
			delay(10000000); // ~3seconds delay
		}
	}
	GPIO_PORTA_ICR_R |= 0x11; /* clear the interrupt flag before return */
}

//---------------------------------------------------------------------------
// GPIO_PortE Initialization for East-West Traffic
//---------------------------------------------------------------------------
void goEast_Init(void)
{
	SYSCTL_RCGC2_R |= 0x10;							// (a) activate port E
	FallingEdges = 0;										// (b) initialize the counter
	GPIO_PORTE_DIR_R &= ~0x10;								// (c) make PE4 input
	GPIO_PORTE_DEN_R |=0x10;								// ( ) enable digital pin
	GPIO_PORTE_IS_R &= ~0x10;								// (d) PE4 is edge sensitive
	GPIO_PORTE_IBE_R &= ~0x10;								// ( ) PE4 not both edge  sensitive
	GPIO_PORTE_IEV_R &= ~0x10;								// ( ) PE4 falling edge event
	GPIO_PORTE_ICR_R = 0x10;								// (e) Clear flag4
	GPIO_PORTE_IM_R |= 0x10;								// (f) ARM Interrupt on PC4
	NVIC_PRI0_R = (NVIC_PRI0_R &0xFF00FFFF) |0x00A00000;	// (g) Priority 5
	NVIC_EN0_R = 4;
}


void goEast(void)
{
	PinoutSet();
	int i = 0;
	for(i = 0; i < 3; i++)
	{
		if(i == 0){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0, 1); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is RED
			delay(30000000); // ~3seconds delay
			i++;
		}
		if(i == 1){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 4); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is GREEN
			delay(50000000); // ~5seconds delay
			i++;
		}
		if(i == 2){
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_1, 2); // B is RED
			GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, 1); // D is YELLOW
			delay(10000000); // ~3seconds delay
		}
	}
	GPIO_PORTE_ICR_R |= 0x11; /* clear the interrupt flag before return */
}
