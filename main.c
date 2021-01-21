//Autonomous Robot
//SysCtlDelay(335000);//~25ms

//BIOS Header Files
#include <xdc/std.h>			//Required - For BIOS Types
#include <ti/sysbios/BIOS.h>	//Required - To call BIOS_start()
#include <xdc/runtime/Log.h>	//Log_info() call
#include <xdc/cfg/global.h>		//Header for defined objects/handles

//TivaWare Header Files
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <time.h>
#include <string.h>
#include <math.h>

#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.c"
#include "driverlib/gpio.c"
#include "driverlib/uart.c"
#include "driverlib/uart.h"
#include "driverlib/pwm.h"
#include "driverlib/debug.h"
#include "driverlib/adc.c"
#include "driverlib/adc.h"

#include "inc/hw_ints.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_timer.h"

#include "utils/uartstdio.c"
#include "utils/uartstdio.h"
#include "utils/softuart.c"
#include "utils/softuart.h"

//Functions
void ADC_Init(void);							//ADC
void UART_Init(void);							//Print to Console through UART
float* distanceSensor(float *arr);				//ADC Distance Sensor
void bluetoothInit(void);						//Print to Console using Bluetooth and UART
void motorsInit(void);							//PWM setup for Motors
void task_autonomousRobotControl(void);			//Autonomous Wall Follower
void stringToChar(char *str);					//Converts string to char for UART
void intToChar(int x);							//Converts int to char for UART
void rightMotor(uint8_t dir, float speed);		//Right Motor Control
void leftMotor(uint8_t dir, float speed);		//Left Motor Control
float updatePID(float measuredDis);				//Update PID Controller
void lightSensorInit(void);						//LightSensor Initializer
void task_lightSensor(void);					//LightSensor tape identifier
void stop_HWI(void);							//Stop Robot and all other tasks
void timerInit(void);							//Timer
void timeCount(void);							//Store current time from timer
void collectData(int err);						//Collect error data
void swapBuffer(void);							//Swap the ping-pong buffer
void driveTime(void);							//Calculates total Drive Time in Minutes and Seconds

//Globals
uint32_t frontVal[4] = {0,0,0,0};	//Store Front Distance Sensor Values
uint32_t rightVal[4] = {0,0,0,0};	//Store Right DistancenSensor Values
uint32_t avgFront, avgRight;		//Distance Sensor Value Averages
uint32_t time1 = 0;					//Keeps track of Real Time
int rightDis, frontDis;				//Return Final Distance from Sensor
#define frequency 80000;			//29[kHz] for PWM
unsigned char dataRead;				//input data UART BLUETOOTH
volatile uint16_t period;			//Total square wave ticks
int i, j;							//for loop
int ping[20], pong[20];				//Ping and Pong Buffers
int *collectBuffer, *printBuffer, countBuffer = 20, swapFlag = 0, printFlag = 0;
int checkColor, colorDuration = 0, statusFlag = 0, sendDataFlag = 0;//Light Sensor

//main
void main(void){
	collectBuffer = ping;
	printBuffer = pong;
	ADC_Init();
	UART_Init();
	bluetoothInit();
	motorsInit();
	lightSensorInit();
	timerInit();
	BIOS_start();
}

//Initialize ADC
void ADC_Init(){
	//Set CPU Clock to 40MHz. 400MHz PLL/2 = 200 DIV 5 = 40MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	//Enable the ADC0 module & Port E. Set PE2, PE3 as ADC type
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);

	//Set reference to the internal reference of 3 v
	ADCReferenceSet(ADC0_BASE, ADC_REF_INT);

	//Wait for ADC0 & ADC1 module to be ready.
	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0)){}

	//Sequence Configure
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);

	//Sequence step Configure PE3 CH0 (Right Sensor)
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH0 | ADC_CTL_IE | ADC_CTL_END);

	//Sequence step Configure PE2 CH1 (Front Sensor)
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH1);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 3, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);

	//Enable sequencer
	ADCSequenceEnable(ADC0_BASE, 0);
	ADCSequenceEnable(ADC0_BASE, 1);

	//Enable interrupt
	ADCIntEnable(ADC0_BASE, 1);
}

//Initialize UART
void UART_Init(){
	//Enable Peripheral for USB COM
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//Set pin type for PB0 and PB1 as UART
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0|GPIO_PIN_1);

	//Configure PA0 as U0RX(Receive) and PA1 as U0TX(Transmit)
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);

	//Enable the appropriate module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//Use internal 16MHz clock for UART
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

	//Set UART as console output
	UARTStdioConfig(0, 9600, 16000000);
}

//Initialize BLUETOOTH using UART
void bluetoothInit(void){
	//Enable Peripheral
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	//Set pin type for PB0 and PB1 as UART
	GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0|GPIO_PIN_1);

	//Configure PB0 as U1RX(Receive) and PB1 as U1TX(Transmit)
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);

	//Enable the appropriate module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	//Configure UART
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 9600, UART_CONFIG_WLEN_8|UART_CONFIG_STOP_ONE|UART_CONFIG_PAR_NONE);

	//Enable UART
	UARTEnable(UART1_BASE);
}

//Initialize Motors using PWM
void motorsInit(){
	//Enable peripheral
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

	//Set Pin to GPIO output and configure - PC6 & PC7 as GPIO
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6|GPIO_PIN_7);

	//Set Pin to PWM and configure - PC4 M0PWM6 & PC5 M0PWM7
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5);
	GPIOPinConfigure(GPIO_PC4_M0PWM6);
	GPIOPinConfigure(GPIO_PC5_M0PWM7);

	//Calculate Period
	period = SysCtlPWMClockGet()/frequency;

	//Configure generators and set period, PC4 & PC5 use GEN 3
	PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, period);
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);
	//PWMGenEnable(PWM0_BASE, PWM_GEN_3);
}

//Initialize Light Sensor Requirements
void lightSensorInit(){
	//For Light Sensor and LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	//LED setup - 2=RED, 4=BLUE, 8=GREEN
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

	//Set interrupt for RED LED to STOP the robot
	GPIOIntTypeSet(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_HIGH_LEVEL);
	GPIOIntEnable(GPIO_PORTF_BASE, GPIO_INT_PIN_1);
}

//Initialize the Timer
void timerInit(void){
	//Enable Timer 2
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);

	//Configure Timer 2: CFG mode = Periodic
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);

	//Period
	TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet());

	//Enable the Timer Interrupt
	TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);

	//Enable the Timer
	TimerEnable(TIMER2_BASE, TIMER_A);
}

//Distance Sensor Code
float* distanceSensor(float *arr){
	// Trigger the sample sequence.
	ADCProcessorTrigger(ADC0_BASE, 0);
	ADCProcessorTrigger(ADC0_BASE, 1);

	// Wait until interrupt is triggered
	while(!ADCIntStatus(ADC0_BASE, 0, false)){}
	while(!ADCIntStatus(ADC0_BASE, 1, false)){}

	//Read the value from the ADC and store it in the address of variable Value
	ADCSequenceDataGet(ADC0_BASE, 0, rightVal);
	ADCSequenceDataGet(ADC0_BASE, 1, frontVal);

	//Average the values
	avgRight = (rightVal[0] + rightVal[1] + rightVal[2] + rightVal[3]) / 4;
	avgFront = (frontVal[0] + frontVal[1] + frontVal[2] + frontVal[3]) / 4;

	//Convert the ADC averages to voltage values
	float voltageRight = avgRight * 0.0007324219;
	float voltageFront = avgFront * 0.0007324219;

	//Calculate the Distance in centimeters Right Sensor Formula: from data-sheet graph
	if(avgRight < 3750 && avgRight > 1100)
		rightDis = 13 * pow(voltageRight, -1) - 1;
	else if(avgRight < 1099 && avgRight > 850)
		rightDis = 13 * pow(voltageRight, -1) - 2;
	else if(avgRight < 849 && avgRight > 650)
		rightDis = 13 * pow(voltageRight, -1) - 4;
	else if(avgRight < 649 && avgRight > 490)
		rightDis = 13 * pow(voltageRight, -1) - 5;
	else if(avgRight < 489)
		rightDis = 30;

	//Calculate the Distance in centimeters Front Sensor Formula: from data-sheet graph
	if(avgFront < 3300 && avgFront > 870)
		frontDis = 13 * pow(voltageFront, -1) - 1.5;
	else if(avgFront < 869 && avgFront > 630)
		frontDis = 13 * pow(voltageFront, -1) - 1;
	else if(avgFront < 629 && avgFront > 550)
		frontDis = 13 * pow(voltageFront, -1);
	else if(avgFront < 549)
		frontDis = 30;

	//Clear Interrupt
	ADCIntClear(ADC0_BASE, 0);
	ADCIntClear(ADC0_BASE, 1);

	//Return Distance
	arr[0] = rightDis;
	arr[1] = frontDis;
	return arr;
}

//Autonomous Robot Control with PID
void task_autonomousRobotControl(void){
	while(1){
		//Variables
		float rightPIDval, arr[2];
		float *measuredDis = distanceSensor(arr);
		//char measured1[8], pid1[8];

		//convert distance float to string and print to console
		//sprintf(measured1, "%0.2f", measuredDis[0]);
		//UARTprintf("Right Dis: %s ", measured1);

		//PID values for right sensor
		rightPIDval = updatePID(measuredDis[0]);

		//convert PID float to string and print to console
		//sprintf(pid1, "%0.2f", rightPIDval);
		//UARTprintf("PID Right: %s\n", pid1);

		if(measuredDis[0] < 25 && measuredDis[1] > 6){//wall follow
			if(rightPIDval < 0){//far from wall
				rightMotor(0x00, 0.9);
				leftMotor(GPIO_PIN_7, 1.0);
			}
			else if(rightPIDval > 0){//close to wall
				rightMotor(0x00, 1.0);
				leftMotor(GPIO_PIN_7, 0.9);
			}
			else{//at set point
				rightMotor(0x00, 1.0);
				leftMotor(GPIO_PIN_7, 1.0);
			}
		}
		else if(measuredDis[0] >= 25 && measuredDis[1] > 6){//right turn
			SysCtlDelay(3350000);//~250ms
			rightMotor(0x00, 0.1);
			leftMotor(GPIO_PIN_7, 1.0);
			SysCtlDelay(6700000);//~500ms
		}
		else if(measuredDis[1] <= 6){//U turn
			rightMotor(0x00, 0.9);
			leftMotor(0x00, 0.9);
			SysCtlDelay(3685000);//~275ms
		}

		//Send semaphore for lightSensor task
		Semaphore_post(semi);
	}
}

//Convert string array to char
void stringToChar(char *str){
    for(i = 0; i < strlen(str); i++)
    	UARTCharPut(UART1_BASE, str[i]);
}

//Convert an int to char
void intToChar(int x){
	//Variables
	int digit[2] = {0, 0}, i = 0, flag = 0;

		//Check if integer is negative
	    if(x < 0){
	        flag = 1;
	        x = x * -1;
	    }

	    //split the integers and store in array
	    while( x > 0) {
	        digit [i++] = x % 10;
	        x = x / 10;
	    }

	    //Add negative symbol if number is negative
	    if(flag == 1)
	    	stringToChar("-");

	    //Print array contents and delete leading zeroes
	    for(i = 1; i >= 0; i--){
	        if(i == 1){
	            if(digit[1] != 0)
	            	UARTCharPut(UART1_BASE, digit[i] + '0');
	        }
	        else
	        	UARTCharPut(UART1_BASE, digit[i] + '0');
	    }

	    //print carriage return and new line at the end
	    //stringToChar("\r\n");
}

//Set speed & Direction Right Motor - [Back = GPIO_PIN_6, Forward = 0x00]
void rightMotor(uint8_t dir, float speed){
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, (period * speed));
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, dir);
}

//Set speed & Direction Left Motor - [Back = 0x00, Forward = GPIO_PIN_7]
void leftMotor(uint8_t dir, float speed){
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, (period * speed));
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, dir);
}

//Update the PID value with new Distance measurement
float updatePID(float measuredDis){
	//Gains
	const float Kp = 3.0, Ki = 4.0, Kd = 4.0;
	//Anti wind up limits
	const float minSys = -22.0, maxSys = 3.0;
	//Variables
	float error, proportional, derivative, setPoint = 8.0;
	static float integral = 0.0;
	static float finalPID = 0.0;
	static float prevError = 0.0;

	//Error Signal
	error = setPoint - measuredDis;

	//collect data for ping-pong
	if(sendDataFlag)
		collectData(error);

	//Proportional
	proportional = Kp * error;

	//Integral
	integral = integral + (Ki * error);

	//Anti wind up for integral
	float min, max;
	if(maxSys > proportional)//max setup
		max = maxSys - proportional;
	else
		max = 0.0;
	if(minSys < proportional)//min setup
		min = minSys - proportional;
	else
		min = 0.0;

	//Secure Integral
	if(integral > max)
		integral = max;
	if(integral < min)
		integral = min;

	//Derivative
	derivative = (error - prevError) * Kd;

	//Final PID value
	finalPID = proportional + integral + derivative;

	//limit the output for the PID controller
	//if(outPID > maxSys)
	//	outPID = maxSys;
	//else if(outPID < minSys)
		//outPID = minSys;

	//Store error for next iteration
	prevError = error;

	//Delay
	SysCtlDelay(200000);//15ms

	//return PID value
	return finalPID;
}

//Run the sensor and detect the tape
void task_lightSensor(void){
	while(1){
		//Receive semaphore from autonomous task
		Semaphore_pend(semi, BIOS_WAIT_FOREVER);

		//Charge the Sensor's Capacitor
		GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_2);
		GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_PIN_2);
		SysCtlDelay(1000);

		//Set pin to read
		GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);

		//Decay checker
		checkColor = 0;

		//Decay Loop
		while (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_2) == GPIO_PIN_2)
			checkColor += 1;

		//current color identifier
		(checkColor >= 3000) ? (colorDuration += 1) : (colorDuration = 0);

		//UARTprintf("checkColor %d ", checkColor);
		//UARTprintf("colorDuration %d\n", colorDuration);

		//Start
		if(colorDuration > 0 && colorDuration <= 1){//Single Black Tape
			if(statusFlag == 0){
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);//Blue
				sendDataFlag = statusFlag = 1;
			}
			else if(statusFlag == 2){
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);//Blue
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);//Green
				sendDataFlag = 0;
				statusFlag = 1;
			}
		}
		else if(colorDuration > 5){//Double Black Tape
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);//Blue
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);//Green
			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);//Red
		}
		else
			(sendDataFlag == 1) ? (statusFlag = 2) : (statusFlag = 0);

		//Print Pin-Pong Buffer if senDataFlag is true and printing is available
		if(sendDataFlag && printFlag){
			stringToChar(":");
			intToChar(printBuffer[countBuffer]);
			stringToChar("0\r\n");
		}
	}
}

//Stop robot 0% dutyCycle to motors
void stop_HWI(void){
	GPIOIntClear(GPIO_PORTF_BASE, GPIO_INT_PIN_1);
	stringToChar("STOP\r\n");
	sendDataFlag = 0;
	rightMotor(0x00, 0.1);
	leftMotor(GPIO_PIN_7, 0.1);
	SysCtlDelay(1000);
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT, false);
	driveTime();
	while(1){}
}

//Keep track of real time using Timer
void timeCount(void){
	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT);
	time1++;
}

//Collect error from PID to send to UART
void collectData(int err){
	if(!countBuffer){
		//Software Interrupt to swap buffers
		Swi_post(swap);
	}
	if(countBuffer)
		collectBuffer[--countBuffer] = err;
}

//Swap the buffers ping-pong style
void swapBuffer(void){
	if(swapFlag == 0){
		collectBuffer = pong;
		printBuffer = ping;
		swapFlag = printFlag = 1;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);//Green
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);//Blue
	}
	else if(swapFlag == 1){
		collectBuffer = ping;
		printBuffer = pong;
		swapFlag = 0;
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);//Blue
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);//Green
	}
	countBuffer = 20;
}

//Calculates total drive time and prints
void driveTime(void){
	uint32_t min = time1 /60;
	uint32_t sec = time1 % 60;

	stringToChar("Final Time: ");
	intToChar(min);
	stringToChar(" min ");
	intToChar(sec);
	stringToChar(" sec\r\n");
}
