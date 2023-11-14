#include "main.h"
#include <stdlib.h>
#include <string.h>


#define MAX_TEMP 		30         					// The maximum temperature for 100% duty cycle
#define MIN_DUTY_CYCLE 	0    						// Minimum duty cycle (0%)
#define MAX_DUTY_CYCLE 	50 							// Maximum duty cycle (100% represented as TIM11->ARR)
#define CCR_TURNON 		35
//FLASH MEMORY
#define FLASH_START_ADDR      	0x08000000 							 			// Start address of Flash memory
#define FLASH_SECTOR_SIZE       0x4000      							 		// Flash memory page size (16 KB for STM32F401RE)
#define FLASH_USER_START_ADDR FLASH_START_ADDR + FLASH_SECTOR_SIZE * 4 			// Example address for user data: SECTOR4
#define DATA_SIZE             	15         							 			// Number of temperature readings to store in memory
#define DATA_READ_SIZE 		  	5
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void config_GPIO(void);
void config_ADC(void);
void config_TIM2(void);
void togglepinA6(void);
void togglepinA7(void);
void TIM11_PWM_Init(void);
void Flash_Init(void);
void WriteToFlash(uint32_t , uint8_t , uint32_t);
void Flash_DeInit(void);
void UART1_sendByte(uint8_t);
void CONFIG_UART1_REG(void);
void CONFIG_UART6_HAL(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef*);
void receiveTemp(void);
void configButton(void);
void debounce(void);
void eraseSector(uint32_t);
void readtoFlash(void);
void config_TIM3(void);
void intToBCD(uint8_t , uint8_t *);

volatile uint32_t adcValue;  // Variable to store the ADC result
volatile uint8_t temperature;
volatile uint8_t TRIGGERING_TEMP = 20;   // The temperature at which the motor starts to spin
volatile uint8_t read_buffer[DATA_READ_SIZE] = {0};

int counter,counter_two;
volatile float CCR_TIM11_PWM;
volatile float dutyCycle;
volatile uint32_t offset_Addr = 0;
UART_HandleTypeDef huart6;
char* welcome_Message = "Welcome to my Embedded System! \n\r";
uint8_t info1[] = "To change triggering temperature type: change \n\r To read 5 of the last 15 readings stored in the internal FLASH press the blue button";
uint8_t rx_indx;
uint8_t rx_data[1];
uint8_t rx_buffer[100];
uint8_t transfer_compl;
int main(void)
{
//CLOCK INITIALIZATION:
  HAL_Init();
  SystemClock_Config();

  config_GPIO();
  config_TIM2();
  CONFIG_UART1_REG();
  CONFIG_UART6_HAL();
  config_ADC();
  TIM11_PWM_Init();
  configButton();
  config_TIM3();

  HAL_UART_Transmit(&huart6, (uint8_t*)welcome_Message, strlen(welcome_Message), 0xFFFF);
  HAL_UART_Transmit(&huart6, (uint8_t*)info1, strlen(info1), 0xFFFF);
  HAL_UART_Receive_IT(&huart6, rx_data, 1); //Receive interrupt

  while (1)
  {
  // we stay here while waiting for the interrupt
  }

}

void configButton(void){
	GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; ///*!< External Interrupt Mode with Falling edge trigger detection         */
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void CONFIG_UART1_REG(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__GPIOA_CLK_ENABLE();
	__USART1_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; 		//Push pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	USART1->CR1 = 0x00;   							// Clear ALL
	USART1->CR1 |= (1<<13);   						// UE = 1... Enable USART
	USART1->CR1 |= (1<<12);   						// M = 0; 8 bit word lenght
	USART1->CR1 &= ~(1<<10); 						//PCE = 0 Parity Control Disabled
	USART1->CR1 &= ~(1<<15); 						//OVER8 = 0: Oversampling by 16
	USART1->CR2 &= ~(0b11 <<12); 					//Number of stop bits set to 1


													//Selecting the Baud Rate: Baud Rate Register (UART_BRR): 9600
	USART1->BRR |= (546 << 4); 						//Mantissa = 546
	USART1->BRR |= (14  << 0); 						//Fraction = 0.875*16 = 14

	USART1->CR1 |= (1<<3);  						// TE=1.. Enable Transmitter
}


void CONFIG_UART6_HAL(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
    __GPIOA_CLK_ENABLE();
    __USART6_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; //Push pull
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);



	huart6.Instance = USART6;
	huart6.Init.BaudRate = 9600;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart6) != HAL_OK)
		  {
		    Error_Handler();
		  }
	NVIC_EnableIRQ(USART6_IRQn);
}

/*
 * I will use the timer11 to generate the PWM signal that will control the sensor spinning.
 * I want a PWM signal of 10 kHz. So being the clock signal configured to 84 MHz. The PWM frequency will be calculated as:
 *
 * 						     Fclock
 * fPWM = 20kHz = ------------------------------
 * 					(PRESCALER + 1)*(ARR + 1)
 *
 * So, fixing the ARR to 400, the PRESCALER value should around 20
 */
void TIM11_PWM_Init(void){

	//1. Enable pheriferal clock sources
	RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;    // Enable TIM11 peripheral clock source


	//3. Set counter mode (we dont do it since timer11 only offers countup)

	//4. Set timing parameters
	TIM11->PSC = 84;     								// Set prescaler, will bring the frequency to 1MHz
	TIM11->ARR = 50;         							// Set auto-reload, will set the freq to 20kHz
	TIM11->CCR1 = MIN_DUTY_CYCLE;    					// Duty cycle: CCR/ARR(%)

														//5. Set PWM mode and enable output compare preload register
	TIM11->CCMR1 |= ( 0x6UL << TIM_CCMR1_OC1M_Pos );    // PWM mode 1
	TIM11->CCMR1 |= TIM_CCMR1_OC1PE;		    		// Enable preload of CCR1

	TIM11->CR1 |= TIM_CR1_ARPE;    						// Enable preload of ARR

	TIM11->CCER &= ~TIM_CCER_CC1P;     					// Set signal polarity
	TIM11->CCER |= TIM_CCER_CC1E;      					// Enable CC

	TIM11->CR1 |= TIM_CR1_URS;    						// Set interrupt source
	//9. Load the register data to the hardware
	TIM11->EGR |= TIM_EGR_UG;    						// Generate an update event
	//12. Optional: Stops the timer when debug is halted
	DBGMCU->APB2FZ |= DBGMCU_APB2_FZ_DBG_TIM11_STOP;
	//We dont enable the timer unless we have the triggering temperature ---> go to ADC handler
}

void config_TIM2(void){
	// Enable the TIM2 clock
	    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	    // Configure TIM2 to generate an update event every 0.05 seconds
	    TIM2->PSC = 99;
	    TIM2->ARR = 42000;
	    // Enable the update interrupt for TIM2
	    TIM2->DIER |= TIM_DIER_UIE;
	    //we will also interrupt
	    NVIC_EnableIRQ(TIM2_IRQn);
	    // Set the priority for TIM2_IRQn
	    NVIC_SetPriority(TIM2_IRQn, 1);
	    // Start TIM2
	    TIM2->CR1 |= TIM_CR1_CEN;
}

void config_TIM3(void){ //TIM3 will interrupt each 5 seconds, and we will store the value of temp
	// Enable the TIM2 clock
	    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	    // Configure TIM3 to generate an update event every 5 seconds
	    TIM3->PSC = 9999;
	    TIM3->ARR = 42000;
	    TIM3->CNT = 0;
	    // Enable the update interrupt for TIM2
	    TIM3->DIER |= TIM_DIER_UIE;
	    //Enable interrupt
	    NVIC_EnableIRQ(TIM3_IRQn);
	    // Set the priority for TIM2_IRQn
	    NVIC_SetPriority(TIM3_IRQn, 0); // Higher priority (lower numerical value)
	    // Start TIM2
	    TIM3->CR1 |= TIM_CR1_CEN;
}

void config_ADC(void){

	//First, we need to enable the clock for the ADC module
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
    // Set the ADC resolution to 12-bit
    ADC1->CR1 &= ~(0b11<<24); //00 in bits 24 and 25
    // Set the channel PA0
    ADC1->SQR3 |= 0x00;
    // Enable the End of Conversion interrupt
    ADC1->CR1 |= ADC_CR1_EOCIE;
    //The ADC will convert each 0.05 seconds (each time TIMER2 triggers), 0b10 for trigger detection on falling edge
    ADC1->CR2 |= (1<<29);
    ADC1->CR2 &= ~(1<<28);
    //External event select: TIMER2 TRG0 event: 0110
    ADC1->CR2 |= (0b11<<25);
    ADC1->CR2 &= ~(0b1001<<24);
    //Now, we enable the ADC, we have to set the ADON(bit0) bit in the ADC_CR2 (ADC Control 2) register:
    ADC1->CR2 |= (1<<0);
    // Enable the ADC interrupt in the NVIC
        NVIC_EnableIRQ(ADC_IRQn);
}

void config_GPIO(void){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  						// Enable GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    					// Enable GPIOB peripheral clock source

//We will use the channel 0 for the analog input (A0)
	GPIOA->MODER |= GPIO_MODER_MODER0;    						// Set PA0 as analog mode, putting 0b11 in the least significant 2 bits (A0)

//---------------We will define PA7 as an output, we will use this to toggle a led each time we store in the flash:----------
	// Configure PA7 as a general-purpose output
	GPIOA->MODER &= ~(GPIO_MODER_MODER7); 						// Clear the bits
	GPIOA->MODER |= GPIO_MODER_MODER7_0;  						// Set the bits to 01 for output mode
	// Enable pull-down resistor for PA7
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR7);  						// Clear the bits
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR7_1;   						// Set the bits to 10 for pull-down


//---------------We will define PA6 as an output, we will use this to toggle a led each time the adc converts:----------
	// Configure PA6 as a general-purpose output
	GPIOA->MODER &= ~(GPIO_MODER_MODER6); 						// Clear the bits
	GPIOA->MODER |= GPIO_MODER_MODER6_0;  						// Set the bits to 01 for output mode
	// Enable pull-down resistor for PA6
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPDR6);  						// Clear the bits
	GPIOA->PUPDR |= GPIO_PUPDR_PUPDR6_1;   						// Set the bits to 10 for pull-down

	//---------------------------------------------------Configure PB9 (TIM11_CH1) for PWM signal generation --------------------------------------------------------
	GPIOB->MODER |= ( 0x2UL << GPIO_MODER_MODER9_Pos );			// PB9 Alternate function
	GPIOB->OSPEEDR |= ( 0x2UL << GPIO_OSPEEDR_OSPEED9_Pos );	// PB9 High speed
	GPIOB->AFR[1] |= ( 0x3UL << 4);								// AF3 for TIM11


}

void UART1_sendByte (uint8_t c){

		/**** STEPS FOLLOWED ******
		1. Write the data to send in the USART_DR register (this clears the TXE bit). Repeat this
			 for each data to be transmitted in case of single buffer.
		2. After writing the last data into the USART_DR register, wait until TC=1. This indicates
			 that the transmission of the last frame is complete. This is required for instance when
			 the USART is disabled or enters the Halt mode to avoid corrupting the last transmission.

		**************/

		USART1->DR = c;   										// LOad the Data
		while (!(USART1->SR & (1<<6)));  						// Wait for TC to SET.. This indicates that the data has been transmitted
}

/*
After reset, write is not allowed in the Flash control register (FLASH_CR) to protect the
Flash memory against possible unwanted operations due, for example, to electric
disturbances. The following sequence is used to unlock this register:
 */
void Flash_Init() {
    // The following values must be programmed consecutively to unlock the FLASH_CR register
	//and allow programming/erasing it
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
}

void Flash_DeInit() {
    FLASH->CR |= FLASH_CR_LOCK;
}

void WriteToFlash(uint32_t flashAddr, uint8_t data, uint32_t dataSize) {
	Flash_Init();
	FLASH->CR |= FLASH_CR_PG; 						// Set the Programming bit
	// Set parallelism to 8 bits. It represents the number of bytes to be programmed each time a write operation occurs to the Flash memory
    FLASH->CR &= ~(1<<9);
    FLASH->CR &= ~(1<<8);

		*(uint8_t*)(flashAddr) = (uint8_t)data; 	// Write data to Flash memory
		flashAddr++; 								//Next byte
		while (FLASH->SR & FLASH_SR_BSY) {} 		// Wait for the Flash to finish writing

	if ((FLASH->SR & FLASH_SR_EOP) != 0) /* (3) */
	{
		FLASH->SR = FLASH_SR_EOP; /* (4) */
	}
	FLASH->CR &= ~FLASH_CR_PG; 						// Clear the Programming bit to lock Flash memory
	Flash_DeInit(); 								//Lock the Flash memory control registers after writing data

}

/*The Flash memory erase operation can be performed at sector level or on the whole Flash
memory (Mass Erase). Mass Erase does not affect the OTP sector or the configuration
sector.*/
void eraseSector(uint32_t sector){

	Flash_Init();
	FLASH_Erase_Sector(sector, FLASH_VOLTAGE_RANGE_1);
	Flash_DeInit();
}

void readtoFlash(void){
	uint32_t address = FLASH_USER_START_ADDR;
	for(int i = 0; i<DATA_READ_SIZE; i++){
		read_buffer[i] = *(__IO uint8_t *) address;
		address++;
	}
	return;
}

void ADC_IRQHandler(void) {
    static uint32_t readingSum = 0;
    static uint8_t readingsCount = 0;
    static uint8_t motor_on_off = 0;
    if (ADC1->SR & ADC_SR_EOC) {  									// Check if End of Conversion flag is set

    	togglepinA6();									 			//toggle pin PA6
        adcValue = ADC1->DR;  										// Read the ADC value and store it in adcValue
        // Starting from the raw value of the ADC output, to obtain the value of voltage we do:
        float voltage = adcValue * 3.3 / 4095;
        // So to finally obtain the temperature value, we must divide this voltage value by the sensitivity of the sensor:
        float temperature_single = voltage / 0.01;
        // Add the temperature to the sum
        readingSum += (uint32_t)temperature_single;
        readingsCount++;
        // If we have 5 readings, calculate the average and proceed
        if (readingsCount >= 30) {
            // Calculate the average temperature
            temperature = (float)readingSum / readingsCount;
            // Reset the counters
            readingSum = 0;
            readingsCount = 0;
            // Map temperature to duty cycle
            CCR_TIM11_PWM = MIN_DUTY_CYCLE; // Default to minimum duty cycle
            if (temperature > TRIGGERING_TEMP) {
                // Ensure temperature is within the desired range
            	if(motor_on_off == 1){
					if (temperature > MAX_TEMP) {
						temperature = MAX_TEMP;
					}
					// Calculate the duty cycle based on temperature
					CCR_TIM11_PWM = (float)(((float)temperature - TRIGGERING_TEMP) / (float)(MAX_TEMP - TRIGGERING_TEMP)) * (float)(MAX_DUTY_CYCLE - MIN_DUTY_CYCLE);

					// Set the new duty cycle for TIM11
					TIM11->CCR1 = CCR_TIM11_PWM;
					//13. Enable timer.
					TIM11->CR1 |= TIM_CR1_CEN;
					dutyCycle = CCR_TIM11_PWM*100/100;

				}else{
					TIM11->CCR1 = CCR_TURNON;
					//13. Enable timer.
					TIM11->CR1 |= TIM_CR1_CEN;
					motor_on_off = 1;
				}
            }else{
            motor_on_off = 0;
            TIM11->CCR1 = 0;
            }
            // We transmit the temperature value via UART protocol
            UART1_sendByte(temperature);
        }
    }
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {  // Check if the update interrupt flag is set
    	//We start conversion of the ADC:
    	ADC1->CR2 |= (1<<30); //SWSTART bit set to one.
        // Clear the update interrupt flag
        TIM2->SR &= ~TIM_SR_UIF;
    }
}

void TIM3_IRQHandler(void){

	if (TIM3->SR & TIM_SR_UIF) {  // Check if the update interrupt flag is set
		//we toggle the pin PA7:
		togglepinA7();
		if(offset_Addr == 0){
			eraseSector(4);
		}
		// Store the average temperature in Memory
		WriteToFlash(FLASH_USER_START_ADDR+offset_Addr, temperature, DATA_SIZE);
		offset_Addr++;
		// If offset_Addr reaches DATA_SIZE, we already wrote 20 values in memory, we need to erase again the sector and start over
		if (offset_Addr >= DATA_SIZE) {
			offset_Addr = 0;
		}
		// Clear the update interrupt flag
		TIM3->SR &= ~TIM_SR_UIF;
	}
}

void togglepinA7(void){

	counter++;
	if(counter%2 == 0){
		GPIOA->BSRR |= GPIO_BSRR_BR7;
	}else{
		GPIOA->BSRR |= GPIO_BSRR_BS7;
	}

}

void togglepinA6(void){

	counter_two++;
	if(counter_two%2 == 0){
		GPIOA->BSRR |= GPIO_BSRR_BR6;
	}else{
		GPIOA->BSRR |= GPIO_BSRR_BS6;
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  char* changeTriggerMessage = "Type the new temperature triggering value (should be less than 100°C): ";
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  uint8_t i;
  if(huart->Instance == USART6){
	  HAL_UART_Transmit(&huart6, rx_data, 1, 0xFFFF);
	  if(rx_indx == 0){
		  for(i=0;i<100;i++){
			  rx_buffer[i]=0;
		  }
	  }
	  if(rx_data[0] != 13){ //If we dont press enter
		  rx_buffer[rx_indx++] = rx_data[0];
	  }else{
		  rx_indx = 0;
		  transfer_compl = 1;
		  HAL_UART_Transmit(&huart6, "\n\r", 2, 100);
		  if(!strcmp(rx_buffer, "change")){
			  HAL_UART_Transmit(&huart6, changeTriggerMessage, strlen(changeTriggerMessage),0xFFFF);
			  receiveTemp();
		  }

	  }
	  HAL_UART_Receive_IT(&huart6, rx_data, 1);
  }
}

void EXTI15_10_IRQHandler(void)
{
	char* message_button = "The first 5 readings of the last 15 are: \n\r";
	uint8_t bcdResult[2];
	uint8_t transmit_read_buffer[2] = {0};

	if(__HAL_GPIO_EXTI_GET_IT(GPIO_PIN_13)){ //We check if the button is the one that triggered the interrupt
	 HAL_UART_Transmit(&huart6, message_button, strlen(message_button), 0xFFFF);

		debounce(); //For debouncing
		readtoFlash();
		for(int i = 0; i< DATA_READ_SIZE; i++){
			intToBCD(read_buffer[i], bcdResult);
			transmit_read_buffer[1] = bcdResult[1] + '0';
			transmit_read_buffer[0] = bcdResult[0] + '0';
			HAL_UART_Transmit(&huart6, transmit_read_buffer, 2, 0xFFFF);
			HAL_UART_Transmit(&huart6, "\n\r", 2, 100);
		}

	 __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_13); //WE CLEAR THE INTERRUPT FLAG
	}
}

void debounce(void){
	for (int i=0 ; i<1000000 ; i++)
		{

		}
}

void intToBCD(uint8_t input, uint8_t *bcdArray) {
    bcdArray[0] = (input / 10) % 10;    // Tens place
    bcdArray[1] = input % 10;          // Ones place
}

void receiveTemp(void){
	uint8_t rx_new_pwd[2];
	int d,u,total,decima,unidad;
	HAL_UART_Receive(&huart6, rx_new_pwd, 2, 0xFFFF); 			//The new temperature is 2 bytes
	HAL_UART_Transmit(&huart6, rx_new_pwd, 2, 0xFFFF);
	decima = rx_new_pwd[0];
	unidad = rx_new_pwd[1];

	d=decima-'0'; 												//decimal tipo char a tipo int
	d=d*10;
	u=unidad-'0'; 												//unidad tipo char a tipo int
	total=d+u; 													// Acá puede ir la variable de temperatura maxima
	TRIGGERING_TEMP = total;
	return;
}

void Error_Handler(void)

{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  //(16 MHz / 16) * 336 / 4 = 84 MHz

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}
