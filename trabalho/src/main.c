#include "lpc17xx_pinsel.h"
#include "lpc17xx_i2c.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_ssp.h"
#include "lpc17xx_adc.h"
#include "lpc17xx_timer.h"

#include "light.h"
#include "oled.h"
#include "temp.h"
#include "acc.h"

static uint32_t msTicks = 0;
static uint8_t buf[10];

static uint8_t barPos = 2;

const int RED = 0;
const int GREEN = 1;

static int ledLevel(uint8_t color, uint8_t level) {
	/*
	 * Color: 0 for red, 1 for green
	 * level: 1 to 8, turns leds up in order
	 */

	int r = 0;
	int i = 0;

	if (color == GREEN) { // green
		for (i = 16; i > 16 - level; i--) {
			r |= (1 << i);
		}
	} else { // red
		for (i = 0; i < level; i++) {
			r |= (1 << i);
		}
	}

	return r;
}

static void setBar(uint8_t color, uint8_t percent) {
	int value = ledLevel(color, percent/12);
	pca9532_setLeds(value, 0xffff);
}

static void apagaLed() {
	pca9532_setLeds(0, 0xffff);
}

static void adjustLeds(uint8_t type, int value) {
	uint8_t color, percent;

	if (!type) { // temperatura
		if (value < 150) {
			color = 0;
			percent = 100 - (int)(100.0*(float)value/150.0);
		} else if (value > 285) {
			color = 0;
			percent = 385 - value;		// totalmente arbitrario
		} else {
			color = 1;
			percent = 100;
		}
	} else if (type == 1) { // luminosidade
		color = 1;
		percent = value / 40;
		if (percent > 100) percent = 100;
	} else { // umidade
		if (value < 2000) {
			color = 0;
			percent = 100 - (int)(100.0*(float)value/2000.0);
		} else if (value > 3000) {
			color = 0;
			int diff = 4000 - value;
			if (diff < 0) diff = 0;
			percent = 100 - diff / 10;
		} else {
			color = 1;
			percent = 100;
		}
	}

	setBar(color, percent);
}

static void intToString(int value, uint8_t* pBuf, uint32_t len, uint32_t base) {
	static const char* pAscii = "0123456789abcdefghijklmnopqrstuvwxyz";
	int pos = 0;
	int tmpValue = value;

	// the buffer must not be null and at least have a length of 2 to handle one
	// digit and null-terminator
	if (pBuf == NULL || len < 2) {
		return;
	}

	// a valid base cannot be less than 2 or larger than 36
	// a base value of 2 means binary representation. A value of 1 would mean only zeros
	// a base larger than 36 can only be used if a larger alphabet were used.
	if (base < 2 || base > 36) {
		return;
	}

	// negative value
	if (value < 0) {
		tmpValue = -tmpValue;
		value = -value;
		pBuf[pos++] = '-';
	}

	// calculate the required length of the buffer
	do {
		pos++;
		tmpValue /= base;
	} while (tmpValue > 0);

	if (pos > len) {
		// the len parameter is invalid.
		return;
	}

	pBuf[pos] = '\0';

	do {
		pBuf[--pos] = pAscii[value % base];
		value /= base;
	} while (value > 0);

	return;

}

void SysTick_Handler(void) {
	msTicks++;
}

static uint32_t getTicks(void) {
	return msTicks;
}

static void init_ssp(void) {
	SSP_CFG_Type SSP_ConfigStruct;
	PINSEL_CFG_Type PinCfg;

	/*
	 * Initialize SPI pin connect
	 * P0.7 - SCK;
	 * P0.8 - MISO
	 * P0.9 - MOSI
	 * P2.2 - SSEL - used as GPIO
	 */
	PinCfg.Funcnum = 2;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Portnum = 0;
	PinCfg.Pinnum = 7;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 8;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 9;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Funcnum = 0;
	PinCfg.Portnum = 2;
	PinCfg.Pinnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	SSP_ConfigStructInit(&SSP_ConfigStruct);

	// Initialize SSP peripheral with parameter given in structure above
	SSP_Init(LPC_SSP1, &SSP_ConfigStruct);

	// Enable SSP peripheral
	SSP_Cmd(LPC_SSP1, ENABLE);

}

static void init_i2c(void) {
	PINSEL_CFG_Type PinCfg;

	/* Initialize I2C2 pin connect */
	PinCfg.Funcnum = 2;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);

	// Initialize I2C peripheral
	I2C_Init(LPC_I2C2, 100000);

	/* Enable I2C1 operation */
	I2C_Cmd(LPC_I2C2, ENABLE);
}

static void init_adc(void) {
	PINSEL_CFG_Type PinCfg;

	/*
	 * Init ADC pin connect
	 * AD0.0 on P0.23
	 */
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 23;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);

	/* Configuration for ADC :
	 * 	Frequency at 1Mhz
	 *  ADC channel 0, no Interrupt
	 */
	ADC_Init(LPC_ADC, 1000000);
	ADC_IntConfig(LPC_ADC, ADC_CHANNEL_0, DISABLE);
	ADC_ChannelCmd(LPC_ADC, ADC_CHANNEL_0, ENABLE);
}

uint8_t readBtn() {
	return !((GPIO_ReadValue(0) >> 4) & 0x01);
}

int main(void) {
	GPIO_SetDir(2, 1 << 0, 1);
	GPIO_SetDir(2, 1 << 1, 1);

	GPIO_SetDir(0, 1 << 27, 1);
	GPIO_SetDir(0, 1 << 28, 1);
	GPIO_SetDir(2, 1 << 13, 1);
	GPIO_SetDir(0, 1 << 26, 1);

	GPIO_ClearValue(0, 1 << 27); //LM4811-clk
	GPIO_ClearValue(0, 1 << 28); //LM4811-up/dn
	GPIO_ClearValue(2, 1 << 13); //LM4811-shutdn

	init_i2c();
	init_ssp();
	init_adc();

	oled_init();
	light_init();

	temp_init(&getTicks);

	if (SysTick_Config(SystemCoreClock / 1000)) {
		while (1);  // Capture error
	}

	light_enable();
	light_setRange(LIGHT_RANGE_4000);

	oled_clearScreen(OLED_COLOR_WHITE);

	oled_putString(1, 1, (uint8_t*) "Temp   : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(1, 9, (uint8_t*) "Light  : ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);
	oled_putString(1, 17, (uint8_t*) "Umidade: ", OLED_COLOR_BLACK, OLED_COLOR_WHITE);

	int32_t temperature = 0;
	uint32_t luminosity = 0;
	uint32_t humidity = 0;

	uint8_t isSwitching = 0;
	uint8_t state = 0;

	while (1) {
		uint8_t isBtnPressed = readBtn();

		if (isBtnPressed) {
			if (!isSwitching) {
				isSwitching = 1;

				temperature = 0;
				luminosity = 0;
				humidity = 0;
				apagaLed();

				if (state == 2) {
					state = 0;
				} else {
					state++;
				}
			}
		} else {
			isSwitching = 0;
		}

		if (state == 0) {
			// Temp
			temperature = temp_read();
			adjustLeds(0, temperature);
		} else if (state == 1) {
			// Light
			luminosity = light_read();
			adjustLeds(1, luminosity);
		} else {
			// Humidity
			ADC_StartCmd(LPC_ADC, ADC_START_NOW);
			//Wait conversion complete
			while (!(ADC_ChannelGetStatus(LPC_ADC, ADC_CHANNEL_0, ADC_DATA_DONE)));
			humidity = ADC_ChannelGetData(LPC_ADC, ADC_CHANNEL_0);
			adjustLeds(2, humidity);
		}

		/* output values to OLED display */

		intToString(temperature, buf, 10, 10);
		oled_fillRect((1 + 9 * 6), 1, 80, 8, OLED_COLOR_WHITE);
		oled_putString((1 + 9 * 6), 1, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

		intToString(luminosity, buf, 10, 10);
		oled_fillRect((1 + 9 * 6), 9, 80, 16, OLED_COLOR_WHITE);
		oled_putString((1 + 9 * 6), 9, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

		intToString(humidity, buf, 10, 10);
		oled_fillRect((1 + 9 * 6), 17, 80, 24, OLED_COLOR_WHITE);
		oled_putString((1 + 9 * 6), 17, buf, OLED_COLOR_BLACK, OLED_COLOR_WHITE);

		/* delay */
		Timer0_Wait(100);
	}

}
