#include "Arduino.h"
#include <SPI.h>
#include <stdio.h>

#define IIM42352
#define IIM42352_CS  4
#define IIM42352_MOSI 23
#define IIM42352_MISO 19
#define IIM42352_CLK 18
#define IIM42352_INT1 13
#define IIM42352_INT2 15
#define LOG_UART_SPEED 921600
#define SPI_BUFFER_SIZE 2080

//#define DEBUG_REGISTERS
//#define DEBUG_SPI_WRITE
//#define DEBUG_SPI_READ

#include "example-high-rate-logger.h"

/* InvenSense utils */
#include "EmbUtils/Message.h"
#include "EmbUtils/ErrorHelper.h"
#include "EmbUtils/RingBuffer.h"

/*
 * Select communication link between ESP32 and ICM42352
 */
#define SERIF_TYPE IIM423XX_UI_SPI4

/*
 * Define msg level
 */
#define MSG_LEVEL INV_MSG_LEVEL_OFF

/* TODO: Move that somewhere else */
#ifndef TO_MASK
#define TO_MASK(a) (1U << (unsigned)(a))
#endif

/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

uint8_t sendBuffer[SPI_BUFFER_SIZE];
uint8_t recvBuffer[SPI_BUFFER_SIZE];

uint8_t out_str[FRAME_SIZE_MAX];
uint8_t payload_size_bytes = SENSOR_1AXIS_SIZE;
uint8_t data_address = MPUREG_ACCEL_DATA_X0_UI;
struct inv_iim423xx_serif iim423xx_serif;

/* Flag set from iim423xx device irq handler */
static volatile int irq_from_device;

/* Just a handy variable to handle the iim423xx object */
static struct inv_iim423xx icm_driver;

/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static void SetupMCUHardware(struct inv_iim423xx_serif *icm_serif);
void ext_interrupt_cb(void *context, unsigned int int_num);
static void check_rc(int rc, const char *msg_context);
void msg_printer(int level, const char *str, va_list ap);


/* --------------------------------------------------------------------------------------
 *  Setup: called once at startup of the sketch
 * -------------------------------------------------------------------------------------- */

void setup() {
	int rc = 0;

	/* Initialize MCU hardware */
	SetupMCUHardware(&iim423xx_serif);
//	Serial.println("SetupMCUHardware DONE!");

	/* Initialize Iim423xx */
	rc = SetupInvDevice(&iim423xx_serif);
	check_rc(rc, "error while setting up INV device");
//	Serial.println("SetupInvDevice DONE!");

	/* Configure Iim423xx */
	rc = ConfigureInvDevice();
	check_rc(rc, "error while configuring INV device");
//	Serial.println("ConfigureInvDevice DONE!");

	/* Frame Header, contains sensor odr information,
	 to be decoded properly by python script */
	out_str[HEADER_LSB_IDX] = HEADER_LSB;
	out_str[HEADER_MSB_IDX] = HEADER_MSB | SENSOR_ODR;

	/* Frame counter index reset*/
	out_str[FRAME_CNT_IDX] = 0;

	if (SENSOR_ODR == HIGH_RATE_32KHZ) {

		payload_size_bytes = SENSOR_1AXIS_SIZE;
		switch (AXIS_TO_LOG) {
		case ACCEL_X_AXIS:
			data_address = MPUREG_ACCEL_DATA_X0_UI;
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Start streaming: Raw Accel x-axis at 32 KHz");
			break;

		case ACCEL_Y_AXIS:
			data_address = MPUREG_ACCEL_DATA_X0_UI + 2;
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Start streaming: Raw Accel y-axis at 32 KHz");
			break;

		case ACCEL_Z_AXIS:
			data_address = MPUREG_ACCEL_DATA_X0_UI + 4;
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Start streaming: Raw Accel z-axis at 32 KHz");
			break;

		default:
			INV_MSG(INV_MSG_LEVEL_INFO,
					"Couldn't select the axis to start streaming the raw accel");
			//return -1;
		}

	} else if (SENSOR_ODR == HIGH_RATE_16KHZ) {
		INV_MSG(INV_MSG_LEVEL_INFO,
				"Start streaming: Raw Accel x,y,z at 16 KHz");

	} else if (SENSOR_ODR == HIGH_RATE_8KHZ) {
		INV_MSG(INV_MSG_LEVEL_INFO,
				"Start streaming: Raw Accel x,y,z at 8 KHz");
	}
}

static void SetupMCUHardware(struct inv_iim423xx_serif *icm_serif) {

	//inv_io_hal_board_init();

	pinMode(IIM42352_CS, OUTPUT);
	pinMode(IIM42352_INT1, INPUT_PULLUP);
	pinMode(IIM42352_INT2, INPUT_PULLUP);

	/* Configure Log UART */
	//config_uart(LOG_UART_ID);
	Serial.begin(LOG_UART_SPEED);

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	/* Configure Streaming UART */
	//inv_uart_mngr_init_struct_t uart_mngr_config;
	//
	//uart_mngr_config.uart_num = INV_UART_SENSOR_CTRL;
	//uart_mngr_config.baudrate = 3000000;
	//uart_mngr_config.flow_ctrl = INV_UART_FLOW_CONTROL_RTS_CTS;
	//inv_uart_mngr_init(&uart_mngr_config);

	/*
	 * Configure input capture mode GPIO connected to pin 13.
	 * This pin is connected to Iim423xx INT1 output and thus will receive interrupts
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	 */
	//inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_cb, 0);
	//attachInterrupt(digitalPinToInterrupt(IIM42352_INT1), ext_interrupt_cb,FALLING);

	/* Init timer peripheral for delay */
	//inv_delay_init(DELAY_TIMER);

	/*
	 * Configure the timer for the timebase
	 */
	//inv_timer_configure_timebase(1000000);
	//inv_timer_enable(TIMEBASE_TIMER);

#if USE_CLK_IN
	rtc_timer_init(NULL);
	/* Output 32kHz SLCK to PA17, it is up to user to connect it or not at board level to have CLKIN capability */
	inv_gpio_output_clk_on_pin(INV_GPIO_CLKIN);
#endif

	/* Initialize serial inteface between MCU and Iim423xx */
	icm_serif->context = 0; /* no need */
	icm_serif->read_reg = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read = 1024 * 32; /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024 * 32; /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;

	/* Configure SPI to 24 MHz */
	//inv_io_hal_configure_spi_speed(24);
	//inv_io_hal_init(icm_serif);
	SPI.begin(IIM42352_CLK, IIM42352_MISO, IIM42352_MOSI, IIM42352_CS);
	SPI.beginTransaction(SPISettings(24000000, MSBFIRST, SPI_MODE3));

	//SPI.endTransaction();

}

int SetupInvDevice(struct inv_iim423xx_serif *icm_serif) {
	int rc = 0;
	uint8_t who_am_i;

	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Iim423xx");

	rc = inv_iim423xx_init(&icm_driver, icm_serif, NULL);

	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_INFO, "Check Iim423xx whoami value");

	rc = inv_iim423xx_get_who_am_i(&icm_driver, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR,
				"!!! ERROR : failed to read Iim423xx whoami value.");
		return rc;
	}

	if (who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR,
				"!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)",
				who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	return rc;
}

int ConfigureInvDevice() {
	int rc = 0;
	uint8_t data;

	/* Set Pulse to support high rates */
	rc |= inv_iim423xx_read_reg(&icm_driver, MPUREG_INT_CONFIG1, 1, &data);
	data |= ((uint8_t) IIM423XX_INT_TPULSE_DURATION_8_US)
			| ((uint8_t) IIM423XX_INT_TDEASSERT_DISABLED);
	rc |= inv_iim423xx_write_reg(&icm_driver, MPUREG_INT_CONFIG1, 1, &data);

	/* Disable fifo usage, data will be read from sensors registers*/
	rc |= inv_iim423xx_configure_fifo(&icm_driver, INV_IIM423XX_FIFO_DISABLED);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR,
				"!!! ERROR : failed to initialize Iim423xx.");
		return rc;
	}

	rc |= inv_iim423xx_enable_clkin_rtc(&icm_driver, USE_CLK_IN);

	rc |= inv_iim423xx_set_accel_fsr(&icm_driver,
			IIM423XX_ACCEL_CONFIG0_FS_SEL_16g);

	if (SENSOR_ODR == HIGH_RATE_8KHZ) {
		rc |= inv_iim423xx_set_accel_frequency(&icm_driver,
				IIM423XX_ACCEL_CONFIG0_ODR_8_KHZ);

	} else if (SENSOR_ODR == HIGH_RATE_16KHZ) {
		rc |= inv_iim423xx_set_accel_frequency(&icm_driver,
				IIM423XX_ACCEL_CONFIG0_ODR_16_KHZ);

	} else if (SENSOR_ODR == HIGH_RATE_32KHZ) {
		rc |= inv_iim423xx_set_accel_frequency(&icm_driver,
				IIM423XX_ACCEL_CONFIG0_ODR_32_KHZ);

	}

	rc |= inv_iim423xx_enable_accel_low_noise_mode(&icm_driver);

	return rc;
}

/* --------------------------------------------------------------------------------------
 *  loop: called in an endless loop
 * -------------------------------------------------------------------------------------- */
void loop() {
	int rc = 0;
	/* Poll device for data */
	//if (irq_from_device & TO_MASK(IIM42352_INT1)) {

		/* Increment counter */
//		out_str[FRAME_CNT_IDX]++;
//	inv_io_hal_read_reg(&iim423xx_serif, data_address, &out_str[RACC_X_IDX], payload_size_bytes);
		delay(500);
		do {
			while (digitalRead(IIM42352_INT1) == HIGH)
				;

			inv_io_hal_read_reg(&iim423xx_serif, MPUREG_ACCEL_DATA_Z0_UI,
					&out_str[RACC_X_IDX], payload_size_bytes);

			Serial.write(out_str[HEADER_LSB_IDX]);
			Serial.write(out_str[RACC_X_IDX]);
			Serial.write(out_str[RACC_X_IDX + 1]);

			while (digitalRead(IIM42352_INT1) == LOW)
				;

		}while(1);

#ifdef DEBUG_REGISTERS
		//Register Verification
		 //Bank 0
		 Serial.println("");
		 Serial.println("");
		 Serial.println("");
		 Serial.println("REGISTERS: ");
		 inv_iim423xx_set_reg_bank(&icm_driver, 0);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x11, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x14, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x16, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4C, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4E, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x50, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x52, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x53, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x54, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x57, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x5F, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x60, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x62, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x64, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x65, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x66, &out_str[RACC_X_IDX],1);

		 //Bank 1
		 inv_iim423xx_set_reg_bank(&icm_driver, 1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x7A, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x7B, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x7C, &out_str[RACC_X_IDX],1);

		 //Bank 4
		 inv_iim423xx_set_reg_bank(&icm_driver, 4);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x09, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4A, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4B, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4C, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4D, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x4F, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x50, &out_str[RACC_X_IDX],1);
		 inv_io_hal_read_reg(&iim423xx_serif, 0x51, &out_str[RACC_X_IDX],1);
		 while (1);
#endif

		/* Split uart send in several packet to allow host to suport 3Mbaud */
//	while(inv_uart_puts(INV_UART_SENSOR_CTRL,(const char*)&out_str[HEADER_LSB_IDX], HEADER_SIZE) != 0);
//	while(inv_uart_puts(INV_UART_SENSOR_CTRL,(const char*)&out_str[FRAME_CNT_IDX], FRAME_CNT_SIZE) != 0);
//	while(inv_uart_puts(INV_UART_SENSOR_CTRL,(const char*)&out_str[RACC_X_IDX], payload_size_bytes) != 0);
		inv_helper_disable_irq();
		irq_from_device &= ~TO_MASK(IIM42352_INT1);
		inv_helper_enable_irq();

}

/*
 * Iim423xx interrupt handler.
 * Function is executed when an Iim423xx interrupt rises on MCU.
 * This function get a timestamp and store it in the timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * are implemented for shared variable timestamp_buffer.
 */
void IRAM_ATTR ext_interrupt_cb(void) {
	// irq_from_device |= TO_MASK(IIM42352_INT1);
}

/*
 * Helper function to check RC value and block programm exectution
 */
static void check_rc(int rc, const char *msg_context) {
	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)\r\n", msg_context, rc,
				inv_error_str(rc));
		//Serial.println
		while (1)
			;
	}
}

/*
 * Printer function for message facility
 */
void msg_printer(int level, const char *str, va_list ap) {
	static char out_str[256]; // static to limit stack usage
	unsigned idx = 0;
	const char *s[INV_MSG_LEVEL_MAX] = { "",    // INV_MSG_LEVEL_OFF
			"[E] ", // INV_MSG_LEVEL_ERROR
			"[W] ", // INV_MSG_LEVEL_WARNING
			"[I] ", // INV_MSG_LEVEL_INFO
			"[V] ", // INV_MSG_LEVEL_VERBOSE
			"[D] ", // INV_MSG_LEVEL_DEBUG
			};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if (idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if (idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if (idx >= (sizeof(out_str)))
		return;

	//inv_uart_mngr_puts(LOG_UART_ID, out_str, idx);
	Serial.println(out_str);
}

/*
 * Clock calibration module needs to disable IRQ. Thus inv_helper_disable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_disable_irq(void) {
	//inv_disable_irq();
	noInterrupts();
}

/*
 * Clock calibration module needs to enable IRQ. Thus inv_helper_enable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_enable_irq(void) {
	//inv_enable_irq();
	interrupts();
}

/*	COPIADO E COLOCADO NA lim423ccDriver_HL.c / Poderia ficar aqui?
 * Iim423xx driver needs to get time in us. Let's give its implementation here.


 uint64_t inv_iim423xx_get_time_us(void)
 {
 #if USE_CLK_IN
 return rtc_timer_get_time_us();
 #else
 // return inv_timer_get_counter(TIMEBASE_TIMER);
 return micros ();
 #endif
 }

 * Iim423xx driver needs a sleep feature from external device. Thus inv_iim423xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.

 void inv_iim423xx_sleep_us(uint32_t us)
 {
 //inv_delay_us(us);
 delayMicroseconds(us);
 }*/

uint64_t inv_iim423xx_get_time_us(void) {
#if USE_CLK_IN
return rtc_timer_get_time_us();
#else
// return inv_timer_get_counter(TIMEBASE_TIMER);
	return micros();
#endif
}

//Iim423xx driver needs a sleep feature from external device. Thus inv_iim423xx_sleep_us
// is defined as extern symbol in driver. Let's give its implementation here.

void inv_iim423xx_sleep_us(uint32_t us) {
//inv_delay_us(us);
	delayMicroseconds(us);
}


int inv_io_hal_read_reg(struct inv_iim423xx_serif *serif, uint8_t reg,
		uint8_t *rbuffer, uint32_t rlen) {

	switch (serif->serif_type) {
	case IIM423XX_AUX1_SPI3:
	case IIM423XX_AUX1_SPI4:
	case IIM423XX_AUX2_SPI3:
	case IIM423XX_UI_SPI4:

#ifdef DEBUG_SPI_READ
		Serial.println("");
		 Serial.print("Read in the register: ");
		 Serial.print(reg, HEX);
		 Serial.print(" size ");
		 Serial.println(rlen, HEX);
#endif
		//	inv_spi_master_read_register(INV_SPI_AP, reg, rlen, rbuffer);
		inv_spi_master_read_register(0, reg, rlen, rbuffer);
		return 0;

	case IIM423XX_UI_I2C:
//			while(inv_i2c_master_read_register(ICM_I2C_ADDR, reg, rlen, rbuffer)) {
//				inv_delay_us(32000); // Loop in case of I2C timeout
//			}
		return -1;
	default:
		return -1;
	}
}

int inv_io_hal_write_reg(struct inv_iim423xx_serif *serif, uint8_t reg,
		const uint8_t *wbuffer, uint32_t wlen) {
	int rc;

	bzero(sendBuffer, sizeof(sendBuffer));

	switch (serif->serif_type) {
	case IIM423XX_AUX1_SPI3:
	case IIM423XX_AUX1_SPI4:
	case IIM423XX_AUX2_SPI3:
	case IIM423XX_UI_SPI4:
		for (uint32_t i = 0; i < wlen; i++) {
			// rc = inv_spi_master_write_register(INV_SPI_AP, reg+i, 1, &wbuffer[i]);
			rc = inv_spi_master_write_register(0, reg + i, 1, &wbuffer[i]);

			if (rc)
				return rc;
		}
		return 0;
	case IIM423XX_UI_I2C:
//			while(inv_i2c_master_write_register(ICM_I2C_ADDR, reg, wlen, wbuffer)) {
//				inv_delay_us(32000); // Loop in case of I2C timeout
//			}
//			return 0;
	default:
		return -1;
	}
}

unsigned long inv_spi_master_read_register(unsigned spi_num,
		unsigned char register_addr, unsigned short len, unsigned char *value) {

	if (len + 1 > SPI_BUFFER_SIZE)
		return 1;

	sendBuffer[0] = (uint8_t) register_addr | 0x80;

	memset(&sendBuffer[1], 0x00, len);
	digitalWrite(IIM42352_CS, LOW);
	SPI.transferBytes(sendBuffer, recvBuffer, len + 1);
	digitalWrite(IIM42352_CS, HIGH);

#ifdef DEBUG_SPI_READ
	for (int i = 1; i < len + 1; i++) {
	 Serial.print("The value: ");
	 Serial.print(recvBuffer[i], BIN);
	 Serial.println("");
	 }
#endif
	memcpy((uint8_t*) value, &recvBuffer[1], len);

	/*
	 pdc_packet_t pdc_spi_packet;

	 if(sm[spi_num].transfer_done_cb)
	 pdc_rx_clear_cnt(sm[spi_num].p_pdc);
	 // Desactivate Irq during buffer write
	 inv_disable_irq();
	 pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].rx_buffer[0];
	 pdc_spi_packet.ul_size = len + 1;
	 pdc_rx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);

	 sm[spi_num].tx_buffer[0] = (uint8_t) register_addr | 0x80;
	 memset(&sm[spi_num].tx_buffer[1], 0x00, len);

	 pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].tx_buffer[0];
	 pdc_spi_packet.ul_size = len + 1;
	 pdc_tx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);
	 // Re activate Irq
	 inv_enable_irq();

	 if (sm[spi_num].transfer_done_cb == 0) {
	 // Enable the RX and TX PDC transfer requests
	 pdc_enable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);

	 // Waiting transfer done
	 while((spi_read_status(sm[spi_num].p_spi) & SPI_SR_ENDRX) == 0);

	 // Disable the RX and TX PDC transfer requests
	 pdc_disable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTDIS |
	 PERIPH_PTCR_TXTDIS);

	 memcpy(value, &sm[spi_num].rx_buffer[1], len);
	 } else {
	 sm[spi_num].rx_dest_addr = value;
	 sm[spi_num].rx_len = len;

	 // Transfer done handler is in ISR
	 spi_enable_interrupt(sm[spi_num].p_spi, SPI_IER_RXBUFF) ;

	 // Enable the RX and TX PDC transfer requests
	 pdc_enable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	 }
	 */

	return 0;
}

unsigned long inv_spi_master_write_register(unsigned spi_num,
		unsigned char register_addr, unsigned short len,
		const unsigned char *value) {

	if (len + 1 > SPI_BUFFER_SIZE)
		return 1;

	sendBuffer[0] = (uint8_t) register_addr;
	memcpy(&sendBuffer[1], (uint8_t*) value, len);

#ifdef DEBUG_SPI_WRITE
	Serial.print("Wrote in the register ");
	 Serial.print(sendBuffer[0], HEX);
	 Serial.print(" ,the value ");
	 Serial.print(sendBuffer[1], HEX);
	 Serial.println(" (HEX)");
#endif

	/*
	 pdc_packet_t pdc_spi_packet;

	 // Desactivate Irq during buffer write
	 inv_disable_irq();
	 pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].rx_buffer[0];
	 pdc_spi_packet.ul_size = len + 1;
	 pdc_rx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);

	 sm[spi_num].tx_buffer[0] = (uint8_t) register_addr;
	 memcpy(&sm[spi_num].tx_buffer[1], (uint8_t *)value, len);

	 pdc_spi_packet.ul_addr = (uint32_t)&sm[spi_num].tx_buffer[0];
	 pdc_spi_packet.ul_size = len + 1;
	 pdc_tx_init(sm[spi_num].p_pdc, &pdc_spi_packet, NULL);

	 //«Enable the RX and TX PDC transfer requests
	 pdc_enable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTEN | PERIPH_PTCR_TXTEN);
	 // Re activate Irq
	 inv_enable_irq();
	 // Waiting transfer done
	 while((spi_read_status(sm[spi_num].p_spi) & SPI_SR_TXEMPTY) == 0);

	 // Disable the RX and TX PDC transfer requests
	 pdc_disable_transfer(sm[spi_num].p_pdc, PERIPH_PTCR_RXTDIS |
	 PERIPH_PTCR_TXTDIS);
	 */
	digitalWrite(IIM42352_CS, LOW);
	//SPI.transferBytes(sendBuffer, 0, 1);

	for (int i = 0; i < len + 1; i++) {
		SPI.transfer(sendBuffer[i]);
	}
	//SPI.transfer(sendBuffer[0]);
	//SPI.transfer(sendBuffer[1]);
	digitalWrite(IIM42352_CS, HIGH);
	return 0;
}



