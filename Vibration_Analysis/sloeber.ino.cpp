#ifdef __IN_ECLIPSE__
//This is a automatic generated file
//Please do not modify this file
//If you touch this file your change will be overwritten during the next build
//This file has been generated on 2022-09-02 18:49:12

#include "Arduino.h"
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
#include "example-high-rate-logger.h"
#include "EmbUtils/Message.h"
#include "EmbUtils/ErrorHelper.h"
#include "EmbUtils/RingBuffer.h"

void setup() ;
static void SetupMCUHardware(struct inv_iim423xx_serif *icm_serif) ;
int SetupInvDevice(struct inv_iim423xx_serif *icm_serif) ;
int ConfigureInvDevice() ;
void loop() ;
void IRAM_ATTR ext_interrupt_cb(void) ;
static void check_rc(int rc, const char *msg_context) ;
void msg_printer(int level, const char *str, va_list ap) ;
void inv_helper_disable_irq(void) ;
void inv_helper_enable_irq(void) ;
uint64_t inv_iim423xx_get_time_us(void) ;
void inv_iim423xx_sleep_us(uint32_t us) ;
int inv_io_hal_read_reg(struct inv_iim423xx_serif *serif, uint8_t reg, 		uint8_t *rbuffer, uint32_t rlen) ;
int inv_io_hal_write_reg(struct inv_iim423xx_serif *serif, uint8_t reg, 		const uint8_t *wbuffer, uint32_t wlen) ;
unsigned long inv_spi_master_read_register(unsigned spi_num, 		unsigned char register_addr, unsigned short len, unsigned char *value) ;
unsigned long inv_spi_master_write_register(unsigned spi_num, 		unsigned char register_addr, unsigned short len, 		const unsigned char *value) ;

#include "Vibration_Analysis.ino"


#endif
