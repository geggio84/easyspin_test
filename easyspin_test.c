/**
 ******************************************************************************
 * @file    main.c
 * @author  EMEA AMS-IPD Marketing & Application, Prague - VE
 * @version V1.0.1
 * @date    June-2012
 * @brief   Demo program entry file.
 ******************************************************************************
 * @copy
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 */

/* Includes ------------------------------------------------------------------*/
#include "easyspin_test.h"
#include "easyspin.h"
#include "eMotionControl.h"
#include "easyspin_target_config.h"
#include "gpio_lib.h"

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>

/** @addtogroup Examples
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint32_t easySPIN_rx_data = 0;

const char *device = "/dev/spidev1.1";
uint32_t mode;
uint8_t bits = 8;
uint32_t speed = 500000;
uint16_t delay;
int verbose;
int spi_fd;

/* Private functions ---------------------------------------------------------*/
static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void print_usage(const char *prog)
{
	printf("Usage: %s [-DsbdlHOLC3]\n", prog);
	puts("  -D --device   device to use (default /dev/spidev1.1)\n"
	     "  -s --speed    max speed (Hz)\n"
	     "  -d --delay    delay (usec)\n"
	     "  -b --bpw      bits per word \n"
	     "  -l --loop     loopback\n"
	     "  -H --cpha     clock phase\n"
	     "  -O --cpol     clock polarity\n"
	     "  -L --lsb      least significant bit first\n"
	     "  -C --cs-high  chip select active high\n"
	     "  -3 --3wire    SI/SO signals shared\n"
	     "  -v --verbose  Verbose (show tx buffer)\n"
	     "  -R --ready    slave pulls low to pause\n"
	     "  -2 --dual     dual transfer\n"
	     "  -4 --quad     quad transfer\n");
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "device",  1, 0, 'D' },
			{ "speed",   1, 0, 's' },
			{ "delay",   1, 0, 'd' },
			{ "bpw",     1, 0, 'b' },
			{ "loop",    0, 0, 'l' },
			{ "cpha",    0, 0, 'H' },
			{ "cpol",    0, 0, 'O' },
			{ "lsb",     0, 0, 'L' },
			{ "cs-high", 0, 0, 'C' },
			{ "3wire",   0, 0, '3' },
			{ "ready",   0, 0, 'R' },
			{ "dual",    0, 0, '2' },
			{ "verbose", 0, 0, 'v' },
			{ "quad",    0, 0, '4' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "D:s:d:b:lHOLC3R24:v", lopts, NULL);

		if (c == -1)
			break;

		switch (c) {
		case 'D':
			device = optarg;
			break;
		case 's':
			speed = atoi(optarg);
			break;
		case 'd':
			delay = atoi(optarg);
			break;
		case 'b':
			bits = atoi(optarg);
			break;
		case 'l':
			mode |= SPI_LOOP;
			break;
		case 'H':
			mode |= SPI_CPHA;
			break;
		case 'O':
			mode |= SPI_CPOL;
			break;
		case 'L':
			mode |= SPI_LSB_FIRST;
			break;
		case 'C':
			mode |= SPI_CS_HIGH;
			break;
		case '3':
			mode |= SPI_3WIRE;
			break;
		case 'v':
			verbose = 1;
			break;
		case 'R':
			mode |= SPI_READY;
			break;
		case '2':
			mode |= SPI_TX_DUAL;
			break;
		case '4':
			mode |= SPI_TX_QUAD;
			break;
		default:
			print_usage(argv[0]);
			break;
		}
	}
	if (mode & SPI_LOOP) {
		if (mode & SPI_TX_DUAL)
			mode |= SPI_RX_DUAL;
		if (mode & SPI_TX_QUAD)
			mode |= SPI_RX_QUAD;
	}
}

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(int argc, char *argv[]) {

	int ret = 0;
	uint8_t *tx;
	uint8_t *rx;
	int size;
	int i;

	parse_opts(argc, argv);

	spi_fd = open(device, O_RDWR);
	if (spi_fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MODE32, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(spi_fd, SPI_IOC_RD_MODE32, &mode);
	if (ret == -1)
		pabort("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1)
		pabort("can't get max speed hz");

	printf("spi mode: 0x%x\n", mode);
	printf("bits per word: %d\n", bits);
	printf("max speed: %d Hz (%d KHz)\n", speed, speed/1000);

	/* eMotionControl module initialization */
	eMotionControl_Init();

	/* Customize target stepper-motor specific registers at easySPIN module level */
	/* TVAL register setup */ 
	easySPIN_SetParam(easySPIN_TVAL, 0x00);

	/* T_FAST register setup */
	easySPIN_SetParam(easySPIN_STEP_MODE, easySPIN_STEP_SEL_1
			| easySPIN_SYNC_SEL_1_2);

	/* TON_MIN register setup */
	easySPIN_SetParam(easySPIN_TON_MIN, 0x00);

	/* TOFF_MIN register setup */
	easySPIN_SetParam(easySPIN_TOFF_MIN, 0x00);

	/* OCD_TH register setup */
	easySPIN_SetParam(easySPIN_OCD_TH, easySPIN_OCD_TH_2625mA);

	/* STEP_MODE register setup  */
	easySPIN_SetParam(easySPIN_STEP_MODE, easySPIN_STEP_SEL_1
			| easySPIN_SYNC_SEL_1_2);

	/* ALARM_EN register setup  */
	easySPIN_SetParam(easySPIN_ALARM_EN, easySPIN_ALARM_EN_OVERCURRENT
			| easySPIN_ALARM_EN_THERMAL_SHUTDOWN
			| easySPIN_ALARM_EN_THERMAL_WARNING
			| easySPIN_ALARM_EN_UNDERVOLTAGE | easySPIN_ALARM_EN_SW_TURN_ON
			| easySPIN_ALARM_EN_WRONG_NPERF_CMD);

	/* CONFIG register setup */
	easySPIN_SetParam(easySPIN_CONFIG, easySPIN_CONFIG_INT_16MHZ
			| easySPIN_CONFIG_EN_TQREG_INT_REG | easySPIN_CONFIG_OC_SD_ENABLE
			| easySPIN_CONFIG_SR_180V_us | easySPIN_CONFIG_TSW_8_us);

	/* Read STATUS register */
	easySPIN_rx_data = easySPIN_Get_Status();

//	/* GoHome command example */
//	eMotionControl_GoHome();
//	eMotionControl_WaitWhileActive();
//	currentPosition = easySPIN_GetParam(easySPIN_ABS_POS);
//
//	/* GoTo command example */
//	eMotionControl_GoTo(-65);
//	eMotionControl_WaitWhileActive();
//	currentPosition = easySPIN_GetParam(easySPIN_ABS_POS);
//
//	eMotionControl_GoTo(0);
//	eMotionControl_WaitWhileActive();
//	currentPosition = easySPIN_GetParam(easySPIN_ABS_POS);
//
//	eMotionControl_GoTo(65);
//	eMotionControl_WaitWhileActive();
//	currentPosition = easySPIN_GetParam(easySPIN_ABS_POS);
//
//	eMotionControl_GoTo(0);
//	eMotionControl_WaitWhileActive();
//	currentPosition = easySPIN_GetParam(easySPIN_ABS_POS);
//
//	/* Move command example */
//	eMotionControl_Move(DIR_Forward, 200);
//	eMotionControl_WaitWhileActive();
//
//	eMotionControl_Move(DIR_Reverse, 200);
//	eMotionControl_WaitWhileActive();
//
//	/* Run command example */
//	eMotionControl_Run(DIR_Forward, MAX_STEPS_PER_SEC);

	/* Wait few seconds - motor turns */
	sleep(3);

	/* Enable easySPIN powerstage */
	easySPIN_Enable();
	set_gpio_value(easySPIN_DIR_GPIO,HIGH);
	for (i=0; i<1000; i++){
		set_gpio_value(easySPIN_STCK_GPIO,(i%2));
		usleep(1000);
	};
	set_gpio_value(easySPIN_DIR_GPIO,LOW);
	for (i=0; i<1000; i++){
		set_gpio_value(easySPIN_STCK_GPIO,(i%2));
		usleep(1000);
	};

	/* SoftStop command example */
	//while(eMotionControl_GetState() != Steady);
	//eMotionControl_SoftStop();
	//eMotionControl_WaitWhileActive();
	
	/* Disable power stage */
	easySPIN_Disable();

	close(spi_fd);

	//while (1)
	//	;
}
