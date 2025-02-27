/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 * that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MKL02Z32xxx4
package_id: MKL02Z32VFM4
mcu_data: ksdk2_0
processor_version: 0.0.9
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"



/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitPins:
- options: {callFromInitBoot: 'true', coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '17', peripheral: UART0, signal: TX, pin_signal: ADC0_SE5/CMP0_IN3/PTB1/IRQ_6/UART0_TX/UART0_RX}
  - {pin_num: '18', peripheral: UART0, signal: RX, pin_signal: ADC0_SE4/PTB2/IRQ_7/UART0_RX/UART0_TX}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : BOARD_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void BOARD_InitPins(void)
{
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);

	/* PORTB1 (pin 17) is configured as UART0_TX */
	PORT_SetPinMux(PORTB, 1U, kPORT_MuxAlt2);

	/* PORTB2 (pin 18) is configured as UART0_RX */
	PORT_SetPinMux(PORTB, 2U, kPORT_MuxAlt2);

	/* PORTB6 (pin 1) is configured as PTB6 */
	PORT_SetPinMux(PORTB, 6U, kPORT_MuxAsGpio);

	/* PORTB7 Green Led */
	PORT_SetPinMux(PORTB, 7U, kPORT_MuxAsGpio);

	/* PORTB10 Blue Led */
	PORT_SetPinMux(PORTB, 10U, kPORT_MuxAsGpio);

	/* PORTB8 (pin X) is configured as PTB8 */
	PORT_SetPinMux(PORTB, 8U, kPORT_MuxAsGpio);

	SIM->SOPT5 = ((SIM->SOPT5 &
			/* Mask bits to zero which are setting */
			(~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK)))

			/* UART0 transmit data source select: UART0_TX pin. */
			| SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)

			/* UART0 receive data source select: UART0_RX pin. */
			| SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX));
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C0_InitPins:
- options: {coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '23', peripheral: I2C0, signal: SCL, pin_signal: PTB3/IRQ_10/I2C0_SCL/UART0_TX, pull_enable: enable}
  - {pin_num: '24', peripheral: I2C0, signal: SDA, pin_signal: PTB4/IRQ_11/I2C0_SDA/UART0_RX, pull_enable: enable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C0_InitPins(void)
{
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);

	const port_pin_config_t portb3_pin23_config = {/* Internal pull-up resistor is enabled */
			kPORT_PullUp,
			/* Passive filter is disabled */
			kPORT_PassiveFilterDisable,
			/* Low drive strength is configured */
			kPORT_LowDriveStrength,
			/* Pin is configured as I2C0_SCL */
			kPORT_MuxAlt2};
	/* PORTB3 (pin 23) is configured as I2C0_SCL */
	PORT_SetPinConfig(PORTB, 3U, &portb3_pin23_config);

	const port_pin_config_t portb4_pin24_config = {/* Internal pull-up resistor is enabled */
			kPORT_PullUp,
			/* Passive filter is disabled */
			kPORT_PassiveFilterDisable,
			/* Low drive strength is configured */
			kPORT_LowDriveStrength,
			/* Pin is configured as I2C0_SDA */
			kPORT_MuxAlt2};
	/* PORTB4 (pin 24) is configured as I2C0_SDA */
	PORT_SetPinConfig(PORTB, 4U, &portb4_pin24_config);
}

/* clang-format off */
/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
I2C0_DeinitPins:
- options: {coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '23', peripheral: n/a, signal: disabled, pin_signal: PTB3/IRQ_10/I2C0_SCL/UART0_TX}
  - {pin_num: '24', peripheral: n/a, signal: disabled, pin_signal: PTB4/IRQ_11/I2C0_SDA/UART0_RX}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */
/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : I2C0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void I2C0_DeinitPins(void)
{
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);

	/* PORTB3 (pin 23) is disabled */
	PORT_SetPinMux(PORTB, 3U, kPORT_PinDisabledOrAnalog);

	/* PORTB4 (pin 24) is disabled */
	PORT_SetPinMux(PORTB, 4U, kPORT_PinDisabledOrAnalog);
}
/* FUNCTION ************************************************************************************************************
 *
 * Function Name : UART0_DeinitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void UART0_DeinitPins(void)
{
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);

	/* PORTB1 (pin 17) is configured as CMP0_IN3 */
	PORT_SetPinMux(PORTB, 1U, kPORT_PinDisabledOrAnalog);

	/* PORTB2 (pin 18) is configured as ADC0_SE4 */
	PORT_SetPinMux(PORTB, 2U, kPORT_PinDisabledOrAnalog);
}

/* clang-format on */

/* FUNCTION ************************************************************************************************************
 *
 * Function Name : UART0_InitPins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 * END ****************************************************************************************************************/
void UART0_InitPins(void)
{
	/* Port B Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortB);

	/* PORTB1 (pin 17) is configured as UART0_TX */
	PORT_SetPinMux(PORTB, 1U, kPORT_MuxAlt2);

	/* PORTB2 (pin 18) is configured as UART0_RX */
	PORT_SetPinMux(PORTB, 2U, kPORT_MuxAlt2);

	SIM->SOPT5 = ((SIM->SOPT5 &
			/* Mask bits to zero which are setting */
			(~(SIM_SOPT5_UART0TXSRC_MASK | SIM_SOPT5_UART0RXSRC_MASK)))

			/* UART0 transmit data source select: UART0_TX pin. */
			| SIM_SOPT5_UART0TXSRC(SOPT5_UART0TXSRC_UART_TX)

			/* UART0 receive data source select: UART0_RX pin. */
			| SIM_SOPT5_UART0RXSRC(SOPT5_UART0RXSRC_UART_RX));
}

/* clang-format off */
/***********************************************************************************************************************
 * EOF
 **********************************************************************************************************************/
