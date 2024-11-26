/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the SBU MUX Functionality Example
*              for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cybsp.h"
#include "stdio.h"
#include "inttypes.h"
#include "sbu_mux.h"
#include "cy_usbpd_typec.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* UART Debug Print */
#define DEBUG_PRINT             (0u)

/* Switch debounce delay */
#define SW_DEBOUNCE_DELAY       (25u)

/*******************************************************************************
* Global Variables
*******************************************************************************/

/* USB PD context for Port0 */
cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;

/* PD device policy configuration and status structure for port0 */
cy_stc_pd_dpm_config_t gl_dpmConfig_p0;

/* SwitchFlag */
volatile uint8_t SwitchPressFlag = 0;

/* Holds CC status for Port0 */
cy_pd_cc_state_t ccStatusp0;

#if PMG1_PD_DUALPORT_ENABLE

/* USB PD context for Port1 */
cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;

/* PD device policy configuration and status structure for Port1 */
cy_stc_pd_dpm_config_t gl_dpmConfig_p1;

/* Holds CC status for Port1 */
cy_pd_cc_state_t ccStatusp1;

#endif /* PMG1_PD_DUALPORT_ENABLE  */

/* Switch interrupt function declaration */
void Switch_ISR();

/* User Switch Interrupt Configuration */
const cy_stc_sysint_t switch_intr_config =
{
    .intrSrc = CYBSP_USER_BTN_IRQ,                  /* Source of interrupt signal */
    .intrPriority = 3u                              /* Interrupt priority */
};

/*******************************************************************************
* Function Name: get_dpm_config_p0
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port0
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
cy_stc_pd_dpm_config_t* get_dpm_config_p0(void)
{
     return &(gl_dpmConfig_p0);
}

/*******************************************************************************
* Function Name: get_dpm_config_p1
********************************************************************************
* Summary:
*  Gets the DPM configuration for Port1
*
* Parameters:
*  None
*
* Return:
*  cy_stc_pd_dpm_config_t
*
*******************************************************************************/
#if PMG1_PD_DUALPORT_ENABLE
cy_stc_pd_dpm_config_t* get_dpm_config_p1(void)
{
     return &(gl_dpmConfig_p1);
}
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*******************************************************************************
* Function Prototypes
********************************************************************************/
#if DEBUG_PRINT
/* Structure for UART context */
cy_stc_scb_uart_context_t        CYBSP_UART_context;

/* Variable used for tracking the print status */
volatile bool ENTER_LOOP = true;
#endif /* DEBUG_PRINT */

/*******************************************************************************
* Function Name: Switch_ISR
********************************************************************************
*
* Summary:
*  This function is executed when interrupt is triggered through
*  the user switch press.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void Switch_ISR()
{
    /* Set Switch press flag to 1 */
    SwitchPressFlag = 1;

    /* Clear the Interrupt */
    Cy_GPIO_ClearInterrupt(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  System entrance point. This function performs
*  - Initialize setup of device, USBPD
*  - Start Type-C
*  - Based on CC line status SBU1/SBU2 pins routed
*    to correct AUX_1/AUX_2 or AUX_P/AUX_N pins
*  - On switch press, SBU1/SBU2 pins routed
*    to correct LSTX/LSRX pins
*
* Parameters:
*  None
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

    /* variable to hold active CC line value for Port0 */
    uint8_t activeCClineP0 = 0;

#if PMG1_PD_DUALPORT_ENABLE
    /* variable to hold active CC line value for Port1 */
    uint8_t activeCClineP1 = 0;
#endif /* PMG1_PD_DUALPORT_ENABLE  */

    /* Initialize the device and board peripherals */
    result = cybsp_init() ;

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

#if DEBUG_PRINT
    cy_stc_scb_uart_context_t CYBSP_UART_context;

    /* Configure and enable the UART peripheral */
    Cy_SCB_UART_Init(CYBSP_UART_HW, &CYBSP_UART_config, &CYBSP_UART_context);
    Cy_SCB_UART_Enable(CYBSP_UART_HW);

    /* Sequence to clear screen */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "\x1b[2J\x1b[;H");

    /* Print "EZ-PDTM PMG1 MCU: USB2.0 Mux Functionality" */
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "***************** ");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, "EZ-PDTM PMG1 MCU: SBU Mux Functionality");
    Cy_SCB_UART_PutString(CYBSP_UART_HW, " ***************** \r\n\n");
#endif /* DEBUG_PRINT */

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize the USBPD driver */
#if defined(CY_DEVICE_SERIES_PMG1S2)
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, NULL,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_config_p0);
#else
    Cy_USBPD_Init(&gl_UsbPdPort0Ctx, 0, mtb_usbpd_port0_HW, mtb_usbpd_port0_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port0_config, get_dpm_config_p0);

#if PMG1_PD_DUALPORT_ENABLE
    Cy_USBPD_Init(&gl_UsbPdPort1Ctx, 1, mtb_usbpd_port1_HW, mtb_usbpd_port1_HW_TRIM,
            (cy_stc_usbpd_config_t *)&mtb_usbpd_port1_config, get_dpm_config_p1);
#endif /* PMG1_PD_DUALPORT_ENABLE  */
#endif /* defined(CY_DEVICE_SERIES_PMG1S2) */

    /* Initialize Switch GPIO interrupt */
    Cy_SysInt_Init(&switch_intr_config, &Switch_ISR);

    /* Clear any pending interrupt and enable the User Switch Interrupt */
    NVIC_ClearPendingIRQ(switch_intr_config.intrSrc);
    NVIC_EnableIRQ(switch_intr_config.intrSrc);

    /* Start Type-C for Port0 */
    Cy_USBPD_TypeC_Start(&gl_UsbPdPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    /* Start Type-C for Port1 */
    Cy_USBPD_TypeC_Start(&gl_UsbPdPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE  */

    /* Check and assign port1value */
    checkPortStatus();

    /* If Port0 is active, check whether Rp and Rd are applied to which CC line. Later, configure the SBU MUX according to the Type-C orientation */
    if((ccStatusp0.cc[0] >= (uint8_t)CY_PD_RD_1_5A) && (ccStatusp0.cc[0] <= (uint8_t)CY_PD_RD_3A))
    {
        /* CC1 is active */
        activeCClineP0 = 1;
    }
    else if ((ccStatusp0.cc[1] >= (uint8_t)CY_PD_RD_1_5A) && (ccStatusp0.cc[1] <= (uint8_t)CY_PD_RD_3A))
    {
        /* CC2 is active */
        activeCClineP0 = 2;
    }

    /* MUX configuration function called to configure SBU MUX based on the active CC line for Port0 */
    mux_ctrl_set_cfg(&gl_UsbPdPort0Ctx, MUX_CONFIG_INIT, activeCClineP0);

    /* Print Type-C orientation for Port0 */
    if((gl_UsbPdPort0Ctx.sbu1State == CY_USBPD_SBU_CONNECT_AUX1)&& (gl_UsbPdPort0Ctx.sbu2State == CY_USBPD_SBU_CONNECT_AUX2))
    {
#if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Port0, Orientation A \r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "CC1 is Active \r\n");
#endif /* DEBUG_PRINT */
    }
    else if((gl_UsbPdPort0Ctx.sbu1State == CY_USBPD_SBU_CONNECT_AUX2)&& (gl_UsbPdPort0Ctx.sbu2State == CY_USBPD_SBU_CONNECT_AUX1))
    {
#if DEBUG_PRINT
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "Port0, Orientation B \r\n");
        Cy_SCB_UART_PutString(CYBSP_UART_HW, "CC2 is Active \r\n");
#endif /* DEBUG_PRINT */
    }

    /* If Port1 is active, check whether Rp and Rd are applied to which CC line. Later, configure the SBU MUX according to the Type-C orientation */
#if PMG1_PD_DUALPORT_ENABLE
    if((ccStatusp1.cc[0] >= (uint8_t)CY_PD_RD_1_5A) && (ccStatusp1.cc[0] <= (uint8_t)CY_PD_RD_3A))
    {
        /* CC1 is active */
        activeCClineP1 = 1;
    }
    else if ((ccStatusp1.cc[1] >= (uint8_t)CY_PD_RD_1_5A) && (ccStatusp1.cc[1] <= (uint8_t)CY_PD_RD_3A))
    {
        /* CC2 is active */
        activeCClineP1 = 2;
    }

    /* MUX configuration function called to configure SBU MUX based on the active CC line for Port1 */
    mux_ctrl_set_cfg(&gl_UsbPdPort1Ctx, MUX_CONFIG_INIT, activeCClineP1);

    /* Print Type-C orientation for Port1 */
    if((gl_UsbPdPort1Ctx.sbu1State == CY_USBPD_SBU_CONNECT_AUX1)&& (gl_UsbPdPort1Ctx.sbu2State == CY_USBPD_SBU_CONNECT_AUX2))
    {
#if DEBUG_PRINT
       Cy_SCB_UART_PutString(CYBSP_UART_HW, "Port1, Orientation A \r\n");
       Cy_SCB_UART_PutString(CYBSP_UART_HW, "CC1 is Active \r\n");
#endif /* DEBUG_PRINT */
    }
    else if((gl_UsbPdPort1Ctx.sbu1State == CY_USBPD_SBU_CONNECT_AUX2)&& (gl_UsbPdPort1Ctx.sbu2State == CY_USBPD_SBU_CONNECT_AUX1))
    {
#if DEBUG_PRINT
       Cy_SCB_UART_PutString(CYBSP_UART_HW, "Port1, Orientation B \r\n");
       Cy_SCB_UART_PutString(CYBSP_UART_HW, "CC2 is Active \r\n");
#endif /* DEBUG_PRINT */
    }
#endif /* PMG1_PD_DUALPORT_ENABLE  */

    for (;;)
        {
#if DEBUG_PRINT
        if (ENTER_LOOP)
        {
            Cy_SCB_UART_PutString(CYBSP_UART_HW, "Entered for loop \r\n\n");
            ENTER_LOOP = false;
        }
#endif /* DEBUG_PRINT */

#if (!defined(CY_DEVICE_SERIES_PMG1S2))
        if (SwitchPressFlag)
        {
            /* Wait for 25 milliseconds for switch debounce */
            Cy_SysLib_Delay(SW_DEBOUNCE_DELAY);

            if (!Cy_GPIO_Read(CYBSP_USER_BTN_PORT, CYBSP_USER_BTN_PIN))
           {
                handleSwitchPressEvent(&gl_UsbPdPort0Ctx, activeCClineP0);

#if PMG1_PD_DUALPORT_ENABLE
                handleSwitchPressEvent(&gl_UsbPdPort1Ctx, activeCClineP1);
#endif /* PMG1_PD_DUALPORT_ENABLE  */

                /* Clear the Switch Press Event */
                SwitchPressFlag = 0;
           }
        }
#endif /* !defined(CY_DEVICE_SERIES_PMG1S2) */
        }
}

/* [] END OF FILE */
