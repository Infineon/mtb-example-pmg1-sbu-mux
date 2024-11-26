/******************************************************************************
* File Name: sub_mux.c
*
* Description: This is source code for the SBU MUX Functionality.
*              This file implements functions for handling SBU MUX, SBU1/SB2
*              configuration based on active CC line.
*
* Related Document: See README.md
*
********************************************************************************
* Copyright 2023-2024, Cypress Semiconductor Corporation (an Infineon company) or
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
#include "sbu_mux.h"
#include "stdio.h"

/* timeout for Port0 */
uint16_t timeoutP0 = 2000;

#if (PMG1_PD_DUALPORT_ENABLE)
/* timeout for Port1 */
uint16_t timeoutP1 = 2000;
#endif /* PMG1_PD_DUALPORT_ENABLE  */

/*******************************************************************************
* Function Name: mux_ctrl_set_cfg
********************************************************************************
* Summary:
*  Configure SBU MUX based on active CC line for Port0/Port1
*
* Parameters:
*  context - USBPD context
*  cfg - settings for the Type-C Data MUX
*  activeCCline - active CC line for Port0/Port1
*
* Return:
*  None
*
*******************************************************************************/
void mux_ctrl_set_cfg(cy_stc_usbpd_context_t* context, mux_select_t cfg, uint8_t activeCCline)
{
    /* Configure SBU MUX for Port0 based on CC polarity */
    switch (cfg)
    {
        case MUX_CONFIG_ISOLATE:
            break;
        case MUX_CONFIG_SAFE:
            break;
        case MUX_CONFIG_SS_ONLY:
            break;
        case MUX_CONFIG_INIT:
            if (activeCCline == 1)
            {
                /* CC1 of Port0 is Active */
                Cy_USBPD_Mux_SbuSwitchConfigure(context, CY_USBPD_SBU_CONNECT_AUX1, CY_USBPD_SBU_CONNECT_AUX2);
            }
            else if(activeCCline == 2)
            {
                /* CC2 of Port0 is Active */
                Cy_USBPD_Mux_SbuSwitchConfigure(context, CY_USBPD_SBU_CONNECT_AUX2, CY_USBPD_SBU_CONNECT_AUX1);
            }
            break;
        case MUX_CONFIG_DEINIT:
            break;
        default:
            break;
    }
}

/********************************************************************************
* Function Name: handleSwitchPressEvent
*********************************************************************************
* Summary:
*  Configure SBU MUX on switch pressed based on active CC line for Port0/Port1
*
* Parameters:
*  context - USBPD context
*  activeCCline - active CC line for Port0/Port1
*
* Return:
*  None
*
*******************************************************************************/
void handleSwitchPressEvent(cy_stc_usbpd_context_t* context, uint8_t activeCCline)
{
    /* Configure SBU MUX for Port0/port1 based on CC polarity. */
    if (activeCCline == 1)
    {
        /* CC1 of Port0 is Active */
        Cy_USBPD_Mux_SbuSwitchConfigure(context, CY_USBPD_SBU_CONNECT_LSTX, CY_USBPD_SBU_CONNECT_LSRX);
    }
    else if (activeCCline == 2)
    {
        /* CC2 of Port0 is Active */
         Cy_USBPD_Mux_SbuSwitchConfigure(context, CY_USBPD_SBU_CONNECT_LSRX, CY_USBPD_SBU_CONNECT_LSTX);
    }
}

/*******************************************************************************
* Function Name: checkPortStatus
********************************************************************************
*
* Summary:
*  This function is executed to check which Port is active.
*
* Parameters:
*  None
*
* Return:
*  None
*******************************************************************************/
void checkPortStatus()
{
    /* Check CC line status for both the Port0 */
    ccStatusp0 = Cy_USBPD_TypeC_GetCCStatus(&gl_UsbPdPort0Ctx);
#if PMG1_PD_DUALPORT_ENABLE
    /* Check CC line status for both the Port1 */
    ccStatusp1 = Cy_USBPD_TypeC_GetCCStatus(&gl_UsbPdPort1Ctx);
#endif /* PMG1_PD_DUALPORT_ENABLE  */

    /* Checking if Port0 is active or not */
    while (timeoutP0)
    {
        timeoutP0--;
        ccStatusp0 = Cy_USBPD_TypeC_GetCCStatus(&gl_UsbPdPort0Ctx);
        /* For Port0, check if any of the CC line is high or not */
        if(!((ccStatusp0.cc[0] <= 1) && (ccStatusp0.cc[1] <= 1)))
        {
            /* If Port0 is active then break */
            break;
        }
    }

    /* Checking if Port1 is active or not */
#if (PMG1_PD_DUALPORT_ENABLE)
    while (timeoutP1)
    {
        timeoutP1--;
        ccStatusp1 = Cy_USBPD_TypeC_GetCCStatus(&gl_UsbPdPort1Ctx);
        /* For Port1, check if any of the CC line is high or not */
        if(!((ccStatusp1.cc[0] <= 1) && (ccStatusp1.cc[1] <= 1)))
        {
            /* If Port1 is active then break */
            break;
        }
    }
#endif /* PMG1_PD_DUALPORT_ENABLE  */
}

/* End of File */
