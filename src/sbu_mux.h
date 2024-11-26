/*******************************************************************************
* File Name: sbu_mux.h
* Version 1.0
*
* Description:
*  This file contains the macro declaration and function prototypes used in
*  the main source code
*
* Related Document: See README.md
*
*******************************************************************************
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
#ifndef SBU_MUX_H_
#define SBU_MUX_H_

/*******************************************************************************
 * Include header files
 ******************************************************************************/
#include "stdint.h"
#include "cybsp.h"
#include "cy_pdl.h"

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Flag to detect switch press event */
extern volatile uint8_t SwitchPressFlag;

/* USB PD context for Port0 */
extern cy_stc_usbpd_context_t gl_UsbPdPort0Ctx;

/* Holds CC status for Port0 */
extern cy_pd_cc_state_t ccStatusp0;

#if PMG1_PD_DUALPORT_ENABLE
/* USB PD context for Port1 */
extern cy_stc_usbpd_context_t gl_UsbPdPort1Ctx;

/* Holds CC status for Port1 */
extern cy_pd_cc_state_t ccStatusp1;
#endif /* PMG1_PD_DUALPORT_ENABLE */

/*****************************************************************************
 * Data Struct Definition
 ****************************************************************************/
/**
  @ typedef mux_select_t
  @ brief Possible settings for the Type-C Data MUX.
  @ note This type should be extended to cover all possible modes for the MUX.
 */
typedef enum
{
    MUX_CONFIG_ISOLATE,                  /**< Isolate configuration. */
    MUX_CONFIG_SAFE,                     /**< USB Safe State (USB 2.0 lines remain active) */
    MUX_CONFIG_SS_ONLY,                  /**< USB SS configuration. */
    MUX_CONFIG_INIT,                     /**< Enables MUX functionality. */
    MUX_CONFIG_DEINIT                    /**< Disables MUX functionality. */
} mux_select_t;

/*****************************************************************************
 * Global Function Declaration
 *****************************************************************************/
void mux_ctrl_set_cfg(cy_stc_usbpd_context_t* context, mux_select_t cfg, uint8_t activeCClineP0);
void handleSwitchPressEvent(cy_stc_usbpd_context_t* context, uint8_t activeCCline);
void checkPortStatus();
#endif /* SBU_MUX_H_ */

/* End of File */
