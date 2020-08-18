/*
 *  Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 *  Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 *  This software, including source code, documentation and related
 *  materials ("Software"), is owned by Cypress Semiconductor Corporation
 *  or one of its subsidiaries ("Cypress") and is protected by and subject to
 *  worldwide patent protection (United States and foreign),
 *  United States copyright laws and international treaty provisions.
 *  Therefore, you may use this Software only as provided in the license
 *  agreement accompanying the software package from which you
 *  obtained this Software ("EULA").
 *  If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 *  non-transferable license to copy, modify, and compile the Software
 *  source code solely for use in connection with Cypress's
 *  integrated circuit products. Any reproduction, modification, translation,
 *  compilation, or representation of this Software except as specified
 *  above is prohibited without the express written permission of Cypress.
 * 
 *  Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 *  reserves the right to make changes to the Software without notice. Cypress
 *  does not assume any liability arising out of the application or use of the
 *  Software or any product or circuit described in the Software. Cypress does
 *  not authorize its products for use in any products where a malfunction or
 *  failure of the Cypress product may reasonably be expected to result in
 *  significant property damage, injury or death ("High Risk Product"). By
 *  including Cypress's product in a High Risk Product, the manufacturer
 *  of such system or application assumes all risk of such use and in doing
 *  so agrees to indemnify Cypress against all liability.
 */

#pragma once

#include <stdint.h>
#include "wiced.h"

typedef enum
{
    APP_AUDIO_RECORD_EVENT_STOPPED,
} app_audio_record_event_t;

/*
 * Structures
 */
typedef struct
{
    uint32_t nb_frames_recorded;
} app_audio_record_event_stopped_t;

typedef union
{
    app_audio_record_event_stopped_t stopped;
} app_audio_record_event_data_t;

typedef void (app_audio_record_callback_t)(app_audio_record_event_t event,
        app_audio_record_event_data_t *p_data);

/*
 * app_audio_record_init
 */
wiced_result_t app_audio_record_init(app_audio_record_callback_t *p_callack);

/*
 * app_audio_record_start
 */
wiced_result_t app_audio_record_start(uint16_t *p_pcm_buffer, uint32_t nb_frames,
        uint32_t sample_frequency);

/*
 * app_audio_record_stop
 */
wiced_result_t app_audio_record_stop(uint32_t *p_nb_frames_recorded);
