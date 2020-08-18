/*
 * Copyright 2016-2020, Cypress Semiconductor Corporation or a subsidiary of
 * Cypress Semiconductor Corporation. All Rights Reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software"), is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
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
 */

/** @file
 *
 * This file implements the Audio Record
 */
#include "wiced.h"
#include "wiced_bt_audio_record.h"
#include "app_trace.h"
#include "app_audio_record.h"
#include "app_audio_opus.h"


/*
 * Definitions
 */
#define AUDIO_RECORD_NB_CHANNELS        2

typedef struct
{
    wiced_bool_t recording;
    app_audio_record_callback_t *p_callback;
    uint16_t *p_pcm_buffer;
    uint32_t nb_frames_max;
    uint32_t nb_frames_current;
} app_audio_record_cb_t;

/*
 * Local functions
 */
static void app_audio_record_callback(wiced_bt_audio_record_event_t event,
        wiced_bt_audio_record_event_data_t *p_data);

/*
 * Global variables
 */
static app_audio_record_cb_t app_audio_record_cb;

/*
 * app_audio_record_init
 */
wiced_result_t app_audio_record_init(app_audio_record_callback_t *p_callback)
{
    wiced_result_t status;

    APP_TRACE_DBG("\n");

    memset(&app_audio_record_cb, 0, sizeof(app_audio_record_cb));

    /* Initialize the Audio REcode library */
    status = wiced_bt_audio_record_init(app_audio_record_callback);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_audio_record_init failed\n");
        return status;
    }

    app_audio_record_cb.p_callback = p_callback;

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_record_start
 */
wiced_result_t app_audio_record_start(uint16_t *p_pcm_buffer, uint32_t nb_frames,
        uint32_t sample_frequency)
{
    wiced_result_t status;

    APP_TRACE_DBG("NbFrameMax:%d SampleFrequency:%d\n", nb_frames, sample_frequency);

    if (app_audio_record_cb.recording)
    {
        APP_TRACE_ERR("Recording already ongoing\n");
        return WICED_BT_ERROR;
    }

#ifdef COMPRESS_ENABLED
    app_opus_encode_init();
#endif

    app_audio_record_cb.recording = WICED_TRUE;
    app_audio_record_cb.p_pcm_buffer = p_pcm_buffer;
#ifdef COMPRESS_ENABLED
    app_audio_record_cb.nb_frames_max = nb_frames * 2 * 2; /* Maximum bytes */
#else
    app_audio_record_cb.nb_frames_max = nb_frames;
#endif
    app_audio_record_cb.nb_frames_current = 0;

    /* Start Audio Recording */
    status = wiced_bt_audio_record_enable(WICED_TRUE, &sample_frequency);
    if (status != WICED_BT_SUCCESS)
    {
        app_audio_record_cb.recording = WICED_FALSE;
        APP_TRACE_ERR("wiced_bt_audio_record_enable failed status:%d\n", status);
        return status;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_record_stop
 */
wiced_result_t app_audio_record_stop(uint32_t *p_nb_frames_recorded)
{
    wiced_result_t status;

    APP_TRACE_DBG("\n");

    if (app_audio_record_cb.recording == WICED_FALSE)
    {
        APP_TRACE_ERR("Recording not started\n");
        return WICED_BT_ERROR;
    }

    app_audio_record_cb.recording = WICED_FALSE;

    /* Stop Audio Recording */
    status = wiced_bt_audio_record_enable(WICED_FALSE, NULL);
    if (status != WICED_BT_SUCCESS)
    {
        APP_TRACE_ERR("wiced_bt_audio_record_enable failed status:%d\n", status);
        return status;
    }

    /* Return the number of PCM Frames recorded */
    if (p_nb_frames_recorded)
    {
        *p_nb_frames_recorded = app_audio_record_cb.nb_frames_current;
    }

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_record_callback
 */
static void app_audio_record_callback(wiced_bt_audio_record_event_t event,
        wiced_bt_audio_record_event_data_t *p_data)
{
    uint32_t nb_frames;
    wiced_result_t status;
    uint32_t nb_frames_recorded;
    app_audio_record_event_data_t data_event;
#ifdef COMPRESS_ENABLED
    uint8_t *compress_buffer = (uint8_t *) app_audio_record_cb.p_pcm_buffer;
    opus_status_t opus_status;
#endif
    wiced_bool_t record_stop = FALSE;

    switch(event)
    {
    /* FW sends information about the PCM stream */
    case WICED_BT_AUDIO_RECORD_EVT_AUDIO_INFO:
        APP_TRACE_DBG("AUDIO_INFO sample_rate:%d\n", p_data->audio_info.sample_rate);
        break;

    /* FW passes the recorded PCM samples */
    case WICED_BT_AUDIO_RECORD_EVT_DATA_READY:
        APP_TRACE_DBG("DATA_READY len:%d\n", p_data->data_ready.data_len);
        nb_frames = p_data->data_ready.data_len / 2;

#ifdef COMPRESS_ENABLED
        opus_status = app_opus_encode(&compress_buffer[app_audio_record_cb.nb_frames_current],
                        p_data->data_ready.p_data, nb_frames);
        if (opus_status == OPUS_STATUS_SUCCESS)
        {
            app_audio_record_cb.nb_frames_current += OPUS_ENC_PACKET_SIZE;
        }

        if (opus_status != OPUS_STATUS_PENDING && opus_status != OPUS_STATUS_SUCCESS)
        {
            record_stop = TRUE;
        }
#else
        /* The data_len field is in number of PCM Samples. Convert it in number of Frames */
        if ((app_audio_record_cb.nb_frames_current + nb_frames) > app_audio_record_cb.nb_frames_max)
        {
            nb_frames = app_audio_record_cb.nb_frames_max - app_audio_record_cb.nb_frames_current;
        }
        memcpy(&app_audio_record_cb.p_pcm_buffer[app_audio_record_cb.nb_frames_current * 2],
                p_data->data_ready.p_data, nb_frames * 2 * AUDIO_RECORD_NB_CHANNELS);
        app_audio_record_cb.nb_frames_current += nb_frames;
#endif

        if (record_stop || app_audio_record_cb.nb_frames_current >= app_audio_record_cb.nb_frames_max)
        {
            APP_TRACE_DBG("PCM Buffer full. Stop audio Recording\n");
            status = app_audio_record_stop(&nb_frames_recorded);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_audio_record_stop failed\n");
            if (app_audio_record_cb.p_callback)
            {
                /* Tell the application that AudioRecord stopped */
                data_event.stopped.nb_frames_recorded = nb_frames_recorded;
                app_audio_record_cb.p_callback(APP_AUDIO_RECORD_EVENT_STOPPED, &data_event);
            }
        }
        break;

    default:
        APP_TRACE_ERR("unknown event:%d\n", event);
        break;
    }
}
