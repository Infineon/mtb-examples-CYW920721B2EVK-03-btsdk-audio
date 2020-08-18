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
#include "wiced_bt_audio_insert.h"
#include "app_trace.h"
#include "app_audio_playback.h"
#ifdef COMPRESS_ENABLED
#include "app_audio_opus.h"
#include "wiced_rtos.h"
#endif

#ifdef COMPRESS_ENABLED
wiced_thread_t          *decode_thread = NULL;

struct decode_thread_arg_struct
{
    uint8_t *compress_buffer;
    uint32_t compress_buffer_size;
    int16_t *pcm_buffer;
    uint32_t samples;
    uint32_t freq;
    wiced_event_flags_t *event;
    volatile wiced_bool_t thread_running;
} decode_thread_arg;

enum decode_thread_event_flags {
    DECODE_THREAD_EVENT_GO = 1 << 0,
    DECODE_THREAD_EVENT_START = 1 << 1,
    DECODE_THREAD_EVENT_STOP = 1 << 2,
};

#endif

/*
 * Definitions
 */
#define AUDIO_INSERT_NB_CHANNELS       2
#define AUDIO_INSERT_NB_SAMPLES        (WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO * AUDIO_INSERT_NB_CHANNELS)


typedef struct
{
    wiced_bool_t playing;
    app_audio_playback_callback_t *p_callback;
    uint16_t *p_pcm_buffer;
    uint32_t nb_frames_max;
    uint32_t nb_frames_current;
    int16_t buffer[AUDIO_INSERT_NB_SAMPLES]; /* 128 Frames, 256 Samples, 512 bytes */
} app_audio_playback_cb_t;

/*
 * Local functions
 */
static void app_audio_playback_pcm_data_add(wiced_bt_audio_insert_type_t type);
static void app_audio_playback_pcm_data_add_noncompressed(void);

#ifdef COMPRESS_ENABLED
static void decode_thread_main( uint32_t arg );
static void decode_thread_main_loop(void);
#endif

/*
 * Global variables
 */
static app_audio_playback_cb_t app_audio_playback_cb;

/*
 * app_audio_playback_init
 */
wiced_result_t app_audio_playback_init(app_audio_playback_callback_t *p_callback)
{
    wiced_result_t status;

    APP_TRACE_DBG("\n");

    memset(&app_audio_playback_cb, 0, sizeof(app_audio_playback_cb));

    /* Initialize the Audio Insert library (used for Audio Playback) */
    wiced_bt_audio_insert_init();

    app_audio_playback_cb.p_callback = p_callback;

#ifdef COMPRESS_ENABLED
    decode_thread_arg.pcm_buffer = app_audio_playback_cb.buffer;
    decode_thread_arg.event = wiced_rtos_create_event_flags();
    decode_thread_arg.thread_running = WICED_FALSE;

    status = wiced_rtos_init_event_flags(decode_thread_arg.event);
    if (status != WICED_SUCCESS)
    {
        APP_TRACE_ERR("wiced_rtos_init_event_flags failed\n");
        return status;
    }

    decode_thread = wiced_rtos_create_thread();

    status = wiced_rtos_init_thread(decode_thread, 3, "decode thread", decode_thread_main, 2048, (void*)decode_thread);
    if (WICED_SUCCESS != status)
    {
	    WICED_BT_TRACE("wiced_rtos_init_thread Error %u\n", status);
        return WICED_BT_ERROR;
    }
#endif

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_playback_start
 */
wiced_result_t app_audio_playback_start(uint16_t *p_pcm_buffer, uint32_t nb_frames,
        uint32_t sample_frequency)
{
    wiced_bt_audio_insert_config_t audio_insert_config = {0};
    uint32_t sample_rate = sample_frequency;

    APP_TRACE_DBG("NbFrame:%d SampleFrequency:%d\n", nb_frames, sample_frequency);

    if (app_audio_playback_cb.playing)
    {
        APP_TRACE_ERR("Playback already ongoing\n");
        return WICED_BT_ERROR;
    }

    app_audio_playback_cb.playing = WICED_TRUE;
    app_audio_playback_cb.p_pcm_buffer = p_pcm_buffer;
    app_audio_playback_cb.nb_frames_max = nb_frames;
    app_audio_playback_cb.nb_frames_current = 0;

#ifdef COMPRESS_ENABLED
    app_opus_decode_init((uint8_t *) p_pcm_buffer);

    decode_thread_arg.compress_buffer = (uint8_t *) p_pcm_buffer;
    decode_thread_arg.compress_buffer_size = nb_frames;
    decode_thread_arg.samples = nb_frames;
    decode_thread_arg.freq = sample_frequency;

    wiced_rtos_set_event_flags(decode_thread_arg.event, DECODE_THREAD_EVENT_START);
#else // COMPRESS_ENABLED
    /* Prepare the 1st-time insertion data. */
    app_audio_playback_pcm_data_add_noncompressed();
#endif // COMPRESS_ENABLED

    /* Start Audio Playback (via the Audio Insertion FW Library) */
    audio_insert_config.type                                                    = WICED_BT_AUDIO_INSERT_TYPE_AUDIO;
    audio_insert_config.p_sample_rate                                           = &sample_rate;
    audio_insert_config.insert_data.audio.p_source                              = app_audio_playback_cb.buffer;
    audio_insert_config.insert_data.audio.len                                   = sizeof(app_audio_playback_cb.buffer);
    audio_insert_config.insert_data.audio.overwrite                             = WICED_FALSE;
    audio_insert_config.insert_data.audio.loop                                  = WICED_TRUE;
    audio_insert_config.insert_data.audio.volume_reduce_factor_insert           = 1;
    audio_insert_config.insert_data.audio.volume_reduce_factor_original         = 1;
    audio_insert_config.insert_data.audio.p_source_data_exhausted_callback      = &app_audio_playback_pcm_data_add;
    audio_insert_config.insert_data.audio.stop_insertion_when_source_exhausted  = WICED_FALSE;

    wiced_bt_audio_insert_start(&audio_insert_config);

    return WICED_BT_SUCCESS;
}

/*
 * app_audio_playback_stop
 */
void app_audio_playback_stop(uint32_t *p_nb_frames_played)
{
    APP_TRACE_DBG("\n");

    if (app_audio_playback_cb.playing == WICED_FALSE)
    {
        APP_TRACE_ERR("Playback not started\n");
        return;
    }

    app_audio_playback_cb.playing = WICED_FALSE;

#ifdef COMPRESS_ENABLED
    wiced_rtos_set_event_flags(decode_thread_arg.event, DECODE_THREAD_EVENT_STOP);
#endif

    /* Stop Audio Insertion */
    wiced_bt_audio_insert_stop(WICED_BT_AUDIO_INSERT_TYPE_AUDIO);

    /* Return the number of PCM Frames recorded */
    if (p_nb_frames_played)
    {
        *p_nb_frames_played = app_audio_playback_cb.nb_frames_current;
    }
}

#ifdef COMPRESS_ENABLED
static void decode_thread_main( uint32_t arg )
{
    while (1)
    {
        uint32_t flags_set;
        const wiced_result_t result = wiced_rtos_wait_for_event_flags(
                decode_thread_arg.event, DECODE_THREAD_EVENT_START,
                &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (WICED_SUCCESS != result)
        {
            APP_TRACE_ERR("wiced_rtos_wait_for_event_flags failed. %u\n", result);
            continue;
        }

        decode_thread_arg.thread_running = WICED_TRUE;
        decode_thread_main_loop();
        decode_thread_arg.thread_running = WICED_FALSE;
    }
}

void decode_thread_main_loop(void)
{
    wiced_result_t status;
    int32_t frame_size = 0;
    uint32_t flags_set;
    int16_t *pcm_buffer = decode_thread_arg.pcm_buffer;
    uint8_t *compress_buffer = decode_thread_arg.compress_buffer;
    uint32_t compress_buffer_current = 0;
    uint32_t consumed_size;

    do
    {
        status = app_opus_decode(pcm_buffer, &compress_buffer[compress_buffer_current], WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO, &consumed_size);
        if (status != WICED_SUCCESS)
        {
            APP_TRACE_ERR("app_opus_decode failed. status=%d\n", status);
            return;
        }

        compress_buffer_current += consumed_size;
        app_audio_playback_cb.nb_frames_current = compress_buffer_current;
        status = wiced_rtos_wait_for_event_flags(decode_thread_arg.event,
                DECODE_THREAD_EVENT_GO | DECODE_THREAD_EVENT_STOP,
                &flags_set, WICED_TRUE, WAIT_FOR_ANY_EVENT, WICED_WAIT_FOREVER);
        if (status != WICED_SUCCESS)
        {
            APP_TRACE_ERR("wiced_rtos_wait_for_event_flags failed. status=%d\n", status);
            return;
        }
    } while (0 == (flags_set & DECODE_THREAD_EVENT_STOP));
}
#endif

static void app_audio_playback_pcm_data_add(wiced_bt_audio_insert_type_t type)
{
    uint32_t nb_frames_played;
    app_audio_playback_event_data_t data_event;

    /* Check if this is the end of the Playback buffer */
#ifdef COMPRESS_ENABLED
    if (decode_thread_arg.thread_running == WICED_FALSE ||  app_audio_playback_cb.nb_frames_current >= app_audio_playback_cb.nb_frames_max)
#else // COMPRESS_ENABLED
    if (app_audio_playback_cb.nb_frames_current >= app_audio_playback_cb.nb_frames_max)
#endif // COMPRESS_ENABLED
    {
        APP_TRACE_DBG("PCM Buffer Empty. Stop audio Playback\n");

        /* Stop Audio Playback */
        app_audio_playback_stop(&nb_frames_played);

        /* Tell the application that AudioRecord stopped */
        if (app_audio_playback_cb.p_callback)
        {
            data_event.stopped.nb_frames_played = nb_frames_played;
            app_audio_playback_cb.p_callback(APP_AUDIO_PLAYBACK_EVENT_STOPPED, &data_event);
        }
    }

    /* Prepare the insertion data for next audio insertion. */
#ifdef COMPRESS_ENABLED
    wiced_rtos_set_event_flags(decode_thread_arg.event, DECODE_THREAD_EVENT_GO);
#else // COMPRESS_ENABLED
    app_audio_playback_pcm_data_add_noncompressed();
#endif // COMPRESS_ENABLED
}

static void app_audio_playback_pcm_data_add_noncompressed(void)
{
    uint32_t nb_frames;

    /* Calculate how many PCM Frames the application's buffer can accept */
    nb_frames = app_audio_playback_cb.nb_frames_max - app_audio_playback_cb.nb_frames_current;
    if (nb_frames > WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO)
    {
        nb_frames = WICED_BT_AUDIO_INSERT_PCM_SAMPLE_NB_AUDIO;
    }
    else
    {
        /* The last buffer may contain uninitialized PCM samples */
        memset(app_audio_playback_cb.buffer, 0, sizeof(app_audio_playback_cb.buffer));
    }

    /* Copy the samples to an internal buffer */
    memcpy(app_audio_playback_cb.buffer,
           &app_audio_playback_cb.p_pcm_buffer[app_audio_playback_cb.nb_frames_current * 2],
           nb_frames * AUDIO_INSERT_NB_CHANNELS * 2);

    /* Update the current frame index */
    app_audio_playback_cb.nb_frames_current += nb_frames;
}
