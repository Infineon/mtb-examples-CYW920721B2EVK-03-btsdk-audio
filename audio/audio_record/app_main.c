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
 * This file implements the entry point of the Wiced Application
 */
#include "wiced.h"
#include "wiced_bt_stack.h"
#include "wiced_hal_puart.h"
#include "wiced_hal_gpio.h"
#include "wiced_memory.h"
#include "wiced_timer.h"
#include "sparcommon.h"
#include "app_trace.h"
#include "app_audio_record.h"
#include "app_audio_playback.h"
#ifdef CYW9BT_AUDIO
#include "wiced_audio_manager.h"
#endif

/*
 * Definitions
 */

#ifdef CYW9BT_AUDIO

#if AUDIO_SHIELD_EVK_VER==1
#define APP_MAIN_BUTTON_RECORD              WICED_P00
#define APP_MAIN_BUTTON_PLAYBACK            WICED_P06
#elif AUDIO_SHIELD_EVK_VER==2
#define APP_MAIN_BUTTON_RECORD              WICED_P00
#define APP_MAIN_BUTTON_PLAYBACK            WICED_P04
#elif AUDIO_SHIELD_EVK_VER==3
#define APP_MAIN_BUTTON_RECORD              WICED_P00
#define APP_MAIN_BUTTON_PLAYBACK            WICED_P02
#else
#error unexpected AUDIO_SHIELD_EVK_VER
#endif

#define APP_MAIN_BUTTON_PRESSED_VALUE       0
#endif

#define APP_MAIN_NB_CHANNELS                2

#ifdef COMPRESS_ENABLED
/* Bytes of compressed data. Duration = total_size / packet_size * 20ms = 30000 * 2 / 80 * 20 / 1000 = 30 seconds */
#define APP_MAIN_NB_FRAMES                  30000
#else
/* Number of PCM samples saved. Duration = NbFrames/ Frequency. 50000 = 3.125 seconds */
#define APP_MAIN_NB_FRAMES                  50000
#endif

#define APP_MAIN_NB_SAMPLES                 (APP_MAIN_NB_FRAMES * APP_MAIN_NB_CHANNELS)

#define APP_MAIN_FREQUENCY                  AM_PLAYBACK_SR_16K

typedef struct
{
    uint16_t pcm_buffer[APP_MAIN_NB_SAMPLES];
    uint32_t nb_frames;
#ifdef CYW9BT_AUDIO
    int32_t am_stream_id;
#endif
} app_main_cb_t;


/*
 * Global variables
 */
static app_main_cb_t app_main_cb;


/*
 * Local functions
 */
static wiced_result_t app_main_bt_management_cback(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data);
static void app_main_init(void);
static void app_main_audio_record_callback(app_audio_record_event_t event,
        app_audio_record_event_data_t *p_data);
static void app_main_audio_playback_callback(app_audio_playback_event_t event,
        app_audio_playback_event_data_t *p_data);
#ifdef CYW9BT_AUDIO
static void app_main_gpio_interrupt_handler(void *data, uint8_t pin);
static wiced_result_t app_main_audio_manager_init(void);
static wiced_result_t app_main_audio_manager_start(void);
static wiced_result_t app_main_audio_manager_stop(void);
#endif

/*
 *  Application Start, i.e., entry point to the application.
 */
APPLICATION_START( )
{
#ifdef WICED_BT_TRACE_ENABLE
    /* Initialize PUART */
    wiced_hal_puart_init();
    /* Set PUART baudrate */
    wiced_hal_puart_configuration(3000000, PARITY_NONE, STOP_BIT_2);
    /* Set the debug trace route to WICED_ROUTE_DEBUG_TO_PUART */
    wiced_set_debug_uart(WICED_ROUTE_DEBUG_TO_PUART);
#endif /* WICED_BT_TRACE_ENABLE */

    WICED_BT_TRACE("*****************************************\n");
    WICED_BT_TRACE("* WICED AUDIO RECORD APPLICATION STARTED *\n");
    WICED_BT_TRACE("*****************************************\n");

    /* Start BT stack */
    wiced_bt_stack_init(app_main_bt_management_cback, NULL, NULL);
}

/*
 * app_main_bt_management_cback
 */
static wiced_result_t app_main_bt_management_cback(wiced_bt_management_evt_t event,
        wiced_bt_management_evt_data_t *p_event_data)
{
    wiced_result_t status = WICED_SUCCESS;

    APP_TRACE_DBG("%d\n", event);

    switch (event)
    {
    /* Bluetooth  stack enabled */
    case BTM_ENABLED_EVT:

        APP_TRACE_DBG("Free Bytes:%d\n", wiced_memory_get_free_bytes());

        /* Start the application */
        app_main_init();

        /* Initialize Audio Recording */
        app_audio_record_init(app_main_audio_record_callback);

        /* Initialize Audio Playback */
        app_audio_playback_init(app_main_audio_playback_callback);

#ifdef CYW9BT_AUDIO
        /* Initialize Codec library */
        app_main_audio_manager_init();
#endif
        break;

    default:
        /* Ignore other events */
        break;
    }
    return status;
}

/*
 * app_init
 */
static void app_main_init(void)
{
    APP_TRACE_DBG("\n");

    memset(&app_main_cb, 0, sizeof(app_main_cb));

#ifdef CYW9BT_AUDIO
    /* Configure the first button (SW15) to control Recording */
    wiced_hal_gpio_register_pin_for_interrupt(APP_MAIN_BUTTON_RECORD,
            app_main_gpio_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(APP_MAIN_BUTTON_RECORD,
            GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_BOTH_EDGE,
            GPIO_PIN_OUTPUT_LOW);

    /* Configure the second button (SW17) to control Playback */
    wiced_hal_gpio_register_pin_for_interrupt(APP_MAIN_BUTTON_PLAYBACK,
            app_main_gpio_interrupt_handler, NULL);
    wiced_hal_gpio_configure_pin(APP_MAIN_BUTTON_PLAYBACK,
            GPIO_INPUT_ENABLE | GPIO_PULL_UP | GPIO_EN_INT_BOTH_EDGE,
            GPIO_PIN_OUTPUT_LOW);
#endif
}


/*
 * app_main_audio_record_callback
 */
static void app_main_audio_record_callback(app_audio_record_event_t event,
        app_audio_record_event_data_t *p_data)
{
    switch(event)
    {
    case APP_AUDIO_RECORD_EVENT_STOPPED:
        APP_TRACE_DBG("Record Stopped NbFramesRecorded:%d\n", p_data->stopped.nb_frames_recorded);
        app_main_cb.nb_frames = p_data->stopped.nb_frames_recorded;
#ifdef CYW9BT_AUDIO
        app_main_audio_manager_stop();  /* Stop the, external, Audio Codec */
#endif
        break;

    default:
        APP_TRACE_ERR("unknown event:%d\n", event);
        break;
    }
}


/*
 * app_main_audio_playback_callback
 */
static void app_main_audio_playback_callback(app_audio_playback_event_t event,
        app_audio_playback_event_data_t *p_data)
{
    switch(event)
    {
    case APP_AUDIO_PLAYBACK_EVENT_STOPPED:
        APP_TRACE_DBG("Record Stopped NbFramesPlayed:%d\n", p_data->stopped.nb_frames_played);
#ifdef CYW9BT_AUDIO
        app_main_audio_manager_stop();  /* Stop the, external, Audio Codec */
#endif
        break;

    default:
        APP_TRACE_ERR("unknown event:%d\n", event);
        break;
    }
}

#ifdef CYW9BT_AUDIO
/*
 * app_main_gpio_interrupt_handler
 */
static void app_main_gpio_interrupt_handler(void *data, uint8_t pin)
{
    uint32_t value;
    uint32_t nb_frames;
    wiced_result_t status;

    value = wiced_hal_gpio_get_pin_input_status(pin);

    APP_TRACE_DBG("Button pin:%d value:%d\n", pin, value);

    /* Record button pressed/released */
    if (pin == APP_MAIN_BUTTON_RECORD)
    {
        if (value == APP_MAIN_BUTTON_PRESSED_VALUE)
        {
            APP_TRACE_DBG("Record Start...\n");

            /* Start the, external, Audio Codec */
            status = app_main_audio_manager_start();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_main_audio_manager_start failed\n");

            /* Start the Audio Record */
            status = app_audio_record_start(app_main_cb.pcm_buffer,
                    APP_MAIN_NB_FRAMES, APP_MAIN_FREQUENCY);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_audio_record_start failed\n");
        }
        else
        {
            APP_TRACE_DBG("Record Stop...\n");

            /* Stop the, external, Audio Codec */
            status = app_main_audio_manager_stop();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_main_audio_manager_stop failed\n");

            /* Stop the Audio Record */
            status = app_audio_record_stop(&nb_frames);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_audio_record_stop failed\n");
            else
            {
                APP_TRACE_DBG("NbFramesRecorded:%d\n", nb_frames);
                app_main_cb.nb_frames = nb_frames;
            }
        }
    }
    /* Playback button pressed/released */
    else if (pin == APP_MAIN_BUTTON_PLAYBACK)
    {
        if (value == APP_MAIN_BUTTON_PRESSED_VALUE)
        {
            APP_TRACE_DBG("Playback Start...\n");

            /* Start the, external, Audio Codec */
            status = app_main_audio_manager_start();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_main_audio_manager_start failed\n");

            /* Start the Audio Playback (to play the previously recorded audio) */
            status = app_audio_playback_start(app_main_cb.pcm_buffer, app_main_cb.nb_frames,
                    APP_MAIN_FREQUENCY);
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_audio_playback_start failed\n");
        }
        else
        {
            APP_TRACE_DBG("Playback Stop...\n");

            /* Stop the, external, Audio Codec */
            status = app_main_audio_manager_stop();
            if (status != WICED_BT_SUCCESS)
                APP_TRACE_ERR("app_main_audio_manager_stop failed\n");

            /* Stop the Audio Playback (to play the previously recorded audio */
            app_audio_playback_stop(&nb_frames);
            APP_TRACE_DBG("NbFramesPlayed:%d\n", nb_frames);
        }
    }

    /* clear the interrupt status */
    wiced_hal_gpio_clear_pin_interrupt_status(pin);
}

/*
 * app_main_audio_manager_init
 * Initialize the external Audio Codec library
 */
static wiced_result_t app_main_audio_manager_init(void)
{
    APP_TRACE_DBG("\n");

    /* Initialize Audio Manager */
    wiced_am_init();

    app_main_cb.am_stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;

    return WICED_BT_SUCCESS;
}

/*
 * app_main_audio_manager_start
 * Starts the external Audio Codec.
 * This function can be used to to start Audio Recording and Playback
 */
static wiced_result_t app_main_audio_manager_start(void)
{
    audio_config_t audio_config;
    wiced_result_t status = WICED_BT_ERROR;
    uint32_t output_device;

    APP_TRACE_DBG("\n");

    /* Open and configure the Codec in HandsFree mode */
    app_main_cb.am_stream_id = wiced_am_stream_open(HFP);
    if (app_main_cb.am_stream_id != WICED_AUDIO_MANAGER_STREAM_ID_INVALID)
    {
        APP_TRACE_DBG("am_stream_id:%d\n", app_main_cb.am_stream_id);
        memset(&audio_config, 0, sizeof(audio_config));
        audio_config.sr = AM_PLAYBACK_SR_16K;
        audio_config.channels = 2;
        audio_config.bits_per_sample = DEFAULT_BITSPSAM;
        audio_config.volume = 5;
        audio_config.mic_gain = 5;
        audio_config.sink = AM_HEADPHONES;

        /* Configure Codec HFP Stream */
        status = wiced_am_stream_set_param(app_main_cb.am_stream_id,
            AM_AUDIO_CONFIG, &audio_config);
        if (status != WICED_BT_SUCCESS)
            APP_TRACE_ERR("set_param(AM_AUDIO_CONFIG) failed\n");

        /* Configure the Codec Output */
        output_device = AM_HEADPHONES;
        status = wiced_am_stream_set_param(app_main_cb.am_stream_id,
                AM_IO_DEVICE, &output_device);
        if (status != WICED_BT_SUCCESS)
            APP_TRACE_ERR("set_param(AM_IO_DEVICE) failed\n");

        /* Start Codec HFP Stream */
        status = wiced_am_stream_start(app_main_cb.am_stream_id);
        if (status != WICED_SUCCESS)
            APP_TRACE_ERR("start failed\n");
    }
    else
    {
        APP_TRACE_ERR("wiced_am_stream_open failed\n");
    }
    return status;
}

/*
 * app_main_audio_manager_stop
 * Stop the external Audio Codec.
 * This function can be used to to Stop Audio Recording and Playback
 */
static wiced_result_t app_main_audio_manager_stop(void)
{
    wiced_result_t status;

    APP_TRACE_DBG("\n");

    /* Stop the Audio Codec */
    status = wiced_am_stream_stop(app_main_cb.am_stream_id);
    if (status != WICED_BT_SUCCESS)
        APP_TRACE_ERR("stop(HFP) failed\n");

    /* Close the Audio Codec */
    status = wiced_am_stream_close(app_main_cb.am_stream_id);
    if (status != WICED_BT_SUCCESS)
        APP_TRACE_ERR("close(HFP) failed\n");
    app_main_cb.am_stream_id = WICED_AUDIO_MANAGER_STREAM_ID_INVALID;

    return status;
}
#endif /* CYW9BT_AUDIO */

/*
 * app_dump_hex
 */
void app_dump_hex(void *p_buf, uint32_t len)
{
    uint32_t i, j;
    uint8_t *p = p_buf;
    char buff1[100];

    while (len != 0)
    {
        memset(buff1, 0, sizeof(buff1));
        for (i = 0; i < len && i < 32; i++)
        {
            int s1 = (*p & 0xf0) >> 4;
            int s2 = *p & 0x0f;
            buff1[i * 3]     = (s1 >= 0 && s1 <= 9) ? s1 + '0' : s1 - 10 + 'A';
            buff1[i * 3 + 1] = (s2 >= 0 && s2 <= 9) ? s2 + '0' : s2 - 10 + 'A';
            buff1[i * 3 + 2] = ' ';
            p++;
        }
        len -= i;
        if (len != 0)
            WICED_BT_TRACE("%s\n", buff1);
    }
    WICED_BT_TRACE("%s\n", buff1);
}
