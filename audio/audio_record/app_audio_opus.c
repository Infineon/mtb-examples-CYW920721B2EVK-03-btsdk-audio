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
#ifdef COMPRESS_ENABLED
#include "wiced_bt_trace.h"
#include "app_audio_opus.h"
#include "celt/celt_encoder_api.h"
#include "celt/celt_decoder_api.h"
#include "celt/opus.h"
#include "wiced_rtos.h"

/* Defines */
#ifndef _countof
#define _countof(x) (sizeof(x) / sizeof(x[0]))
#endif

/* Local variables */
uint8_t app_opus_packet[OPUS_ENC_PACKET_SIZE + 8];           // plus 8 bytes of headser
int16_t pcm_buffer_mono[OPUS_ENC_FRAME_SIZE];

int16_t pcm_ring_buffer[OPUS_ENC_FRAME_SIZE * 2];
uint32_t pcm_ring_buffer_head = 0;
uint32_t pcm_ring_buffer_current = 0;
uint8_t pcm_ring_buffer_full = 0;


CELT_ENC_PARAMS celt_enc_params = {
    .sampling_rate = OPUS_ENC_SAMPLING_RATE,
    .channels = 1,
    .bitrate = OPUS_ENC_BITRATE,
    .complexity = 3,
    .use_vbr = 0,                       // must be 0 (CBR only)
    .use_cvbr = 0,                      // must be 0 (CBR only)
    .frame_size = OPUS_ENC_FRAME_SIZE,
    .pcmBuffer = pcm_buffer_mono,
    .packet = app_opus_packet,
    .enc_handler = NULL,
};

CELT_DEC_PARAMS celt_dec_params = {
    .sampling_rate = OPUS_ENC_SAMPLING_RATE,        /* 8k, 16k, 24k or 48k */
    .channels = 1,                                  /* mono or streo */
    .pkt_len = OPUS_ENC_PACKET_SIZE,                /* Input packet length (bytes) */
    .pcmBuffer = pcm_buffer_mono,                   /* Pointer of output buffer */
    .packet = app_opus_packet,                      /* Pointer of input buffer */
    .frame_status = 0,                              /* Frame status: 0:GOOD, 1:BAD(lost)  */
    .frame_size = OPUS_ENC_FRAME_SIZE,              /* PCM samples per frame per channel, needed for PLC init*/
    .enable_plc = 0,                                /* Enable PLC: 1:enable, 0:disable */
};

/* Local functions */
static void pcm_ring_buffer_init(void);
static opus_status_t pcm_ring_buffer_enqueue(int16_t *input, uint32_t num);
static opus_status_t pcm_ring_buffer_dequeue(int16_t *output, uint32_t num);
static void pcm_stereo_to_mono(int16_t *out_buffer, int16_t *in_buffer, uint32_t samples);
static void pcm_mono_to_stereo(int16_t *out_buffer, int16_t *in_buffer, uint32_t samples);
static opus_status_t app_opus_encode_frame(uint8_t *opus_buffer, int16_t *pcm_buffer, uint32_t samples);

opus_status_t app_opus_encode_init(void)
{
    int32_t celt_ret;

    pcm_ring_buffer_init();

    celt_ret = CELT_Encoder_Init(&celt_enc_params);
    if (celt_ret != 0)
    {
        WICED_BT_TRACE("CELT_Encoder_Init ERROR!! celt_ret=%d\n", celt_ret);
        return OPUS_STATUS_ERROR;
    }
    return OPUS_STATUS_SUCCESS;
}

opus_status_t app_opus_encode(uint8_t *opus_buffer, int16_t *pcm_buffer_stero, uint32_t samples)
{
    opus_status_t status;
    uint32_t packet_length;

    pcm_stereo_to_mono(pcm_buffer_mono, pcm_buffer_stero, samples);
    status = pcm_ring_buffer_enqueue(pcm_buffer_mono, samples);

    if (status != OPUS_STATUS_SUCCESS)
    {
        WICED_BT_TRACE("pcm_ring_buffer_enqueue failed\n");
        return status;
    }

    status = pcm_ring_buffer_dequeue(pcm_buffer_mono, OPUS_ENC_FRAME_SIZE);
    if (status != OPUS_STATUS_SUCCESS && status != OPUS_STATUS_PENDING)
    {
        WICED_BT_TRACE("pcm_ring_buffer_dequeue failed\n");
        return status;
    }

    if (status == OPUS_STATUS_SUCCESS)
    {
        status = app_opus_encode_frame(opus_buffer, pcm_buffer_mono, samples);
        if (status != OPUS_STATUS_SUCCESS)
        {
            WICED_BT_TRACE("app_opus_encode_frame failed status=%d\n");
            return status;
        }
    }

    return status;
}

static uint32_t pcm_ring_buffer_get_size(void)
{
    if (pcm_ring_buffer_full)
    {
        return _countof(pcm_ring_buffer);
    }
    else
    {
        return (pcm_ring_buffer_current + _countof(pcm_ring_buffer) - pcm_ring_buffer_head) % _countof(pcm_ring_buffer);
    }
}

static void pcm_ring_buffer_init(void)
{
    pcm_ring_buffer_head = 0;
    pcm_ring_buffer_current = 0;
    pcm_ring_buffer_full = 0;
}

static opus_status_t pcm_ring_buffer_enqueue(int16_t *input, uint32_t num)
{
    uint32_t nf_used = pcm_ring_buffer_get_size();
    uint32_t nf_tail;

    if (pcm_ring_buffer_full)
    {
        WICED_BT_TRACE("PCM ring buffer full\n");
        /* Should not happen */
        return OPUS_STATUS_ERROR;
    }

    if (num > _countof(pcm_ring_buffer) - nf_used)
    {
        /* no room */
        WICED_BT_TRACE("pcm_ring_buffer_enqueue: nf_used = %d\n", nf_used);
        /* Should not happen */
        return OPUS_STATUS_ERROR;
    }

    // Copy input data to 'current' position
    nf_tail = (num > _countof(pcm_ring_buffer) - pcm_ring_buffer_current) ? _countof(pcm_ring_buffer) - pcm_ring_buffer_current : num;
    memcpy(&pcm_ring_buffer[pcm_ring_buffer_current], input, nf_tail * sizeof(input[0]));

    // Update head and current positions
    pcm_ring_buffer_current = (pcm_ring_buffer_current + num) % _countof(pcm_ring_buffer);
    num -= nf_tail;

    // Copy rest input data if tail is full
    if (num)
    {
        memcpy(pcm_ring_buffer, &input[nf_tail], num * sizeof(input[0]));
        num = 0;
    }

    if (pcm_ring_buffer_current == pcm_ring_buffer_head)
    {
        pcm_ring_buffer_full = 1;
    }

    //WICED_BT_TRACE("head=%d current = %d\n", pcm_ring_buffer_head, pcm_ring_buffer_current);

    return OPUS_STATUS_SUCCESS;
}

static opus_status_t pcm_ring_buffer_dequeue(int16_t *output, uint32_t num)
{
    uint32_t total_size = num * sizeof(output[0]);
    int16_t *ptr;
    uint32_t copy_size;

    if (num == 0)
    {
        return OPUS_STATUS_INVALID_PARAMETERS;
    }

    if (!pcm_ring_buffer_full && pcm_ring_buffer_current == pcm_ring_buffer_head)
    {
        WICED_BT_TRACE("PCM ring buffer empty\n");
        return OPUS_STATUS_ERROR;
    }

    if (num > pcm_ring_buffer_get_size())
    {
        // not enough
        return OPUS_STATUS_PENDING;
    }

    ptr = &pcm_ring_buffer[pcm_ring_buffer_head];

    if (pcm_ring_buffer_current > pcm_ring_buffer_head)
    {
        // 1 continuous buffer
        memcpy(output, ptr, total_size);
    }
    else
    {
        // 2 separated buffers, tail and head
        copy_size = (_countof(pcm_ring_buffer) - pcm_ring_buffer_head) * sizeof(output[0]);
        memcpy(output, ptr, copy_size);

        if (total_size - copy_size > 0)
        {
            memcpy((uint8_t*) output + copy_size, pcm_ring_buffer, total_size - copy_size);
        }
    }

    // Update positions
    pcm_ring_buffer_head = (pcm_ring_buffer_head + num) % _countof(pcm_ring_buffer);
    pcm_ring_buffer_full = 0;
    //WICED_BT_TRACE("head=%d current=%d\n", pcm_ring_buffer_head, pcm_ring_buffer_current);

    return OPUS_STATUS_SUCCESS;
}

static void pcm_stereo_to_mono(int16_t *out_buffer, int16_t *in_buffer, uint32_t samples)
{
    int i;

    for (i = 0; i < samples; i++)
    {
        *out_buffer = *in_buffer;
        out_buffer++;
        in_buffer += 2;
    }
}

static void pcm_mono_to_stereo(int16_t *out_buffer, int16_t *in_buffer, uint32_t samples)
{
    int i;

    for (i = 0; i < samples; i++)
    {
        out_buffer[0] = *in_buffer;
        out_buffer[1] = 0;
        out_buffer += 2;
        in_buffer++;
    }
}

static opus_status_t app_opus_encode_frame(uint8_t *opus_buffer, int16_t *pcm_buffer, uint32_t samples)
{
    int32_t celt_ret;
    uint32_t packet_length;

    celt_enc_params.pcmBuffer = pcm_buffer;
    celt_ret = CELT_Encoder(&celt_enc_params);
    if (celt_ret < 8)
    {
        WICED_BT_TRACE("CELT_Encoder returns %d\n", celt_ret);
        return OPUS_STATUS_ERROR;
    }
    packet_length = SWAP_ENDIAN_32(*((uint32_t*) app_opus_packet));

    // Skip 8-byte length and 8-byte final_range
    memcpy(opus_buffer, app_opus_packet + 8, packet_length);

    return OPUS_STATUS_SUCCESS;
}

void app_opus_decode_init(uint8_t *data)
{
    int32_t celt_ret;
    OpusEncoder *enc;

    pcm_ring_buffer_init();

    celt_dec_params.frame_size = celt_opus_packet_get_samples_per_frame(data, OPUS_ENC_SAMPLING_RATE);
    WICED_BT_TRACE("decoder init: frame_size=%d\n", celt_dec_params.frame_size);

    celt_ret = CELT_Decoder_Init(&celt_dec_params);
    if (celt_ret != 0)
    {
        WICED_BT_TRACE("CELT_Decoder_Init ERROR!! celt_ret=%d\n", celt_ret);
        return;
    }
}

wiced_result_t app_opus_decode(int16_t *pcm_buffer, uint8_t *opus_encode_buffer, uint32_t request_samples, uint32_t *consumed_size)
{
    int32_t frame_size = 0;
    wiced_bool_t ret;
    *consumed_size = 0;
    int i = 0;;

    while (_countof(pcm_ring_buffer) - pcm_ring_buffer_get_size() >= OPUS_ENC_FRAME_SIZE)
    {
        celt_dec_params.packet = opus_encode_buffer;
        celt_dec_params.pcmBuffer = &pcm_ring_buffer[pcm_ring_buffer_current];

        //WICED_BT_TRACE("[%d]head=%d cur=%d\n", i, pcm_ring_buffer_head, pcm_ring_buffer_current);
        frame_size = CELT_Decoder(&celt_dec_params);
        if (frame_size < 0)
        {
            WICED_BT_TRACE("CELT_Decoder failed. frame_size=%d\n", frame_size);
            return WICED_ERROR;
        }

        pcm_ring_buffer_current = (pcm_ring_buffer_current + frame_size) % _countof(pcm_ring_buffer);
        if (pcm_ring_buffer_current == pcm_ring_buffer_head)
        {
            pcm_ring_buffer_full = 1;
        }
        opus_encode_buffer += OPUS_ENC_PACKET_SIZE;
        *consumed_size += OPUS_ENC_PACKET_SIZE;
        i++;
    }

    pcm_mono_to_stereo(pcm_buffer, &pcm_ring_buffer[pcm_ring_buffer_head], request_samples);
    pcm_ring_buffer_head = (pcm_ring_buffer_head + request_samples) % _countof(pcm_ring_buffer);
    pcm_ring_buffer_full = 0;
    //WICED_BT_TRACE("PCM:head=%d cur=%d\n", pcm_ring_buffer_head, pcm_ring_buffer_current);

    return WICED_SUCCESS;
}
#endif /* COMPRESS_ENABLED */
