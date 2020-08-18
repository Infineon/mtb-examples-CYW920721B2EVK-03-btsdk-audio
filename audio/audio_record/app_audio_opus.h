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
#define OPUS_ENC_SAMPLING_RATE  16000
#define OPUS_ENC_BITRATE        32000
#define OPUS_ENC_FRAME_SIZE     320     // frame_size must be equal to (sampling_rate / N)
                                        // where N = 400, 200, 100, 50, 25, 50/3
#define OPUS_ENC_PACKET_SIZE    (OPUS_ENC_BITRATE * 20 / 1000 / 8) // 20ms

typedef enum
{
    OPUS_STATUS_SUCCESS             = 0,
    OPUS_STATUS_PENDING             = 1,
    OPUS_STATUS_ERROR               = 2,
    OPUS_STATUS_INVALID_PARAMETERS  = 3,
} opus_status_t;

#define OPUS_PACKET_LENGTH_32KBPS   80
opus_status_t app_opus_encode_init(void);
opus_status_t app_opus_encode(uint8_t *opus_buffer, int16_t *pcm_buffer_stero, uint32_t samples);
void app_opus_decode_init(uint8_t *data);
wiced_result_t app_opus_decode(int16_t *pcm_buffer, uint8_t *opus_encode_buffer, uint32_t request_samples, uint32_t *consumed_size);
