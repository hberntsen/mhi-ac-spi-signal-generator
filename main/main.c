#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "spi_rmt";

/* ── Pin / timing config ─────────────────────────────────────────────────── */
#define GPIO_CLK             6
#define GPIO_MOSI            7
#define RMT_RESOLUTION_HZ   40000000U  /* 40 MHz → 25 ns / tick               */
#define FRAME_PERIOD_MS     50          /* repeat period; 50 ms ≈ captured rate */

static const rmt_symbol_word_t clk_symbols[] = {
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9980,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9940,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9980,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9980,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9980,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9990,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=9990,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=10090,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=9890,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=10000,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=720},
{.level0=1,.duration0=610,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=620,.level1=0,.duration1=710},
{.level0=1,.duration0=610,.level1=1,.duration1=0},
};
static const rmt_symbol_word_t mosi_symbols[] = {
{.level0=1,.duration0=1950,.level1=0,.duration1=1370},
{.level0=1,.duration0=2610,.level1=0,.duration1=1380},
{.level0=1,.duration0=2610,.level1=0,.duration1=1680},
{.level0=1,.duration0=9010,.level1=0,.duration1=9350},
{.level0=1,.duration0=10660,.level1=0,.duration1=2700},
{.level0=1,.duration0=1280,.level1=0,.duration1=6990},
{.level0=1,.duration0=9030,.level1=0,.duration1=5360},
{.level0=1,.duration0=2610,.level1=0,.duration1=1370},
{.level0=1,.duration0=11940,.level1=0,.duration1=2710},
{.level0=1,.duration0=15970,.level1=0,.duration1=4030},
{.level0=1,.duration0=1280,.level1=0,.duration1=1380},
{.level0=1,.duration0=1280,.level1=0,.duration1=1370},
{.level0=1,.duration0=10660,.level1=0,.duration1=2700},
{.level0=1,.duration0=1280,.level1=0,.duration1=5360},
{.level0=1,.duration0=10650,.level1=0,.duration1=10990},
{.level0=1,.duration0=9030,.level1=0,.duration1=10980},
{.level0=1,.duration0=9020,.level1=0,.duration1=4030},
{.level0=1,.duration0=1280,.level1=0,.duration1=4030},
{.level0=1,.duration0=10660,.level1=0,.duration1=10980},
{.level0=1,.duration0=32767,.level1=1,.duration1=32767},
{.level0=1,.duration0=32767,.level1=1,.duration1=10739},
{.level0=0,.duration0=10980,.level1=1,.duration1=9020},
{.level0=0,.duration0=10980,.level1=1,.duration1=10350},
{.level0=0,.duration0=2710,.level1=1,.duration1=1280},
{.level0=0,.duration0=5660,.level1=1,.duration1=10350},
{.level0=0,.duration0=2700,.level1=1,.duration1=1280},
{.level0=0,.duration0=2710,.level1=1,.duration1=1280},
{.level0=0,.duration0=1680,.level1=1,.duration1=9030},
{.level0=0,.duration0=10980,.level1=1,.duration1=9020},
{.level0=0,.duration0=5370,.level1=1,.duration1=1280},
{.level0=0,.duration0=4330,.level1=1,.duration1=9030},
{.level0=0,.duration0=1370,.level1=1,.duration1=1280},
{.level0=0,.duration0=1380,.level1=1,.duration1=1280},
{.level0=0,.duration0=5670,.level1=1,.duration1=9020},
{.level0=0,.duration0=10980,.level1=1,.duration1=9020},
{.level0=0,.duration0=10980,.level1=1,.duration1=9030},
{.level0=0,.duration0=10980,.level1=1,.duration1=9020},
{.level0=0,.duration0=10980,.level1=1,.duration1=9020},
{.level0=0,.duration0=10980,.level1=1,.duration1=32767},
{.level0=1,.duration0=32767,.level1=1,.duration1=24836},
{.level0=0,.duration0=4030,.level1=1,.duration1=2610},
{.level0=0,.duration0=1370,.level1=1,.duration1=610},
{.level0=1,.duration0=0,.level1=1,.duration1=0},
};

/* ═══════════════════════════════════════════════════════════════════════════ */

static rmt_channel_handle_t  s_clk_chan  = NULL;
static rmt_channel_handle_t  s_mosi_chan = NULL;
static rmt_encoder_handle_t  s_clk_copy_enc  = NULL;
static rmt_encoder_handle_t  s_mosi_copy_enc  = NULL;
static rmt_sync_manager_handle_t s_sync  = NULL;

static void spi_rmt_init(void)
{
    rmt_tx_channel_config_t tx_cfg = {
        .clk_src           = RMT_CLK_SRC_DEFAULT,
        .resolution_hz     = RMT_RESOLUTION_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 16,
        .flags.invert_out  = false,
        .flags.with_dma    = false,
    };

    tx_cfg.gpio_num = GPIO_CLK;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_clk_chan));
    tx_cfg.gpio_num = GPIO_MOSI;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &s_mosi_chan));

    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &s_clk_copy_enc));
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &s_mosi_copy_enc));

    ESP_ERROR_CHECK(rmt_enable(s_clk_chan));
    ESP_ERROR_CHECK(rmt_enable(s_mosi_chan));

    rmt_channel_handle_t channels[] = { s_clk_chan, s_mosi_chan };
    rmt_sync_manager_config_t sync_cfg = {
        .tx_channel_array = channels,
        .array_size       = sizeof(channels) / sizeof(channels[0]),
    };
    ESP_ERROR_CHECK(rmt_new_sync_manager(&sync_cfg, &s_sync));

}

static void spi_rmt_send_frame(void)
{
    ESP_ERROR_CHECK(rmt_sync_reset(s_sync));

    rmt_transmit_config_t tx_trans = {
    .loop_count = 0,
        .flags = {
                .eot_level = 1
        }
    };

    /* Queue CLK first, then MOSI.  Neither channel starts until both have been
     * queued — the sync manager holds them in reset and fires both on the same
     * RMT clock edge once the second rmt_transmit() call completes. */
    ESP_ERROR_CHECK(rmt_transmit(s_clk_chan,  s_clk_copy_enc,
                                 clk_symbols,  sizeof(clk_symbols),  &tx_trans));
    ESP_ERROR_CHECK(rmt_transmit(s_mosi_chan, s_mosi_copy_enc,
                                mosi_symbols, sizeof(mosi_symbols), &tx_trans));

    /* Wait for both to complete */
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_clk_chan,  portMAX_DELAY));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(s_mosi_chan, portMAX_DELAY));
}

void app_main(void)
{
    spi_rmt_init();
    ESP_LOGI(TAG, "SPI RMT replay started — period %d ms", FRAME_PERIOD_MS);

    int64_t next_us = esp_timer_get_time();

    while (1) {
        spi_rmt_send_frame();
        /* Maintain a fixed period regardless of frame duration */
        next_us += (int64_t)FRAME_PERIOD_MS * 1000;
        int64_t now = esp_timer_get_time();
        int64_t sleep_us = next_us - now;
        if (sleep_us > 1000) {
            vTaskDelay(pdMS_TO_TICKS(sleep_us / 1000));
        }
    }
}
