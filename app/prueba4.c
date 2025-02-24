#include <stdbool.h>

#include "spi.h"
#include "leds.h"
#include "sx126x.h"
#include "sx126x_hal_context.h"
#include "sx126x_hal.h"

#define SX1262_SPI_HANDLE      &hspi1

#define SX1262_NSS_GPIO_PORT   GPIOD
#define SX1262_NSS_GPIO_PIN    GPIO_PIN_0   

#define SX1262_RESET_GPIO_PORT GPIOB
#define SX1262_RESET_GPIO_PIN  GPIO_PIN_0

#define SX1262_IRQ_GPIO_PORT   GPIOB
#define SX1262_IRQ_GPIO_PIN    GPIO_PIN_1

#define SX1262_BUSY_GPIO_PORT  GPIOC
#define SX1262_BUSY_GPIO_PIN   GPIO_PIN_0

sx126x_hal_context_t sx1262_context =
{
    .spi = SX1262_SPI_HANDLE,
    .nss =
    {
        .port = SX1262_NSS_GPIO_PORT,
        .pin = SX1262_NSS_GPIO_PIN,
    },
    .reset =
    {
        .port = SX1262_RESET_GPIO_PORT,
        .pin = SX1262_RESET_GPIO_PIN,
    },
    .irq =
    {
        .port = SX1262_IRQ_GPIO_PORT,
        .pin = SX1262_IRQ_GPIO_PIN,
    },
    .busy =
    {
        .port = SX1262_BUSY_GPIO_PORT,
        .pin = SX1262_BUSY_GPIO_PIN,
    },
};

extern void SystemClock_Config(void);
static bool status_check(void);
static bool buffer_sanity_check(void);
inline static uint8_t compute_lora_ldro(const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();

    const sx126x_mod_params_lora_t sx1262_lora_mod_params =
    {
        .sf = SX126X_LORA_SF7,
        .bw = SX126X_LORA_BW_125,
        .cr = SX126X_LORA_CR_4_5,
        .ldro = compute_lora_ldro(SX126X_LORA_SF7, SX126X_LORA_BW_125),
    };

    const sx126x_pkt_params_lora_t sx1262_pkt_params_lora =
    {
        .preamble_len_in_symb = 8,
        .header_type = SX126X_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes     = 7,
        .crc_is_on            = false,
        .invert_iq_is_on      = false,
    };

    const sx126x_pa_cfg_params_t pa_config = // The parameters depend on the chip being used
    {
        .pa_duty_cycle = 0x03,
        .hp_max = 0x02,
        .device_sel = 0x00,
        .pa_lut = 0x01,
    };

    /*************   sx1262_hw_init  **********************/ // Initialize the system configuration of the transceiver
    sx126x_hal_reset(&sx1262_context);
    sx126x_init_retention_list(&sx1262_context);
    sx126x_status_t set_reg_status = sx126x_set_reg_mode(&sx1262_context, SX126X_REG_MODE_DCDC); // TODO: No entiendo
    sx126x_status_t set_dio2_status = sx126x_set_dio2_as_rf_sw_ctrl(&sx1262_context, true);
    sx126x_status_t set_dio3_status = sx126x_set_dio3_as_tcxo_ctrl(&sx1262_context, SX126X_TCXO_CTRL_3_3V, 300);
    sx126x_status_t set_cal_status = sx126x_cal(&sx1262_context, SX126X_CAL_ALL);

    /************* sx1262_radio_init **********************/ // Initialize the radio configuration of the transceiver
    sx126x_status_t set_stdby_status = sx126x_set_standby(&sx1262_context, SX126X_STANDBY_CFG_XOSC); // XOSC
    sx126x_status_t set_pkt_status = sx126x_set_pkt_type(&sx1262_context, SX126X_PKT_TYPE_LORA);
    sx126x_status_t set_rf_status = sx126x_set_rf_freq(&sx1262_context, 868000000U);

    sx126x_status_t set_pa_status = sx126x_set_pa_cfg(&sx1262_context, &pa_config);
    sx126x_status_t set_tx_params_status = sx126x_set_tx_params(&sx1262_context, 14, SX126X_RAMP_40_US); // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )

    sx126x_status_t set_fallback_status = sx126x_set_rx_tx_fallback_mode(&sx1262_context, SX126X_FALLBACK_STDBY_XOSC);
    sx126x_status_t set_cfg_rx_status = sx126x_cfg_rx_boosted(&sx1262_context, false);

    sx126x_status_t set_lora_mod_status = sx126x_set_lora_mod_params(&sx1262_context, &sx1262_lora_mod_params);
    sx126x_status_t set_lora_pkt_status = sx126x_set_lora_pkt_params(&sx1262_context, &sx1262_pkt_params_lora);
    sx126x_status_t set_lora_sync_status = sx126x_set_lora_sync_word(&sx1262_context, 0x12);

    sx126x_status_t set_dio_irq_status = sx126x_set_dio_irq_params(
        &sx1262_context, SX126X_IRQ_ALL,
        SX126X_IRQ_RX_DONE | SX126X_IRQ_TIMEOUT,
        SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    sx126x_status_t set_clear_irq_status = sx126x_clear_irq_status(&sx1262_context, SX126X_IRQ_ALL);

    sx126x_status_t set_rx_status = sx126x_set_rx(&sx1262_context, SX126X_RX_SINGLE_MODE);

    if (set_rx_status != SX126X_STATUS_OK)
    {
        while(1)
        {
            HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
            HAL_Delay(200);
        }
    }

    while (1)
    {
        HAL_Delay(1000);

        status_check();

        if (buffer_sanity_check())
        {
            HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_SET);
        }
        else
        {
            HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        }

        HAL_Delay(1000);

        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN | RED_LED_PIN | ORANGE_LED_PIN | BLUE_LED_PIN, GPIO_PIN_RESET);
    }
}

static bool status_check(void)
{
    sx126x_chip_status_t radio_status;
    if (sx126x_get_status(&sx1262_context, &radio_status) != SX126X_STATUS_OK)
    {
        return false;
    }
    uint8_t chip_status = radio_status.chip_mode;

    switch (chip_status)
    {
    case SX126X_CHIP_MODE_STBY_RC:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_STBY_XOSC:
        HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_FS:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_RX:
        HAL_GPIO_WritePin(GREEN_LED_PORT, GREEN_LED_PIN, GPIO_PIN_SET);
        break;

    case SX126X_CHIP_MODE_TX:
        HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);
        break;
        
    default:
        return false;
    }

    return true;
}

static bool buffer_sanity_check(void)
{
    uint8_t testPattern[4] = {0x55, 0xAA, 0x33, 0xCC};
    uint8_t readBuffer[4] = {0};
    uint8_t offset = 0;

    if (sx126x_write_buffer(&sx1262_context, offset, testPattern, 4) != SX126X_STATUS_OK)
    {
        return false;
    }

    if (sx126x_read_buffer(&sx1262_context, offset, readBuffer, 4) != SX126X_STATUS_OK)
    {
        return false;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        if (testPattern[i] != readBuffer[i])
        {
            HAL_GPIO_WritePin(ORANGE_LED_PORT, ORANGE_LED_PIN, GPIO_PIN_SET);
            return false;
        }
    }

    return true;
}

inline static uint8_t compute_lora_ldro(const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw)
{
    switch(bw)
    {
    case SX126X_LORA_BW_500:
        return 0;

    case SX126X_LORA_BW_250:
        if (sf == SX126X_LORA_SF12)
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_125:
        if ((sf == SX126X_LORA_SF12) || (sf == SX126X_LORA_SF11))
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_062:
        if ((sf == SX126X_LORA_SF12) || (sf == SX126X_LORA_SF11) || (sf == SX126X_LORA_SF10))
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_041:
        if ((sf == SX126X_LORA_SF12) || (sf == SX126X_LORA_SF11) || (sf == SX126X_LORA_SF10) ||
            (sf == SX126X_LORA_SF9))
        {
            return 1;
        }
        else
        {
            return 0;
        }

    case SX126X_LORA_BW_031:
    case SX126X_LORA_BW_020:
    case SX126X_LORA_BW_015:
    case SX126X_LORA_BW_010:
    case SX126X_LORA_BW_007:
        return 1;

    default:
        return 0;
    }
}