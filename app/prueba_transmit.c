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

#define PAYLOAD_LENGTH 7

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

const uint8_t test_msg[PAYLOAD_LENGTH] = "TEST\r\n";
volatile bool sx1262_irq_fired = false; // ¿volatile needed?

extern void SystemClock_Config(void);
static bool status_check(void);
inline static uint8_t compute_lora_ldro(const sx126x_lora_sf_t sf, const sx126x_lora_bw_t bw);
static void sx1262_system_init(void);
static void sx1262_radio_init(void);
static void sx1262_irq_process(void);
static void on_tx_done(void);
static void on_tx_timeout(void);

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_SPI1_Init();
    sx1262_system_init();
    sx1262_radio_init();

    sx126x_set_dio_irq_params(
        &sx1262_context, SX126X_IRQ_ALL,
        SX126X_IRQ_TX_DONE | SX126X_IRQ_TIMEOUT,
        SX126X_IRQ_NONE, SX126X_IRQ_NONE);
    sx126x_clear_irq_status(&sx1262_context, SX126X_IRQ_ALL);

    uint8_t tx_buffer[PAYLOAD_LENGTH];

    memcpy(tx_buffer, test_msg, PAYLOAD_LENGTH);
    
    sx126x_write_buffer(&sx1262_context, 0, tx_buffer, PAYLOAD_LENGTH);

    sx126x_status_t set_tx_status = sx126x_set_tx(&sx1262_context, 1000);

    if (set_tx_status != SX126X_STATUS_OK)
    {
        while(1)
        {
            HAL_GPIO_TogglePin(RED_LED_PORT, RED_LED_PIN);
            HAL_Delay(300);
        }
    }

    while (1)
    {
        sx1262_irq_process();
        HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_RESET);
        HAL_Delay(1000);
    }
}

static void on_tx_done(void)
{
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_SET);

    HAL_Delay(1000);

    sx126x_set_tx(&sx1262_context, 1000);
}

static void on_tx_timeout(void)
{
    HAL_GPIO_WritePin(BLUE_LED_PORT, BLUE_LED_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(RED_LED_PORT, RED_LED_PIN, GPIO_PIN_SET);

    HAL_Delay(1000);

    sx126x_set_tx(&sx1262_context, 1000);
}

static void sx1262_irq_process(void)
{
    if (sx1262_irq_fired == true)
    {
        sx1262_irq_fired = false;

        sx126x_irq_mask_t irq_reg;
        sx126x_get_and_clear_irq_status(&sx1262_context, &irq_reg);

        if ((irq_reg & SX126X_IRQ_TX_DONE))
        {
            on_tx_done();
        }

        if ((irq_reg & SX126X_IRQ_TIMEOUT))
        {
            on_tx_timeout();
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == SX1262_IRQ_GPIO_PIN)
  {
    sx1262_irq_fired = true;
  }
}

// Initialize the system configuration of the transceiver
static void sx1262_system_init(void)
{
    sx126x_hal_reset(&sx1262_context);
    sx126x_init_retention_list(&sx1262_context);
    sx126x_set_reg_mode(&sx1262_context, SX126X_REG_MODE_DCDC);
    sx126x_set_dio2_as_rf_sw_ctrl(&sx1262_context, true);
    sx126x_set_dio3_as_tcxo_ctrl(&sx1262_context, SX126X_TCXO_CTRL_3_3V, 300);
    sx126x_cal(&sx1262_context, SX126X_CAL_ALL);
}

// Initialize the radio configuration of the transceiver
static void sx1262_radio_init(void)
{
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
        .pld_len_in_bytes = 7,
        .crc_is_on = false,
        .invert_iq_is_on = false,
    };

    const sx126x_pa_cfg_params_t pa_config = // The parameters depend on the chip being used
    {
        .pa_duty_cycle = 0x03,
        .hp_max = 0x02,
        .device_sel = 0x00,
        .pa_lut = 0x01,
    };

    sx126x_set_standby(&sx1262_context, SX126X_STANDBY_CFG_XOSC);
    sx126x_set_pkt_type(&sx1262_context, SX126X_PKT_TYPE_LORA);
    sx126x_set_rf_freq(&sx1262_context, 868000000U);

    sx126x_set_pa_cfg(&sx1262_context, &pa_config);
    sx126x_set_tx_params(&sx1262_context, 14, SX126X_RAMP_40_US); // range [-17, +22] for sub-G, range [-18, 13] for 2.4G ( HF_PA )

    sx126x_set_rx_tx_fallback_mode(&sx1262_context, SX126X_FALLBACK_STDBY_XOSC);
    sx126x_cfg_rx_boosted(&sx1262_context, false);

    sx126x_set_lora_mod_params(&sx1262_context, &sx1262_lora_mod_params);
    sx126x_set_lora_pkt_params(&sx1262_context, &sx1262_pkt_params_lora);
    sx126x_set_lora_sync_word(&sx1262_context, 0x12);
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