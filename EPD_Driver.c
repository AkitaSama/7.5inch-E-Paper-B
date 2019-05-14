#include "EPD_Driver.h"

#define EDP_SPI_INSTANCE  0
static const nrf_drv_spi_t epd_spi = NRF_DRV_SPI_INSTANCE(EDP_SPI_INSTANCE);
static volatile bool epd_spi_xfer_done;

static uint8_t epd_spi_tx_buf[256];
static uint8_t epd_spi_rx_buf[256];

uint8_t BlackImage[IMAGE_SIZE];
uint8_t RedImage  [IMAGE_SIZE]; 

void NRF_GPIO_Init(void)
{
    nrf_gpio_cfg_input(EPD_BUSY_PIN, NRF_GPIO_PIN_NOPULL);
    nrf_gpio_cfg_output(EPD_DC_PIN);
    nrf_gpio_cfg_output(EPD_RST_PIN);
    nrf_gpio_pin_clear(EPD_DC_PIN);
    nrf_gpio_pin_clear(EPD_RST_PIN);
}

void NRF_GPIO_unInit(void)
{
    nrf_gpio_cfg_default(EPD_BUSY_PIN);
    nrf_gpio_cfg_default(EPD_DC_PIN);
    nrf_gpio_cfg_default(EPD_RST_PIN);
}

void epd_spi_event_handler(nrf_drv_spi_evt_t const * p_event, void * p_context)
{
    epd_spi_xfer_done = true;
}

void NRF_SPI_Init(void)
{
    nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
    spi_config.ss_pin   = EPD_CS_PIN;
    spi_config.mosi_pin = EPD_SDI_PIN;
    spi_config.sck_pin  = EPD_SCK_PIN;
    nrf_drv_spi_init(&epd_spi, &spi_config, epd_spi_event_handler, NULL);
}

void NRF_SPI_unInit(void)
{
    nrf_drv_spi_uninit(&epd_spi);
}

static void EPD_SpiTransfer(unsigned char data)
{
    epd_spi_tx_buf[0] = data;
    epd_spi_xfer_done = false;
    nrf_drv_spi_transfer(&epd_spi, epd_spi_tx_buf, 1, epd_spi_rx_buf, 0);
    while(!epd_spi_xfer_done);
}

static void EPD_Reset(void)
{
    nrf_gpio_pin_write(EPD_RST_PIN, 1);
    nrf_delay_ms(200);
    nrf_gpio_pin_write(EPD_RST_PIN, 0);
    nrf_delay_ms(200);
    nrf_gpio_pin_write(EPD_RST_PIN, 1);
    nrf_delay_ms(200);
}

static void EPD_SendCommand(unsigned char command) 
{
    nrf_gpio_pin_write(EPD_DC_PIN, LOW);
    EPD_SpiTransfer(command);
}

static void EPD_SendData(unsigned char data) 
{
    nrf_gpio_pin_write(EPD_DC_PIN, HIGH);
    EPD_SpiTransfer(data);
}

static void EPD_WaitUntilIdle(void)
{
    uint8_t busy;
    do {
        EPD_SendCommand(0x71);
        busy = nrf_gpio_pin_read(EPD_BUSY_PIN);
        busy =!(busy & 0x01);
    } while(busy);
}

static void EPD_TurnOnDisplay(void)
{
    EPD_SendCommand(POWER_ON);
    EPD_WaitUntilIdle();
    EPD_SendCommand(DISPLAY_REFRESH);
    nrf_delay_ms(100);
    EPD_WaitUntilIdle();
}

uint8_t EPD_Init(void) 
{
    EPD_Reset();
	
    EPD_SendCommand(POWER_SETTING);
    EPD_SendData(0x37);
    EPD_SendData(0x00);

    EPD_SendCommand(PANEL_SETTING);
    EPD_SendData(0xCF);
    EPD_SendData(0x08);

    EPD_SendCommand(PLL_CONTROL);
    EPD_SendData(0x3A);

    EPD_SendCommand(VCM_DC_SETTING);
    EPD_SendData(0x28);

    EPD_SendCommand(BOOSTER_SOFT_START);
    EPD_SendData (0xc7);
    EPD_SendData (0xcc);
    EPD_SendData (0x15);

    EPD_SendCommand(VCOM_AND_DATA_INTERVAL_SETTING);
    EPD_SendData(0x77);

    EPD_SendCommand(TCON_SETTING);
    EPD_SendData(0x22);

    EPD_SendCommand(SPI_FLASH_CONTROL);
    EPD_SendData(0x00);

    EPD_SendCommand(TCON_RESOLUTION);
    EPD_SendData(EPD_WIDTH >> 8);
    EPD_SendData(EPD_WIDTH & 0xff);
    EPD_SendData(EPD_HEIGHT >> 8);
    EPD_SendData(EPD_HEIGHT & 0xff);

    EPD_SendCommand(0xe5);
    EPD_SendData(0x03);

    return 0;
}

void EPD_Clear(void)
{
    uint16_t Width, Height;
    Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8): (EPD_WIDTH / 8 + 1);
    Height = EPD_HEIGHT;

    EPD_SendCommand(DATA_START_TRANSMISSION_1);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            for(uint8_t k = 0; k < 4; k++) {
                EPD_SendData(0x33);
            }
        }
    }
    EPD_TurnOnDisplay();
}

void EPD_Display(void)
{
    uint8_t Data_Black, Data_Red, Data;
    uint32_t i, j, Width, Height;
    Width = (EPD_WIDTH % 8 == 0)? (EPD_WIDTH / 8 ): (EPD_WIDTH / 8 + 1);
    Height = EPD_HEIGHT;

    EPD_SendCommand(DATA_START_TRANSMISSION_1);
    for (j = 0; j < Height; j++) {
        for (i = 0; i < Width; i++) {
            Data_Black = BlackImage[i + j * Width];
            Data_Red = RedImage[i + j * Width];
            for(uint8_t k = 0; k < 8; k++) {
                if ((Data_Red & 0x80) == 0x00) {
                    Data = 0x04;
                } else if ((Data_Black & 0x80) == 0x00) {
                    Data = 0x00;
                } else {
                    Data = 0x03;
                }
                Data = (Data << 4) & 0xFF;
                Data_Black = (Data_Black << 1) & 0xFF;
                Data_Red = (Data_Red << 1) & 0xFF;
                k += 1;

                if((Data_Red & 0x80) == 0x00) {
                    Data |= 0x04;
                } else if ((Data_Black & 0x80) == 0x00) {
                    Data |= 0x00;
                } else {
                    Data |= 0x03;
                }
                Data_Black = (Data_Black << 1) & 0xFF;
                Data_Red = (Data_Red << 1) & 0xFF;
                EPD_SendData(Data);
            }
        }
    }

    EPD_TurnOnDisplay();
}

void EPD_Sleep(void)
{
    EPD_SendCommand(POWER_OFF);
    EPD_SendCommand(DEEP_SLEEP);
    EPD_SendData(0XA5);
}
