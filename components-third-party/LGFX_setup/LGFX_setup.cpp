/*
 * Written by Tomas Hak in 2025
 *
 * This library is a wrapper setup for LovyanGFX library written by Limor Fried/Ladyada from Adafruit Industries.
 * See license file of the LovyanGFX library in this directory for further usage.
 *
 * I do not claim any rights for further usage of these additional changes written in this file and realted .hpp file to Limor Fried's library.
 */

#include <LGFX_setup.hpp>


void LGFX::LGFX_config(gpio_num_t sclk, gpio_num_t mosi, gpio_num_t dc, gpio_num_t rst, panel_type panel_type)
{
    // SPI bus configuration common for all panels
    spi_bus_cfg.spi_host	= VSPI_HOST;
    spi_bus_cfg.freq_write	= 40000000; // 40 MHz
    spi_bus_cfg.freq_read	= 16000000;  // 16 MHz
    spi_bus_cfg.pin_sclk	= sclk;
    spi_bus_cfg.pin_mosi	= mosi;
    spi_bus_cfg.pin_miso	= -1; // not used
    spi_bus_cfg.pin_dc		= dc;
    spi_bus_cfg.use_lock	= true;

    // store for later usage
    panel_type_info = panel_type;
    
    switch (panel_type) {
        case PANEL_ST7789:
            // SPI bus configuration specific to ST7789
            spi_bus_cfg.spi_mode	= SPI_MODE_2;

            // ST7789 panel configuration
            panel_cfg_ST7789 = panel_ST7789.config();
            panel_cfg_ST7789.pin_cs			    = -1;  // CS pin (not used)
            panel_cfg_ST7789.pin_rst			= rst;  // RST pin
            panel_cfg_ST7789.pin_busy			= -1;  // not used
            panel_cfg_ST7789.offset_x			= 0;
            panel_cfg_ST7789.offset_y			= 0;
            panel_cfg_ST7789.offset_rotation	= 0;
            panel_cfg_ST7789.readable			= false;
            panel_cfg_ST7789.invert			    = true;
            break;

        case PANEL_ST7796:
            // SPI bus configuration specific to ST7796
            spi_bus_cfg.spi_mode	= SPI_MODE_0;

            // ST7796 panel configuration
            panel_cfg_ST7796 = panel_ST7796.config();
            panel_cfg_ST7796.pin_cs			    = -1;  // CS pin (not used)
            panel_cfg_ST7796.pin_rst			= rst;  // RST pin
            panel_cfg_ST7796.pin_busy			= -1;  // not used
            panel_cfg_ST7796.offset_x			= 0;
            panel_cfg_ST7796.offset_y			= 0;
            panel_cfg_ST7796.offset_rotation	= 0;
            panel_cfg_ST7796.readable			= false;
            panel_cfg_ST7796.invert			    = true;
            break;

        default:
            break;
    }

    
}

void LGFX::LGFX_init(void)
{
    switch (panel_type_info) {
        case PANEL_ST7789:
            // SPI bus configuration
            bus_spi.config(spi_bus_cfg); // apply config to SPI bus
            panel_ST7789.setBus(&bus_spi);  // attach SPI bus to panel

            // ST7789 panel configuration
            panel_ST7789.config(panel_cfg_ST7789); // apply panel configuration
            setPanel(&panel_ST7789);        // attach panel to LGFX_Device
            break;
        
        case PANEL_ST7796:
            // SPI bus configuration
            bus_spi.config(spi_bus_cfg); // apply config to SPI bus
            panel_ST7796.setBus(&bus_spi);  // attach SPI bus to panel

            // ST7796 panel configuration
            panel_ST7796.config(panel_cfg_ST7796); // apply panel configuration
            setPanel(&panel_ST7796);        // attach panel to LGFX_Device
            break;

        default:
            break;
    }
}