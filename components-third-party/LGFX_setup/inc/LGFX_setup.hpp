/*
 * Written by Tomas Hak in 2025
 *
 * This library is a wrapper setup for LovyanGFX library written by Limor Fried/Ladyada from Adafruit Industries.
 * See license file of the LovyanGFX library for further usage.
 *
 * I do not claim any rights for further usage of these additional changes written in this file and realted .cpp file to Limor Fried's library.
 */

#pragma once
#include <LovyanGFX.hpp>

typedef enum {
    SPI_MODE_0 = 0,
    SPI_MODE_1 = 1,
    SPI_MODE_2 = 2,
} spi_mode_t;

typedef enum {
    PANEL_ST7789,
    PANEL_ST7796,
} panel_type;

class LGFX : public lgfx::LGFX_Device {
public:
    void LGFX_config(gpio_num_t sclk, gpio_num_t mosi, gpio_num_t dc, gpio_num_t rst, panel_type panel_type);
    void LGFX_init(void);

private:
    panel_type panel_type_info;

    lgfx::Bus_SPI bus_spi;   // SPI bus object
    lgfx::Panel_ST7789 panel_ST7789; // ST7789 panel object
    lgfx::Panel_ST7796 panel_ST7796; // ST7796 panel object

    lgfx::v1::Bus_SPI::config_t spi_bus_cfg;
    lgfx::Panel_ST7789::config_t panel_cfg_ST7789; // ST7789 panel configuration
    lgfx::Panel_ST7796::config_t panel_cfg_ST7796; // ST7796 panel configuration
};