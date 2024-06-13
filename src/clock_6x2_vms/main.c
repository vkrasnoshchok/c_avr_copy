#include "display.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "onewire.h"
#include "ds18b20.h"
#include "bmp180.h"
#include "UART.h"
#include "SPI.h"
#include "TWI.h"
#include "timer.h"
#include <stdint.h>
#include "neo7m.h"
#include "max7219.h"

#define DISPLAY_ENABLED

#define MOTION_SENSOR_ENABLED
#define MOTION_TIMER_ID 1
#define MOTION_TIMER_PERIOD 10 // 1 second
#define MOTION_PIN (PIND & (1 << PD6))
#define IDLE_COUNTER_INIT_VALUE 45

#define TEMPERATURE_TIMER_ID 2
#define GPS_TIMEOUT_TIMER_ID 3

int16_t temperature;

static uint8_t idle_counter=0;

void convert_temperature(void);

void print_temperature(void)
{
    if (temperature == DS18B20_INVALID_TEMPERATURE)
    {
        clear_ext_temperature();
    }
    else
    {
        if (temperature & 0x0800)
        {
            temperature |= 0xF000;
        }
        print_ext_temperature(temperature);
    }
    timer_register(TEMPERATURE_TIMER_ID, 200, convert_temperature);
    // TODO: workaround to print big temp on start; need to debug(?)
    max7219_update();
}

void temperature_cb(uint16_t data)
{
    temperature = data;
    timer_register(TEMPERATURE_TIMER_ID, 10, print_temperature);
}

void read_temperature(void)
{
    ds18b20_read_temperature(temperature_cb);
    timer_register(TEMPERATURE_TIMER_ID, 200, convert_temperature);
}

void convert_temperature(void)
{
    ds18b20_convert();
    timer_register(TEMPERATURE_TIMER_ID, 10, read_temperature);
}

static void gps_timeout(void)
{
    timer_stop(GPS_TIMEOUT_TIMER_ID);
    display_activate();
}

static void _time_cb(void)
{
    time_update_handler();
    timer_register(GPS_TIMEOUT_TIMER_ID, 200, gps_timeout);
}


SIGNAL(PCINT2_vect)
{
    // motion interrupt
}

inline static void motion_handler(void)
{
    if (idle_counter == 0 && MOTION_PIN)
    {
        // if (ADC < ILLUMINANCE_ON_VALUE)
        // {
        //     LED_CONTROL_PORT |= 1 << LED_CONTROL_PIN;
        // }
        print_ext_temperature(temperature);
        display_activate();
        idle_counter = IDLE_COUNTER_INIT_VALUE;
    }
}

void motion_timer(void)
{
    // uart_send_hex(idle_counter);
    if (MOTION_PIN)
    {
        // if (ADC < ILLUMINANCE_ON_VALUE)
        // {
        //     LED_CONTROL_PORT |= 1 << LED_CONTROL_PIN;
        // }
        // else if (ADC > ILLUMINANCE_OFF_VALUE)
        // {
        //     LED_CONTROL_PORT &= ~(1 << LED_CONTROL_PIN);
        // }
        if (idle_counter != 0xFF && idle_counter != 0)
        {
            idle_counter = IDLE_COUNTER_INIT_VALUE;
        }
    }
    else if (idle_counter)
    {
        idle_counter--;
        if (idle_counter == 0)
        {
            // LED_CONTROL_PORT &= ~(1 << LED_CONTROL_PIN);
            display_deactivate();
        }
    }
}

void setup(void)
{
    twi_init();
    spi_master_init();
    timer_init();
    onewire_init();
    asm("sei");
    neo7m_init(_time_cb);
    bmp180_init(print_int_temperature, print_pressure);
    //_delay_ms(5);
#ifdef DISPLAY_ENABLED
    display_init();
#endif

    asm("sei");

#ifdef MOTION_SENSOR_ENABLED
    PCMSK2 |= 1 << PCINT22;
    PCICR |= 1 << PCIE2;  // Pin Change Interrupt Enable 2
    timer_register(MOTION_TIMER_ID, MOTION_TIMER_PERIOD, motion_timer);
#else
    display_activate();
#endif

}

int main(void)
{
    setup();
    uart_send_byte('S');
    convert_temperature();
    while(1)
    {
#ifdef MOTION_SENSOR_ENABLED
        motion_handler();
#endif
#ifdef DISPLAY_ENABLED
        max7219_handler();
#endif
        neo7m_handler();
        bmp180_handler();
        timer_handler();
    }
}
