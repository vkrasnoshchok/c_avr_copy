#ifndef CONFIG_H
#define CONFIG_H

/* Timer config */
#define TIMER_NUMBER 2
#define USE_LOCALTIME 1

/* e-paper */
#define EPAPER_RST_PORT PORTC
#define EPAPER_RST_DDR DDRC
#define EPAPER_RST_PIN PC2
#define EPAPER_DC_PORT PORTC
#define EPAPER_DC_DDR DDRC
#define EPAPER_DC_PIN PC1
#define EPAPER_CS_PORT PORTC
#define EPAPER_CS_DDR DDRC
#define EPAPER_CS_PIN PC0
#define EPAPER_BUSY_PORT PORTC
#define EPAPER_BUSY_DDR DDRC
#define EPAPER_BUSY_PIN PC3
#define EPAPER_BUSY_PORT_IN PINC

/* serial config */
#define SERIAL_COMMAND_COUNT 1

/* 1wire config */
#define ONEWIRE_PORT PORTB
#define ONEWIRE_DDR DDRB
#define ONEWIRE_PORT_IN PINB
#define ONEWIRE_PIN PB0

#endif  /* CONFIG_H */
