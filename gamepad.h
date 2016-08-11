#ifndef _GAMEPAD_H_
#define _GAMEPAD_H_

#include <inttypes.h>
#include "defines.h"

#define GLUE(a,b) a##b
#define DDR(p) GLUE(DDR,p)
#define PORT(p) GLUE(PORT,p)
#define PIN(p) GLUE(PIN,p)

#define N64_PORT_PORT PORT(N64_PORT)
#define N64_PORT_DDR DDR(N64_PORT)
#define N64_PORT_PIN PIN(N64_PORT)

#define NES_PORT_PORT PORT(NES_PORT)
#define NES_PORT_DDR DDR(NES_PORT)
#define NES_PORT_PIN PIN(NES_PORT)

#define SNES_PORT_PORT PORT(NES_PORT)
#define SNES_PORT_DDR DDR(NES_PORT)
#define SNES_PORT_PIN PIN(NES_PORT)

#define SMD_SELECT_PORT_PORT PORT(SMD_SELECT_PORT)
#define SMD_SELECT_PORT_DDR DDR(SMD_SELECT_PORT)
#define SMD_DATA_PORT_PORT PORT(SMD_DATA_PORT)
#define SMD_DATA_PORT_DDR DDR(SMD_DATA_PORT)
#define SMD_DATA_PORT_PIN PIN(SMD_DATA_PORT)
#define SMD_DATA_PORT_PORT2 PORT(SMD_DATA_PORT2)
#define SMD_DATA_PORT_DDR2 DDR(SMD_DATA_PORT2)
#define SMD_DATA_PORT_PIN2 PIN(SMD_DATA_PORT2)

#define DUALSHOCK_PORT_PORT PORT(DUALSHOCK_PORT)
#define DUALSHOCK_PORT_DDR DDR(DUALSHOCK_PORT)
#define DUALSHOCK_PORT_PIN PIN(DUALSHOCK_PORT)

#define WAIT(t) {TCNT0=0; while(TCNT0 < (F_CPU / 1000000UL) * t);}

#define N64SEND(t) {N64_PORT_DDR |= (1<<N64_DATA_PIN); WAIT(t); N64_PORT_DDR &= ~(1<<N64_DATA_PIN);}
#define N64SEND_1 {N64SEND(1); WAIT(3);}
#define N64SEND_0 {N64SEND(3); WAIT(1);}
#define N64SEND_STOP {N64SEND(1); WAIT(2);}
#define N64SIGNAL (!((N64_PORT_PIN>>N64_DATA_PIN)&1))

void init_nes_gamepad(void);
uint32_t get_nes_gamepad(void);
uint32_t get_nes_gamepad_decoded(void);
void init_snes_gamepad(void);
uint16_t get_snes_gamepad(void);
void init_n64_gamepad(void);
int get_n64_gamepad(uint8_t* data);
void init_smd_gamepad(void);
uint32_t get_smd_gamepad(void);
uint32_t get_smd_gamepad_decoded(void);
void init_dualshock_gamepad(void);
int dualshock_command(uint8_t* command, uint8_t* data, int length);
int get_dualshock_gamepad(uint8_t* data, int size, uint8_t motor_small, uint8_t motor_large);

#endif
