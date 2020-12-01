#ifndef _GAMEPAD_H_
#define _GAMEPAD_H_

#define set_bit(reg,value) reg |= (1 << (value))

uint32_t get_nes_gamepad_decoded(void);
uint32_t get_smd_gamepad_decoded(void);

#endif
