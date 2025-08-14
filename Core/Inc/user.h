#ifndef __USER_H__
#define __USER_H__

#define VERSION_MAJOR 0
#define VERSION_MINOR 2

#define RDATA_SIZE 11
#define TDATA_SIZE 30

void idle_irq(void);
void loop_1s(void);
void loop_100ms(void);

#endif
