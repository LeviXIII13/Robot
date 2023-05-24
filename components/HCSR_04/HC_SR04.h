#ifndef HC_SR04_H
#define HC_SR04_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/gpio.h"

#define HCSR04_FORWARD_TRIG GPIO_NUM_40
#define HCSR04_FORWARD_TRIG_SEL (1ULL<<GPIO_NUM_40)
#define HCSR04_FORWARD_ECHO GPIO_NUM_39

#define HCSR04_LEFT_TRIG GPIO_NUM_47
#define HCSR04_LEFT_TRIG_SEL (1ULL<<GPIO_NUM_47)
#define HCSR04_LEFT_ECHO GPIO_NUM_48

#define HCSR04_RIGHT_TRIG GPIO_NUM_41
#define HCSR04_RIGHT_TRIG_SEL (1ULL<<GPIO_NUM_41)
#define HCSR04_RIGHT_ECHO GPIO_NUM_42






void Generate_Trig(void *arg);
void HCSR04_Init(void);
void Fowrward_Distance_Get(void *param);
void Left_Distance_Get(void *param);
void Right_Distance_Get(void *param);





#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* HC_SR04_H */