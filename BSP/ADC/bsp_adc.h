

#ifndef BSP_ADC_H_
#define BSP_ADC_H_

#include "F28x_Project.h"

void bsp_adc_init(void);
void bsp_adc_start(void);



typedef struct {
    float alpha;/* 滤波系数 */
    float f;/* 截止频率 */
    float t;/* 时间常数 */
}FILTE;

#endif /* BSP_ADC_H_ */

