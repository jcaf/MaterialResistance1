#include "../types.h"
#include "../ads1115/ads1115.h"
#include "../i2c/I2CCommonFx.h"
#include "voltageMeas.h"
//#define P_GAIN (4.096f/32768) //0.00012500f


float voltageMeas(void)
{
    float v;
    int16_t ib16;
    uint8_t reg[2];
    //
    I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
    ib16 = (reg[0]<<8) + reg[1];
    //
    v = ib16 * P_GAIN;//aqui v ya es voltaje

    //v = v - 1.5;//1.5 center, elimino el offset
    v = v - 1.498;//1.5 center, elimino el offset
    v = v * -1; //invierto la señal
    
    return v;
}
/*
float voltageMeas(void)
{
    float v;
    //float i;
    int16_t ib16;
    uint8_t reg[2];
    //
    I2Ccfx_ReadRegistersAtAddress(ADS115_ADR_GND, ADS1115_CONVRS_REG, &reg[0], 2);
    ib16 = (reg[0]<<8) + reg[1];
    //
    v = ib16 * P_GAIN;//aqui v ya es voltaje

    //v = v - 1.5;//1.5 center, elimino el offset
    v = v - 1.498;//1.5 center, elimino el offset
    v = v * -1; //invierto la señal
    
    return v;
}
*/
