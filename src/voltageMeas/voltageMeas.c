#include "../types.h"
#include "../ads1115/ads1115.h"
#include "../i2c/I2CCommonFx.h"
#define P_GAIN (4.096f/32768) //0.00012500f

uint16_t i_N = 0;
float irms = 0;
float i_sumatoria = 0;
int8_t i_Tintegrt = 0;

//void voltageMeas(void)
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

    v = v - 1.5;//1.5 center, elimino el offset
    v = v * -1; //invierto la señal
    
    return v;
    
    // i = v*2;//100/50V
    // i_sumatoria += (i*i);
    // //
    // i_N++;

    // if (main_flag.f20ms )
    // {
    //     if (++i_Tintegrt >= 25)//25* 20ms = 500
    //     {
    //         irms = sqrt(i_sumatoria/i_N);

    //         //reset
    //         i_Tintegrt = 0x00;
    //         i_N = 0x00;
    //         i_sumatoria = 0;
    //     }
    // }
}
