
extern "C"{
    #include "src/system.h"
    #include "src/i2c/I2C.h"
    #include "src/i2c/I2CCommonFx.h"
    #include "src/ads1115/ads1115.h"
    #include "src/voltageMeas/voltageMeas.h"
    #include "src/usart/usart.h"
    #include "src/pinGetLevel/pinGetLevel.h"
};
volatile struct _isr_flag
{
    unsigned f10ms :1;
    //unsigned adq:1;
    unsigned __a :7;
} isr_flag = { 0,0 };

struct _main_flag
{
    unsigned f10ms :1;
    //unsigned adq:1;
    unsigned __a:7;

}main_flag = { 0,0 };

volatile float metros;
volatile float kmeters = 0;


//+- Encoder
/*
500 pulsos / vuelta
1 vuelta = m
*/
//unidad de medicion = Metros
#define ENC_NUMPULSOS_VUELTA 500    //500 pulsos por vuelta, salida del encoder en una sola direccion
float ENC_1V_METROSLINEAL = 0.5;    //metros
float ENC_RESOL = ENC_1V_METROSLINEAL/ENC_NUMPULSOS_VUELTA;

#include "qdec.h"
#define ROTARY_PIN_A 2//D2
#define ROTARY_PIN_B 3//D3
::SimpleHacks::QDecoder qdec(ROTARY_PIN_A, ROTARY_PIN_B, true);
volatile int32_t rotaryCount = 0;
int lastLoopDisplayedRotaryCount = 0;
void IsrForQDEC(void);
//-+
void IsrForQDEC(void)
{
  using namespace ::SimpleHacks;
  QDECODER_EVENT event = qdec.update();

  if (event & QDECODER_EVENT_CW)
  {
    rotaryCount++;
  }
  else if (event & QDECODER_EVENT_CCW)
  {
    rotaryCount--;
  }
  //
//  metros = rotaryCount * ENC_RESOL;//
//  if ( (metros % kmeters) == 0) //Si es multiplo de kmeters...
//  {
//    isr_flag.adq = 1;
//  }
//
  return;
}

inline float enc_getMeters(void)
{
 return rotaryCount * ENC_RESOL;
}
inline void enc_resetCounter(void)
{
    rotaryCount = 0x00;
}

//PD4
// #define DDRxSW_ENC_RESET    (DDRG)
// #define PORTWxSW_ENC_RESET  (PORTG)
// #define PORTRxSW_ENC_RESET  (PING)
// #define PIN_SW_ENC_RESET    (5)

int16_t ADQ_KTIME=0;

void setup()
{
    //ADS1115 init
    I2C_unimaster_init(100E3);//100KHz
    uint8_t reg[2];

    //++--Write config
    reg[0] = (1<<OS_BIT) | (MUX_AIN0_GND<<MUX_BIT) | (PGA_4p096V<<PGA_BIT) | (CONTINUOUS_CONV<<MODE_BIT);//PGA 4.096V -> RESOL=125 μV
    reg[1] = (DR_860SPS<<DR_BIT);
    I2Ccfx_WriteArray(ADS115_ADR_GND, ADS1115_CONFIG_REG, &reg[0], 2);
    //default state of ConfigRegister = 0x8583 = 34179

    //Encoder init
    qdec.begin();
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), IsrForQDEC, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), IsrForQDEC, CHANGE);

    //USART por hardware
    //USART_Init ( MYUBRR );//@9600
    Serial.begin(9600);

    //D4 = PG5 SW reset encoder counter
    pinGetLevel_init(); //with Changed=flag activated at initialization
    //PinTo1(PORTWxSW_ENC_RESET, PIN_SW_ENC_RESET); //Enable pull-up
    //ConfigInputPin(DDRxSW_ENC_RESET, PIN_SW_ENC_RESET);

    //int
    //Config to 10ms
    TCNT0 = 0x00;
    TCCR0A = (1 << WGM01) | (1 << CS02) | (0 << CS01) | (1 << CS00); //CTC, PRES=1024
    OCR0A = CTC_SET_OCR_BYTIME(10e-3, 1024); //TMR8-BIT @16MHz @PRES=1024-> BYTIME maximum = 16ms
    //
    TIMSK1 |= (1 << OCIE0A);
    sei();
}

int8_t send(int32_t m, float v)
{
    //aun por definir el envio
    Serial.print("["); Serial.print(m); Serial.print(" m,");
    Serial.print(v);Serial.println(" v]");
    return 0;
}

void loop()
{
	//volatile
	float metros_diff = 0;
	//volatile
	float kmeter_new0 = 0;

    float  voltaje;
    int8_t SW=0;
    static int8_t c;

    //-------------------------
    if (isr_flag.f10ms)
    {
        isr_flag.f10ms = 0;
        main_flag.f10ms = 1;
    }
//    if (isr_flag.adq)
//    {
//        isr_flag.adq  = 0;
//        main_flag.adq = 1;
//    }

    //-------------------------
    //+-
    metros = enc_getMeters();
    metros_diff = metros - kmeter_new0;
    if (metros_diff >= kmeters)
    {
        kmeter_new0 = metros;
        //
        voltaje = voltageMeas();//en ese punto, no el promedio
		send(metros, voltaje);

    }//+-

    //----------------------
    if (main_flag.f10ms)
    {
        if (++c == 2)    //20ms
        {
            c = 0;

            pinGetLevel_job();
            if (pinGetLevel_hasChanged(PGLEVEL_LYOUT_SW_ENC_RESET))
            {
                SW = !pinGetLevel_level(PGLEVEL_LYOUT_SW_ENC_RESET);
                pinGetLevel_clearChange(PGLEVEL_LYOUT_SW_ENC_RESET);
            }
        }
    }

    //comunicacion USB
    if ((SW == 1) ) //|| (char == "s") // -> usar el pulsador mientras
    {
        enc_resetCounter();// resetear el contador de # de pulsos
        SW = 0x00;
    }
    //-+
    // if (char == "i")//set new adquisition time
    // {
    //     ADQ_KTIME =  / 20;//20ms... el intervalo que envie el host div.x 20ms
    // }
    main_flag.f10ms = 0;
}


ISR(TIMER0_COMPA_vect)
{
    isr_flag.f10ms = 1;
}
