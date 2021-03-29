/* Medicion de resistencia de material
 * Eclipse IDE Eclipse IDE for C/C++ Developers (includes Incubating components) Version: 2020-12 (4.18.0)
 * Arduino Plugins
 *
 */

extern "C"{
    #include "src/system.h"
    #include "src/i2c/I2C.h"
    #include "src/i2c/I2CCommonFx.h"
    #include "src/ads1115/ads1115.h"
    #include "src/voltageMeas/voltageMeas.h"
    //#include "src/usart/usart.h"
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


//+- Encoder
uint16_t ENCODER_PPR = 500;    			//500 Pulses Per Revolution
float ENCODER_1REV_INMETERS = 0.5f;    	//1revol = X meters
volatile float ADQ_KMETERS = 0.15f;		//Adquirir cada "x metros"

float ENC_RESOL = (float)ENCODER_1REV_INMETERS/ENCODER_PPR;

#include "qdec.h"
#define ROTARY_PIN_A 2//D2
#define ROTARY_PIN_B 3//D3
void IsrForQDEC(void);
::SimpleHacks::QDecoder qdec(ROTARY_PIN_A, ROTARY_PIN_B, true);
//
typedef int64_t ROTARYCOUNT_T;//para rotaryCount y quien va a tomar su ultimo valor para comparar
//
volatile ROTARYCOUNT_T rotaryCount = 0;
ROTARYCOUNT_T rotaryCount_last = 0;	//toma el valor de rotaryCount para ser el nuevo punto de referencia de inicio

//Las sgtes. variables no necesitan ser de 64bits
int32_t numPulsesIn_ADQ_KMETERS = (ADQ_KMETERS * ENCODER_PPR) / ENCODER_1REV_INMETERS;//truncar
int32_t numPulses_diff = 0;

void IsrForQDEC(void)
{
	using namespace ::SimpleHacks;
	QDECODER_EVENT event = qdec.update();
	if (event & QDECODER_EVENT_CW)
		rotaryCount++;
	else if (event & QDECODER_EVENT_CCW)
		rotaryCount--;
}

//inline float enc_getMeters(void)
//{
// return rotaryCount * ENC_RESOL;
//}
inline void enc_resetCounter(void)
{
    rotaryCount = 0x00;
    rotaryCount_last = 0x00;
}

//PD4
// #define DDRxSW_ENC_RESET    (DDRG)
// #define PORTWxSW_ENC_RESET  (PORTG)
// #define PORTRxSW_ENC_RESET  (PING)
// #define PIN_SW_ENC_RESET    (5)

int16_t ADQ_KTIME=0;


//Current measurement
#define TRAMA_START '@'
#define TRAMA_END 	'\n'
//#define TRAMA_PAYLOAD_SIZEMAX    sizeof(float)//determinado por el compilador

float meters = 0.0f;
float volts = 0.0f;
float current = 0.0f;
//
void setup()
{

    I2C_unimaster_init(100E3);//100KHz

    //ADS1115 init
    uint8_t reg[2];
    reg[0] = (1<<OS_BIT) | (MUX_AIN0_GND<<MUX_BIT) | (PGA_4p096V<<PGA_BIT) | (CONTINUOUS_CONV<<MODE_BIT);//PGA 4.096V -> RESOL=125 μV
    reg[1] = (DR_8SPS<<DR_BIT);//menos ruido
    I2Ccfx_WriteArray(ADS115_ADR_GND, ADS1115_CONFIG_REG, &reg[0], 2);

    //Encoder init
    qdec.begin();
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_A), IsrForQDEC, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_PIN_B), IsrForQDEC, CHANGE);
    
    Serial.begin(230400);	//USART0 x PC
    //Serial1.begin(9600);	//USART1 x ATmega328P x current measurement 9600 ok
    Serial1.begin(38400);	//USART1 x ATmega328P x current measurement 38400 ok
    //Serial1.begin(76800);	//USART1 x ATmega328P x current measurement

    //Serial2.begin(115200);	//USART2 x HC05 BLuetooth

    //D4 = PG5 SW reset encoder counter
    pinGetLevel_init(); //with Changed=flag activated at initialization
    //PinTo1(PORTWxSW_ENC_RESET, PIN_SW_ENC_RESET); //Enable pull-up
    //ConfigInputPin(DDRxSW_ENC_RESET, PIN_SW_ENC_RESET);

    //Config TCNT0 CT Atmega2560
    TCNT0 = 0x00;
    TCCR0A = (1 << WGM01);//CTC mode
    TCCR0B = (1 << CS02) | (0 << CS01) | (1 << CS00); //CTC, PRES=1024
    OCR0A = CTC_SET_OCR_BYTIME(10e-3, 1024); //TMR8-BIT @16MHz @PRES=1024-> BYTIME maximum = 16ms
    TIMSK0 |= (1 << OCIE0A);
    sei();
}
int8_t send(float m, float v, float current)
{
    Serial.print(m,2);
    Serial.print(",");
    //Serial.println(v*1000,1);//represente en millivolts

    Serial.print(v*1000,1);//represente en millivolts
    Serial.print(",");
    Serial.println(current,2);

    return 0;
}


void loop()
{
	uint8_t c;
	static int8_t sm0 = 0;
	static int8_t trama_counter=0;
	static char pcurrentBuffered[20];
	//

	int8_t SW=0;
    static int8_t counter1;

    //-------------------------
    if (isr_flag.f10ms)
    {
        isr_flag.f10ms = 0;
        main_flag.f10ms = 1;
    }
    //-------------------------
	numPulses_diff = rotaryCount - rotaryCount_last;//el software estaria siempre recibiendo el rotaryCount
	if (numPulses_diff >= numPulsesIn_ADQ_KMETERS)
	{
		rotaryCount_last = rotaryCount;

		//Serial.print("pdiff: ");Serial.print(numPulses_diff);
		meters = rotaryCount * ENC_RESOL;
		//Serial.print(meters);// Serial.println(" m");
		volts = voltageMeas();
		//send(meters,volts);
		send(meters,volts, current);
	}
	//Serial.println(voltageMeas());

    //----------------------
    if (main_flag.f10ms)
    {
        if (++counter1 == 2)    //20ms
        {
            counter1 = 0;
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

    main_flag.f10ms = 0;

    //Current measurement
    if (Serial1.available() > 0)
    {
    	 c = Serial1.read();

    	 if (sm0 == 0)
    	 {
    		 if (c == TRAMA_START)
    		 {
    			 trama_counter = 0x00;
    			 sm0++;
    		 }
    	 }
    	 else
    	 {
    		 if (c == TRAMA_END)
			 {
    			 pcurrentBuffered[trama_counter] = '\0';
				 current = strtod(pcurrentBuffered, NULL);

				 //Serial.println(current,2);
				 sm0 = 0x00;
			 }
    		 else
    		 {
    			 pcurrentBuffered[trama_counter++] = c;
    		 }

    	 }
    }
}

ISR(TIMER0_COMPA_vect)
{
    isr_flag.f10ms = 1;
}
