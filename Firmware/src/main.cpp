#include <main.h>

void setup() 
{
    RTC_init();
    prepare_sleep();

    #ifndef NO_SERIAL
        Serial.begin(9600);
    #endif
    
    // Circuit is made to scale a 0-10v signal to 0-2.5V
    analogReference(INTERNAL2V5);

    // Output has 4x gain to get 0-10v
    DACReference(INTERNAL2V5);
    
    Wire.begin();
    delay(15);
    sht40.init();

}

void RTC_init()
{
  /* Initialize RTC: */
  while (RTC.STATUS > 0)
  {
    ;                                   /* Wait for all register to be synchronized */
  }
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc;    /* 32.768kHz Internal Ultra-Low-Power Oscillator (OSCULP32K) */

  RTC.PITINTCTRL = RTC_PI_bm;           /* PIT Interrupt: enabled */

  RTC.PITCTRLA = RTC_PERIOD_CYC32768_gc /* RTC Clock Cycles 16384, resulting in 32.768kHz/16384 = 1Hz */
  | RTC_PITEN_bm;                       /* Enable PIT counter: enabled */
}

void prepare_sleep()
{
    DAC0.CTRLA |= _BV(DAC_RUNSTDBY_bp); // Ensure DAC is enabled during sleep

    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable();

    // Ensure pins aren't floating 
    for (uint8_t i=0; i<sizeof(unused_pins); i++)
    {
        pinMode(unused_pins[i], INPUT_PULLUP);
    }
}

void go_sleep()
{
#ifndef NO_SERIAL
    Serial.flush();
#endif
    sleep_cpu();
}


uint8_t outv = 0;

uint16_t last_value_open = 0;
uint16_t last_value_close = 0;

void loop() 
{   
    uint16_t closed_ticks = analogRead(pin_close);
    uint16_t open_ticks = analogRead(pin_open);

    sht40.poll_temp_humi();
    uint16_t temperature = sht40.get_temp();
    // temperature is a uint. If it's negative it'll be >600C
    if (temperature > max_temp)
    {
        temperature = 0;
    }

    if (closed_ticks < (dial_range[0]>>1))
    {
        // Manual close override
        analogWrite(pin_dac, pos_out_range[0]);
        force_close = true;
        force_open = false;
    }

    if (open_ticks < (dial_range[0]>>1))
    {
        // Manual open override
        analogWrite(pin_dac, pos_out_range[1]);
        force_close = false;
        force_open = true;
    }
    else if(!force_close && !force_open)
    {
        

        sp_closed = map(
                    closed_ticks, 
                    dial_range[0], dial_range[1], 
                    closed_range[0], closed_range[1]
                );
        
        sp_open = map(
                    open_ticks,
                    dial_range[0], dial_range[1], 
                    open_range[0], open_range[1]
                );     
        
        new_pos = get_pos_out(temperature, sp_closed, sp_open);

        if(abs(new_pos-old_pos) > pos_deadband_ticks)
        {
            old_pos = new_pos;
            deadline = millis() + pos_timeout;
            analogWrite(pin_dac, new_pos);
        }
    }
    else
    {
        if (temperature < sp_closed)
        {
            force_close = false;
        }
        if (temperature > sp_open)
        {
            force_open = false;
        }
    }


    
#ifndef NO_SERIAL
    Serial.print("T:"); Serial.print(sht40.get_temp());
    Serial.print(",RH:"); Serial.println(sht40.get_humi());
#endif

    // If position error too big for too long, re-init actuator
    if (abs(new_pos-old_pos) > pos_deadband_ticks && millis() > deadline)
    {
        deadline = millis() + pos_timeout;
        digitalWrite(pin_dac, 0);
        delay(500);
    }

    // for example have a shrinking max allowable error over time.
    //      - meh
    go_sleep();
}


uint8_t get_pos_out(uint16_t temp, uint16_t sp_closed, uint16_t sp_open)
{
    if (sp_open > sp_closed) // Servo mode
    {
        temp = constrain(temp, sp_closed, sp_open);

        return map(temp, sp_closed, sp_open, pos_out_range[0], pos_out_range[1]);

    }
    else // Hysteresis mode
    {
        if (temp > sp_open)
        {
            return pos_out_range[1];
        }
        else if (temp < sp_closed)
        {
            return pos_out_range[0];
        }
    }
}

ISR(RTC_PIT_vect)
{
  RTC.PITINTFLAGS = RTC_PI_bm;          /* Clear interrupt flag by writing '1' (required) */
}