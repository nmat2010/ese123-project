// compile this file with the
// Project Properties / XC8 compiler / Preprocesing and messages / Use CCI syntax
// checkbox selected.

// to retain user row calibration constants program this project with the
// Project Properties / Snap / Program Options / Erase All Before Program
// checkbox unchecked. 

// choose seconds mode: binary, bar_graph, or all
#define binary

// define ups if the ESE272 back up power module is installed
//#define ups

// these are the ADC calibration factors unique to your hardware
#define AVDD_cal 1.0000
#define curr_cal 1.0000

#include "functions.h"

int main(void) 
{
    // declare variables
    unsigned char alarm_armed = 0;
    unsigned char alarm_hr = 8;
    unsigned char alarm_min = 0;
    unsigned char alarm_sounding = 0;
    unsigned char buttons;
    unsigned char charge_mode = 1;
    unsigned char charge_state = 0;
    unsigned char time_mode; 
    unsigned char wake_hr;  
    float end_current = 0; 
   
    // setup system
    setup();        // setup microcontroller  
    self_test ();   // test the hardware (also enables interrupts)
    splash ();      // display splash screen
    
    //set time
    time_mode = set_time ();    // set the time and get the 12/24 hour mode
  
    // repeat this loop forever
    while (1)
    {
        // check for USB disconnect
        if ((charge_state != 0) && (USB_connected() == 0))
        {
            USB_off ();                 // USB off
            charge_state = 0;           // terminate charge
            disp_4char ("End ");        // END message
            delay_ms (1000);
            disp_float (total_energy / 1000, 1); // display energy delivered in kJ
            delay_ms (4000);
        }
        
        // check for USB connect
        if ((charge_state == 0) && (USB_connected() == 1))
        {
            total_energy = 0;               // initialize charge energy 
            charge_state = 1;               // charging
            loop_count = 0;                 // reset loop counter
            charge_mode = choose_charge (); // select the charge mode
            if ((charge_mode == 1) || (charge_mode == 2))
            {
                end_current = 0;            // placeholder value for end_current
                charge_state = 4;           // learn charge current
            }
        }
        
        // check for low system voltage or a power fail flag from PD0 interrupt
        if ((qavdd < 4.4) || (pwr_ok == 0))
        {
            // set diagnostic flag for scope timing measurements
            PORTD_OUTSET = BOD_pulse;       // diagnostic bit high
            
            // voltage is due to falling VDD: SCRAM
            PORTC_OUTSET = 0b00111111;      // turn off display and USB port
            PORTE_OUTCLR = alrLED;          // turn off alarm light
            RTC_PITINTCTRL = 0b00000000;    // digit interrupt disabled

            // trap in sleep until power is restored 
            do
            {
                set_sleep_mode (0);
                sleep_mode ();              // go to sleep 
            }
            while (measure_AVDD () < 4.6);
            
            // power restored, enable display interrupt
            RTC_PITINTCTRL = 0b00000001;    // digit interrupt enabled
            
            // power restored reset power flag
            pwr_ok = 1; 
            
            // clear diagnostic bit
            PORTD_OUTCLR = BOD_pulse;       // diagnostic bit low 
        }
        
        // check for alarm 
        if (alarm_armed == 1)                           // is alarm armed?
        {
            if ((hr == alarm_hr) && (min == alarm_min)) // is it the alarm time?
            {
                if (alarm_sounding == 0)                // is it already reset?
                    alarm_sounding = 1;                 // sound alarm
            }
            else 
                alarm_sounding = 0;                     // reset alarm after 1 minute
        }
        
        // read and mask buttons
        buttons = (PORTE_IN & b_mask);
        if ((buttons != none) && (alarm_sounding == 1))
        {
            alarm_sounding = 2;             // silence alarm
            wait_up();                      // wait for button release
            buttons = 0b00000000;           // reset buttons
        }
            
        // UP button: arm alarm
        if (buttons == up)                  // UP button pressed
        {
            if (alarm_armed == 0)           // toggle alarm arm
                alarm_armed = 1; 
            else
                alarm_armed = 0;
            wait_up();
        }
        
        // OK button: set alarm time
        if (buttons == ok)                      // OK button pressed
        {
            wait_up();                          // wait for button release
            set_alarm (&alarm_hr, &alarm_min, time_mode);   // set new alarm time
            alarm_armed = 1;                    // arm alarm
        }
        
        // DOWN button: set time of day
        if (buttons == down)                // DOWN button pressed
        {
            // display number of days since last set 
            disp_float (last_set / (24.0 * 60.0 * 60.0), 1);
            wait_up ();                     // wait for button release
            time_mode = set_time ();        // set time and quartz calibration factor
        }
        
        // no buttons are pressed - display stuff
        if (buttons == none)
        {
            // set alarm armed LED
            if (alarm_armed == 1)           // keep LED synchronized with mode
                PORTE_OUTSET = alrLED;
            else
                PORTE_OUTCLR = alrLED; 
            
            if (alarm_sounding == 1)        // sound the alarm!
            {
                PORTF_OUTSET = buzz; 
                delay_us (70);             // ~4kHz with overhead
                PORTF_OUTCLR = buzz; 
                delay_us (50);             // ~4kHz with overhead
            }
                
            if ((charge_state == 1) || (charge_state == 3) || (charge_state == 4))   
            {
                // charging: the display alternates between time and power
                if (sec % 2)                   
                    disp_fast_float (current*avdd); // display the power (odd seconds)
                else
                    disp_time (time_mode);          // display the time (even seconds)
            }
            else
            {
                // not charging: just display the time
                disp_time (time_mode);  // display the time
            }
        }
        
        // check for charge termination
        // current must be low for multiple consecutive readings
        //
        // charge modes:
        //      mode 1 : terminate charge at threshold
        //      mode 2 : terminate and resume 1 hour before alarm time
        //      mode 3 : charge continously 
        //
        // charge states:
        //      state 0 : USB disconnected
        //      state 1 : USB connected and charging
        //      state 2 : USB connected but charging is paused
        //      state 3 : USB connected and charging is resumed
        //      state 4 : USB connected and charging, learning end_current
        //
        // loop_count is incremented once per second by the RTC interrupt
        if (charge_state == 1)
        {
            if (PORTC_OUT & USBfet)
            {
                // USB port is off - turn it on
                USB_on ();
            }
            // charging, compare current to threshold
            if (current > end_current)
                loop_count = 0;         // current is over the setpoint: reset loop counter
        }
            
        if ((charge_mode == 1) || (charge_mode == 2))   // terminate charge in these modes
        {
            if ((loop_count >= term_loops) && (charge_state == 1))
            {
                USB_off ();                 // turn USB port off
                loop_count = 0;             // reset loop counter
                charge_state = 2;           // charge paused
                if (charge_mode == 1)
                {
                    disp_4char ("End ");    // charging is finished
                    delay_ms (1000);
                }
                if (charge_mode == 2)
                {
                    disp_string ("PAUSE");  // charging is paused
                }
            }
        }
        
        if (charge_state == 3)
        {
            if (PORTC_OUT & USBfet)
            {
                // USB port is off - turn it on
                USB_on ();
            }
        }
        
        if (charge_state == 4)
        {
            if (PORTC_OUT & USBfet)
            {
                // USB port is off - turn it on
                USB_on ();
            }
            if (loop_count == learn_loops)
            {
                // compute termination current
                end_current = (curr_thresh * (total_energy / avdd / learn_loops));
                charge_state = 1;                   // resume charging
                disp_4char ("SEt ");                // display setpoint
                delay_ms (1000);
                disp_float (end_current * avdd, 1); // display threshold power
                delay_ms (2000);
                loop_count = 0;                     // reset loop counter
            }
        }
          
        // check for wake up on charge mode 2
        if ((charge_mode == 2) && (charge_state == 2))
        {
            // calculate wake hour one hour before alarm time
            if (alarm_hr == 0)
                wake_hr = 23;
            else
                wake_hr = alarm_hr - 1; 
            if ((hr == wake_hr) && (min == alarm_min))
            {
                charge_state = 3;       // resume charging
            }
        }
    }
}


// to do: 
//      fix total energy readout = 0 after power interruption
//      add temperature display


// done:
//      improve button response
//      improve alarm time setting
//      restore brownout protection
//      improve charge termination criteria
//      change alarm parameters to achieve 4kHz
//      moved USB self-test before the crystal test
//      fixed flicker visible when charging
//      fixed low frequency alarm sound while charging
//      fixed display when charging and using binary seconds