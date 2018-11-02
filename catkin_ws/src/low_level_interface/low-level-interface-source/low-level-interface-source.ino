/*
  The Clear BSD License

  Copyright (c) 2018 Philipp Rothenhäusler
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted (subject to the limitations in the disclaimer
  below) provided that the following conditions are met:

       * Redistributions of source code must retain the above copyright notice,
       this list of conditions and the following disclaimer.

       * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

       * Neither the name of the copyright holder nor the names of its
       contributors may be used to endorse or promote products derived from this
       software without specific prior written permission.

  NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
  THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
  IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/


/*
        ########################################
        ########################################
        Configuration for Atmega 328p - Arduino Uno Rev.2
        ########################################
        ########################################


        ########################################
        Input / Output
        ########################################
        Digital input: 17, 8, 7, 4, 2   (PCINT: 11, 0, 23, 20, 18)
        Digital output: 10, 9, 6, 5, 3  (Servo library using only one timer for a hardware based PWM, Fast PWM Mode using OCRA and ORCB)
 
        
        ########################################
        Basics for Arduino UNO (Page reference referring to ATMEGA 328p datasheet)
        ########################################
        
        PIN-Overview (p.14) - see port, digital, alternate functions etc.
          DDxn - configure direction of pin (1 output, 0 input)
          PORTxn  - INPUT configure pullup (1 - activated)
                  - OUTPUT configure 1 - high and 0 - low
          PINxn - toggle value
        
        To enable the Pin Interrup set
          SREG set I-bit (p.9)
          PCICR set PCIE1
          PCMSK0-2 set corresponding pin with PCINT0-23 (p.91)
        
        Three different interrupt vectors
        PCICR
          PCIE0
          PCIE1
          PCIE2    
             
        Pin to interrupt map: (p.95)
        D digital, A analog
        D8-D13                = PCINT 0-5     = PCIR0 = PB = PCIE0 = pcmsk0
        A0-A5 (D14-D19)       = PCINT 8-13    = PCIR1 = PC = PCIE1 = pcmsk1
        D0-D7                 = PCINT 16-23   = PCIR2 = PD = PCIE2 = pcmsk2

        
        ########################################
        RC - REMOTE CONTROL TQi:
        ########################################
        MODES OF OPERATION:
                CRAWL-MODE:
                Reverse without braking (immediate reverse)
        CHANNEL-CONFIGURATION FOR (p.12)        
        CH1 - Steering          [1,2] = [left, right] angle
        CH2 - Throttle          [1,2] = [reverse, forward] velocity
        CH3 - Transmisssion     [1,2] = [low, high] gear
                Shift Channel - (Red Switch on same height as throttle lever)
        CH4 - Differential F    [1,2] = [unlocked, locked] differential
                T-Lock Switch - (top lever) forward
        Ch5 - Differential R    [1,2] = [unlocked, locked] differential
                T-Lock Switch - (top lever) forward
        
        DIFFERENTIAL-LOGIC (1ms = false, 2ms = true)
        CH4     |       CH5     |       CONFIG          |       lever-position          |
        (ms)    |      (ms)     |        Unit           |             -                 |
        2       |       1       |    f&r unlocked       |            BACK               |
        1       |       1       |  f. locked r. unlock  |           MIDDLE              |
        1       |       2       |     f&r locked        |            FRONT              |
                T-Lock Switch - (top lever) forward
        
        
        Channel 4 inverted in State Machine for Switch Logic
        DIFFERENTIAL-LOGIC ADAPTED (1ms = UNLOCKED, 2ms = LOCKED)
        CH4     |       CH5     |       CONFIG          |       lever-position          |
        (ms)    |      (ms)     |        Unit           |             -                 |
        1       |       1       |    f&r unlocked       |            BACK               |
        2       |       1       |  f. locked r. unlock  |           MIDDLE              |
        2       |       2       |     f&r locked        |            FRONT              |
        
        
        ########################################
        ROS configuration (1355 bytes)
        ########################################
                make library
                publish with same frequency, no interrupts while sending msgs
                receive control signals
                rc interrupt if control signal flips
                send actuated control signals 40 Hz
                send V, I, RESET
        
        ########################################
        to do
        ########################################
                - replace all timer constants with #defines
                - comment out definitions according to config
                - add input filter to reject outliers (median filter) or hardcode threshold in order to apply new i2c output
                - for v1.1 define header files and cpp files for custom_interrupts, external actuation, internal actuation, ros system, measurement, alarm, 
*/

// General Included Libraries
#include <stdint.h>
#include <avr/interrupt.h>
#include <Arduino.h>

/*
   DEFINE MACROS
*/
// FLASH CONFIGURATION                          // Serial print-outs INACTIVE, ACTIVE [0,1]
        #define ACTUATE_EXTERNAL 1              // Use PCA9685 - external timer chip (i2c)
        #define ACTUATE_INTERNAL 0              // send signal to servo objects 
        #define ROS_ACTIVE 1                    // include ros publisher in build
        #define MEASUREMENT 1                   // Activate current and voltage measurement
        #define ALARM_ACTIVE 0                  // Activate Alarm with Buzzer in case of low voltage
        #define DIAGNOSIS 0                     // Allow serial-prints (only in combination with ROS_ACTIVE 0)
        #define PRINT_BITS 0                    // Output some register information (see print function)
        #define PRINT_CTRL_MODE 1               // Visualise the current control mode
        #define PRINT_CTRL 0                    // Print logic for control bit switching
        #define PRINT_INPUT_PERIODS 1           // Show the measured input time periods and their channel dependent idle state
        #define PRINT_INPUT_PPMS 0              // Show the PPM values for each channel based on the input period
        #define PRINT_OUTPUT 0                  // Show the PPM values for each channel based on the input period
        #define PRINT_MEASUREMENT 0             // Print Measurement data
        #define PRINT_FILTER 0                  // Print Moving average debug data
        
        #define EXTERNAL_PWM_DRIVER_ADDR 0x40     // default address without any solder bridges for adress selection
        #define F_IO 16000000
        #define BAUD_RATE 115200
        #define ANALOG_RES 1024                                 // (10bit)
        #define VALUE_IS(PINX,PINn) ((PINX&PINn)==PINn)

#if ROS_ACTIVE
        #include <ros.h>
        #include <low_level_interface/lli_ctrl_request.h>
        #include <low_level_interface/lli_ctrl_actuated.h>
#endif /* ROS_ACTIVE */ 
       
// INCLUDE CONFIGURATION SPECIFIC LIBRARIES
#if ACTUATE_INTERNAL
        #include <Servo.h>
#endif /* ACTUATE_INTERNAL */

#if ACTUATE_EXTERNAL
        #include <Adafruit_PWMServoDriver.h>
#endif /* ACTUATE_EXTERNAL */

// CONTROL BITS 
        #define V0(REGISTER) ((REGISTER&(0x01)))
        #define V1(REGISTER) ((REGISTER&(0x02))>>1)
        #define V2(REGISTER) ((REGISTER&(0x04))>>2)
        #define V3(REGISTER) ((REGISTER&(0x08))>>3)
        
        #define IS_L1(REGISTER) (V0(REGISTER) && V1(REGISTER))
        #define IS_L2(REGISTER) (V2(REGISTER) && V3(REGISTER))
        #define NOT_L1(REGISTER) ((!V0(REGISTER)) && (!V1(REGISTER)) )
        #define NOT_L2(REGISTER) ((!V2(REGISTER)) && (!V3(REGISTER)) )
        #define NONE_L1(REGISTER) (V0(REGISTER) ^ V1(REGISTER))
        #define NONE_L2(REGISTER) (V0(REGISTER) ^ V1(REGISTER))
        #define L1_IS_NOT_L2(REGISTER) ( (IS_L1(REGISTER) && NOT_L2(REGISTER))|| (NOT_L1(REGISTER) && IS_L2(REGISTER)))
        #define L1_IS_L2(REGISTER) ( (IS_L1(REGISTER) && IS_L2(REGISTER))|| (NOT_L1(REGISTER) && NOT_L2(REGISTER)) )
        
        #define S0(REGISTER) ((REGISTER&(0x10))>>4)
        #define S1(REGISTER) ((REGISTER&(0x20))>>5)
        #define S2(REGISTER) ((REGISTER&(0x40))>>6)
        #define NO_STATE(REGISTER) (!(bool(REGISTER&(0x70)) ))
        #define READ_OK(REGISTER) ((REGISTER&(0x80))>>7)
        #define SET_S0(REGISTER) (REGISTER|0x10)
        #define SET_S1(REGISTER) (REGISTER|0x20)
        #define SET_S2(REGISTER) (REGISTER|0x40)
        #define SET_READ_OK(REGISTER) (REGISTER|(0x80))
        #define RESET_READ_OK(REGISTER) (REGISTER&(~0x80))
        #define CLEAR_STATES(REGISTER) (REGISTER&(0x8F))
        #define CLEAR_VALUES(REGISTER) (REGISTER&(~0xF0))
        #define RESET_ALL_VALUES(REGISTER) (REGISTER|(0x05))
        #define CLEAR_L1_VALUES(REGISTER) (REGISTER&( (0xF1) | (0x0C)) )
        #define CLEAR_L2_VALUES(REGISTER) (REGISTER&( (0xF3) | (0x04)) )
  
// IDENTIFIERS - OUTPUT
        #define PWM_LOW 1000.0
        #define PWM_NEUTRAL 1500.0
        #define PWM_HIGH 2000.0
        #define PWM_THRESHOLD 500.0
        #define PWM_CTRL_THRESHOLD 200.0                          // margin around center value for clearly defined logic states
        #define PWM_EMERGENCY_CTRL_SWITCH_LOW 1150.0              // if in CONTROL_SW mode acceleration or deceleration outside of these margins, switches immediately to CONTROL_REMOTE
        #define PWM_EMERGENCY_CTRL_SWITCH_HIGH 1850.0
        #define PWM_EXTERNAL_MIN_TICK 204.0 // 1ms --> 1/20 --> 0.05
        #define PWM_EXTERNAL_MAX_TICK 410.0
        #define PWM_EXTERNAL_RES 4096.0
        //define PWM_INTERNAL2INTERAL //40,9-82
        
// IDENTIFIERS - INPUT
        //adc -- voltage
        #define ADC_VOLTAGE 0x10
        #define ADC_BIT_RES 1024.0
        #define ADC_GAIN 4.2082         // 13.3/3.3
        #define ADC_REF 5.0               //
        #define POWER_SUPPLY_VOLTAGE 20 // MAX ADC VALUE

        //adc -- voltage
        #define ADC_CURRENT 0x10
        
        // alarm
        #define ADC_PIN 0x10            // Port B PB4, di12
        #define F_ALARM 4000            // Hz
        #define T_ALARM1 100            // ms
        #define T_ALARM2 500            // ms
        #define T_ALARM3 1000           // ms
        #define F_IO 16000000           // Hz
        #define Prescaler 32            //
        #define ALARM_TICK_COUNT 163 //
          
// BIT MASK
        #define BIT0 1
        #define BIT1 2
        #define BIT2 4
        #define BIT3 8
        #define BIT4 16
        #define BIT5 32
        #define BIT6 64
        #define BIT7 128

// INPUT PINS - DIGITAL                 //      // (PCINT)      // timer        //common interrupt vecor       -- see setup() pin definition for interrupts
        #define INP_STEER 17            // A3        // (11)         // ti 1         //2
        #define INP_VEL 8               // PB0       // (0)          // ti 2         //1
        #define INP_TRANS 7             // PD7       // (23)         // ti 3         //3
        #define INP_DIFF_F 4            // PD4       // (20)         // ti 4         //3
        #define INP_DIFF_R 2            // PD2       // (18)         // ti 5         //3

// OUTPUT PINS (PWM)                    // used for servo object initialisation
        #define OUT_STEER 10            // PB2
        #define OUT_VEL 9               // PB1
        #define OUT_TRANS 6             // PD6
        #define OUT_DIFF_F 5            // PD5
        #define OUT_DIFF_R 3            // PD3
        #define OUT_LED 13              // Internal LED
        #define OUT_CTRL 12             // PB4

  
/*
   DEFINE BOOLEAN VARIABLES
*/
  volatile bool CONTROL_SW = false;
  volatile bool CONTROL_REMOTE = true;

  volatile bool PWM_REMOTE_INPUT_UPDATED[5] = {false, false, false, false, false};
  volatile bool PWM_SW_INPUT_UPDATED = false;
  volatile bool IDLE_ACTIVE[5] = {false, false, false, false, false};
  volatile bool IDLE_SW_ACTIVE = false;
  volatile bool UPDATE_OUTPUT = false;

  // interrupt logic variables
  volatile bool TMP_CONDITION = false;
  volatile bool PWM_READ_HIGH[5] = {false, false, false, false, false};


/*
   DEFINE DATA VARIABLES
*/
// ADC variables
volatile float voltage = 0;
volatile float current = 0;

  volatile uint8_t ctrl_code = 0;
  volatile uint8_t CTRL_REG = 0x05;                     // Control mode byte for switching modes/variable
  volatile uint16_t ACTUATE[5] = {PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL};
  volatile uint16_t T_REMOTE_in[5] = {PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL};   // REMOTE PPM input period
  volatile uint16_t T_SW_in[5] = {PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL, PWM_NEUTRAL};           // SW PPM input period
  volatile int8_t PPM_REMOTE_in[5] = {0,0,0,0,0};       // REMOTE PPM input 
  volatile int8_t PPM_SW_in[5] = {0,0,0,0,0};           // SW PPM input

  // REMOTE interrupt timer
  volatile long TMP_t[5] = {0,0,0,0,0};
  volatile uint16_t TMP_T[5] = {0,0,0,0,0};

  // REMOTE filter data in case of missed interrupts
  volatile long ti_in[5] = {0, 0, 0, 0, 0};


  // REMOTE interrupt - logic variables
  volatile uint8_t schedule_ti2 = 0;                                    // fixes missed interrupts on shared interrupt vector
  volatile uint8_t schedule_ti4 = 0;
  #define schedule_ti2_n_idle_max 5
  #define schedule_ti4_n_idle_max 2
  //volatile uint8_t schedule_ti2_n_idle_max = 5;
  //volatile uint8_t schedule_ti4_n_idle_max = 2;
  volatile uint8_t idx = 0;                                             // Interrupt timer index [0,4]
  volatile uint8_t seq_n = 0;
  volatile uint8_t idx_int = 0;                                         // Interrupt index [0:2]
  volatile uint8_t PIN_INT_MASK[5] = {BIT3, BIT0, BIT7, BIT4, BIT2};    // ti dependend bit
  volatile uint8_t PIN_MASK[3] = {0x00, 0x00, 0x00};                    // mask for active pins in corresponding interrupt
  volatile uint8_t PIN_V_CURR[3] = {0x00, 0x00, 0x00};                  // B, C, D
  volatile uint8_t PIN_V_PREV[3] = {0x00, 0x00, 0x00};
  volatile uint8_t PIN_V_COMP[3] = {0x00, 0x00, 0x00};

/*
   DEFINE TIMING VARIABLES
*/
  // ROS publish timing
  volatile long ti_pub = 0;
  #define T_pub 25
  //volatile uint16_t T_pub = 100;                                // [T_pub] = s 10E-3
  
  // Control mode timing
  volatile long ti_ctrl = 0;
  #define T_ctrl 1000
  //volatile uint16_t T_ctrl = 1000;                              // [T_pub] = s 10E-3
    
  // Output update frequency
  volatile long ti_out = 0;
  #define T_out 10
  //volatile uint16_t T_out = 500;                                // [T_pub] = s 10E-3

  // REMOTE idle timing
  volatile long ti_remote_idle[5] = {0, 0, 0, 0, 0};
  #define T_remote_idle 1000
  //volatile uint16_t T_remote_idle = 1000;                       // [T_remote_idle] = s 10E-3

  // SW idle timing
  volatile long ti_sw_idle = 0;
  #define T_sw_idle 1000
  //volatile uint16_t T_sw_idle = 1000;                           // [T_remote_idle] = s 10E-3

  // DIAGNOSIS timing
  volatile long ti_diag = 0;
  #define T_diag 500
  //volatile uint16_t T_diag = 500;                               // [T_diag] = s 10E-3

  // LED timing
  volatile long ti_led = 0;
  volatile uint16_t T_led[5] = {50,100,1000,2000,4000};       // [T_led] = s 10E-3 [STATE 1, STATE 2, STATE 3, STATE 4, STATE 5]
  volatile uint8_t IDX_LED_MODE = 1;                            // STATE INDICATOR

  // Measurment timing
  volatile long ti_sense = 0;
  #define T_sense 1000
  //volatile uint16_t T_sense = 500;                              // [T_led] = s 10E-3

  // Alarm timing
  volatile long ti_alarm = 0;
  #define T_alarm 500
  volatile long ti_alarm_f = 0; // measure frequency to validate configuration
  float f_alarm = 0;
  uint16_t f_cnt = 0;
  #define F_CNT_MAX 1


/*
   DEFINE MEDIAN_FILTER
*/

#define MEDIAN_HORIZON 6.0
#define MEDIAN_HORIZON_IDX (MEDIAN_HORIZON-2)
/* moved to function as static variables - saves 28 bytes
volatile uint16_t temp_lp[2][5]={{PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}, {PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}};
//volatile uint16_t temp_lp[2][10]={{PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}, {PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL, PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}};
volatile uint32_t sum[2] = {PWM_NEUTRAL, PWM_NEUTRAL}; */
volatile bool ros_status = false;


/*
Define ROS network
*/
#if ROS_ACTIVE
  void cb_ctrl_request(const low_level_interface::lli_ctrl_request data){
     
    ti_sw_idle = millis();
    if (CONTROL_SW){
      PPM_SW_in[0] = data.steering;
      PPM_SW_in[1] = data.velocity;
      PPM_SW_in[2] = data.transmission;
      PPM_SW_in[3] = data.differential_front;
      PPM_SW_in[4] = data.differential_rear;
      ctrl_code = data.ctrl_code;
      
    } else{;}
    IDLE_SW_ACTIVE = false;
    PWM_SW_INPUT_UPDATED = true;
  }
  ros::NodeHandle_<ArduinoHardware, 1, 1, 240, 240> nh; // Important to not overload arduino with to big messages
  //ros::NodeHandle nh;
  low_level_interface::lli_ctrl_request MSG_REQUEST;
  low_level_interface::lli_ctrl_actuated MSG_ACTUATED;
  ros::Publisher ctrl_actuated("/lli/ctrl_actuated",&MSG_ACTUATED);
  ros::Subscriber<low_level_interface::lli_ctrl_request> ctrl_request("/lli/ctrl_request",&cb_ctrl_request );

#endif /* ROS_ACTIVE */


#if ACTUATE_EXTERNAL
        Adafruit_PWMServoDriver external_pwm = Adafruit_PWMServoDriver();
#endif /* ACTUATE_EXTERNAL */

#if ACTUATE_INTERNAL  
  Servo output[5];
#endif /* ACTUATE_INTERNAL */

/*
   Clear Screen
*/
inline void clear_screen() {
#if DIAGNOSIS
  for (int i = 0;i<15;i++){
      Serial.println(""); 
  }
#endif /* DIAGNOSIS */
} 

/*
   Print binary
*/
inline void print_binary(uint8_t binary_number) {
#if DIAGNOSIS
  for (uint8_t k = 0x80; k; k >>= 1) {
    if (binary_number & k) {
      Serial.print("1");
    }
    else {
      Serial.print("0");
    }
  }
  Serial.println("");
#endif /* DIAGNOSIS */
}

/*
  Transate period to pwm
 */
inline int8_t period_to_pwm(volatile uint16_t input){
        return int8_t( ( float((input-PWM_NEUTRAL))/(PWM_NEUTRAL-PWM_LOW) )*100.0 );
}

/*
  Translate pwm to period
 */
inline uint16_t pwm_to_period(volatile int8_t input){
        return uint16_t( (input/100.0)*float(PWM_NEUTRAL-PWM_LOW) + PWM_NEUTRAL );
}

/*
   saturate read pwm to keep period in range [1,2] ms and frequency 50 Hs with T=20ms
*/
inline void IS_VALID(volatile uint16_t &period, volatile boolean &valid) {
  valid = true;
  if (((PWM_LOW - PWM_THRESHOLD) < period) && (period <= PWM_LOW)) {
    period = PWM_LOW;

  }
  else if ((PWM_HIGH <= period) && (period < (PWM_HIGH + PWM_THRESHOLD))) {
    period = PWM_HIGH;
  }
  else if ((PWM_LOW < period) && (period < PWM_HIGH)) {
    period = period;
  }
  else {
    period = PWM_NEUTRAL;
    valid = false;
  }
}

/*
 * Control mode switching logic for channel 4 and channel 5 (TOP-switch)
 * 
        INVERT LOGIC traxxas definition 
         * including invert logic here allows keeping the output confirming to the traxxas definition
         * front unlocked = 1 front locked = 0
         * rear unlocked = 0 rear locked = 1
         * inverting front channel 4 gives for both channels
         * locked = 1 unlocked = 0
         * 
        
 */
inline void trigger_channel_logic_state(volatile uint8_t &variable, volatile uint16_t &T, uint8_t &shift){
        // invert channel 4 (only case with shift = 0 and shift = 2 )
        boolean INVERT = false;
        if ((shift==0)||(shift==2)){
                INVERT = true;
        }
        else{
                INVERT = false;        
        }
        
        if (T < (PWM_NEUTRAL - PWM_CTRL_THRESHOLD)){ // LOW
                variable = SET_READ_OK(variable);
                if (INVERT) { // HIGH
                        variable |= (0x01 << shift);
                }
                else { // LOW
                        variable &= (~(0x01 << shift));
                }
               
                
        }
        else if ((PWM_NEUTRAL + PWM_CTRL_THRESHOLD) < T){ // HIGH
                variable = SET_READ_OK(variable);
                if (INVERT) { // LOW
                        variable &= (~(0x01 << shift));
                }
                else { // HIGH
                        variable |= (0x01 << shift);
                }
        }
        else {
                variable = RESET_READ_OK(variable);
        }
}

/*
    Interrupt Service Routines
    Use Interrupt Vector and determine which pin is changed

*/
ISR(PCINT0_vect) { //portb interrupts
  idx = 1;               //interrupt only used for one pin (velocity)  
  TMP_t[idx] = micros();                       
  idx_int = 0;                  //port b
  PIN_V_CURR[idx_int] = PINB & PIN_MASK[idx_int];
  PIN_V_COMP[idx_int] = PIN_V_CURR[idx_int] ^ PIN_V_PREV[idx_int];
                            
  if (PIN_V_COMP[idx_int]&PIN_INT_MASK[idx]) {  //ti1    
    if (VALUE_IS(PIN_V_CURR[idx_int], PIN_INT_MASK[idx])) {
      ti_in[idx] = TMP_t[idx];
      PWM_READ_HIGH[idx] = true;
    }
    else if (PWM_READ_HIGH[idx]) {
      TMP_T[idx] = TMP_t[idx] - ti_in[idx];
      PWM_REMOTE_INPUT_UPDATED[idx] = true;
      PWM_READ_HIGH[idx] = false;
    }
    else {
      PWM_READ_HIGH[idx] = false;
    }
  }
  else {;}
  PIN_V_PREV[idx_int] = PIN_V_CURR[idx_int];
}
ISR(PCINT1_vect) { //portc interrupts
  idx = 0;            //interrupt only used for one pin (steering)
  TMP_t[idx] = micros();
  idx_int = 1;              // portc
  PIN_V_CURR[idx_int] = PINC & PIN_MASK[idx_int];
  PIN_V_COMP[idx_int] = PIN_V_CURR[idx_int] ^ PIN_V_PREV[idx_int];
  if (PIN_V_COMP[idx_int]&PIN_INT_MASK[idx]) { //ti0
    if (VALUE_IS(PIN_V_CURR[idx_int], PIN_INT_MASK[idx])) {
      ti_in[idx] = TMP_t[idx];
      PWM_READ_HIGH[idx] = true;
    }
    else if (PWM_READ_HIGH[idx]) {
      TMP_T[idx] = TMP_t[idx] - ti_in[idx];
      PWM_REMOTE_INPUT_UPDATED[idx] = true;
      PWM_READ_HIGH[idx] = false;
    }
    else {
      PWM_READ_HIGH[idx] = false;
    }
  }
  else {
    ;// trigger error boolean flag Serial.println("Mysterious interrupt");
  }
  PIN_V_PREV[idx_int] = PIN_V_CURR[idx_int];
}
ISR(PCINT2_vect) { //non-time-sensitive interrupts
        
        PCIFR = 0x00;
  idx_int = 2;          // portd
  PIN_V_CURR[idx_int] = PIND & PIN_MASK[idx_int];
  PIN_V_COMP[idx_int] = PIN_V_CURR[idx_int] ^ PIN_V_PREV[idx_int];

  if (schedule_ti2 > 0) {
    PIN_V_COMP[idx_int] &= (~PIN_INT_MASK[2]);
    schedule_ti2 -= 1;
  }
  else {;}

  if (schedule_ti4) {
    PIN_V_COMP[idx_int] &= (~PIN_INT_MASK[4]);
    schedule_ti4 -= 1;

  }
  else {;}


  if ( (PIN_V_COMP[idx_int] & PIN_INT_MASK[2]) == PIN_INT_MASK[2]) { //ti2
    idx = 2;
    TMP_t[idx] = micros();
    if (VALUE_IS(PIN_V_CURR[idx_int], PIN_INT_MASK[idx])) {
      ti_in[idx] = TMP_t[idx];
      PWM_READ_HIGH[idx] = true;
    }
    else if (PWM_READ_HIGH[idx]) {
      TMP_T[idx] = TMP_t[idx] - ti_in[idx];
      PWM_REMOTE_INPUT_UPDATED[idx] = true;
      PWM_READ_HIGH[idx] = false;
      schedule_ti2 = schedule_ti2_n_idle_max;
    }
    else {
      PWM_READ_HIGH[idx] = false;
    }
  }
  else if ((PIN_V_COMP[idx_int] & PIN_INT_MASK[3]) == PIN_INT_MASK[3]) { // ti3
    idx = 3;
    TMP_t[idx] = micros();
    if (VALUE_IS(PIN_V_CURR[idx_int], PIN_INT_MASK[idx])) {
      ti_in[idx] = TMP_t[idx];
      PWM_READ_HIGH[idx] = true;

    }
    else if (PWM_READ_HIGH[idx]) {
      TMP_T[idx] = TMP_t[idx] - ti_in[idx];
      PWM_REMOTE_INPUT_UPDATED[idx] = true;
      PWM_READ_HIGH[idx] = false;
    }
    else {
      PWM_READ_HIGH[idx] = false;
    }
  }
  else if ( (PIN_V_COMP[idx_int] & PIN_INT_MASK[4]) == PIN_INT_MASK[4] ) { //ti4
    idx = 4;
    TMP_t[idx] = micros();
    if (VALUE_IS(PIN_V_CURR[idx_int], PIN_INT_MASK[idx])) {
      ti_in[idx] = TMP_t[idx];
      PWM_READ_HIGH[idx] = true;
    }
    else if (PWM_READ_HIGH[idx]) {
      TMP_T[idx] = TMP_t[idx] - ti_in[idx];
      PWM_REMOTE_INPUT_UPDATED[idx] = true;
      PWM_READ_HIGH[idx] = false;
      schedule_ti4 = schedule_ti4_n_idle_max;
    }
    else {
      ti_in[idx] = TMP_t[idx];
      PWM_READ_HIGH[idx] = false;
    }
  }
  else {;}
  PIN_V_PREV[idx_int] = PIN_V_CURR[idx_int];
}

/*
 * Timer2 Interrupt - 4kHz
 */
ISR(TIMER2_COMPB_vect){
        TCNT2 = 0x00;
        OCR2B = ALARM_TICK_COUNT;
        f_alarm = 1 / ((micros()-float(ti_alarm_f))/1000000);
#if DIAGNOSIS
        Serial.println(f_alarm);
#endif /* DIAGNOSIS */
        ti_alarm_f = micros();
        f_cnt++;
        if (f_cnt>=F_CNT_MAX){
                toggle(ADC_PIN, PINB);  
                f_cnt=0;    
        }
        else {;}        
}

/*
 * Alarm - easy pin toggle Interrupt - 4kHz //TODO: Write quick Macro
 */
inline void toggle(uint8_t reg, uint8_t port){
        static boolean state = false;
        if (state){
                port &= (~reg);
        }
        else{
                port |= reg;
        }
}

/*
   update pwm from interrupts
*/
inline void update_input_pwm_periods() {
        for (uint8_t k = 0; k < 5; k++) {
                if (PWM_REMOTE_INPUT_UPDATED[k]) {
                        PWM_REMOTE_INPUT_UPDATED[k] = false; 
                        ti_remote_idle[k] = millis();                                           // reset idle counter
                        IDLE_ACTIVE[k] = false;
                        TMP_CONDITION = false;
                        IS_VALID(TMP_T[k], TMP_CONDITION);
                        if (TMP_CONDITION) {                                          // might include lock on interrupt ISR here
                                T_REMOTE_in[k] = TMP_T[k];
                                PPM_REMOTE_in[k] = period_to_pwm(T_REMOTE_in[k]);
                        }
                        else {;}
                        
                        // CONTROL SWITCHING LOGIC
                        if ( (k==3) || (k==4) ){
                                bool condition[2] = {false, false};
                                condition[0] = NO_STATE(CTRL_REG);
                                condition[1] = S0(CTRL_REG) || S1(CTRL_REG);
                                if (condition[0]){ // NO_STATE
                                        uint8_t j=k-3;
                                        trigger_channel_logic_state(CTRL_REG, T_REMOTE_in[k],j);
                                }
                                else if(condition[1]){ //STATE 0 OR STATE 1
                                        uint8_t j=k-1;
                                        trigger_channel_logic_state(CTRL_REG, T_REMOTE_in[k],j);
                                }
                                else{;} // NO_STATE OR STATE_2
                        }
                        else{;}
                        UPDATE_OUTPUT=true;
                }
                else {
                if (millis() - ti_remote_idle[k] > T_remote_idle) { // reset single channels if start idleing or loosing connection
                  ti_remote_idle[k] = millis(); // reset to only trigger output_update_periodically.
                  T_REMOTE_in[k] = PWM_NEUTRAL;
                  IDLE_ACTIVE[k] = true;
                  UPDATE_OUTPUT = true;
                }
                else {;}
                }
                PPM_REMOTE_in[k] = period_to_pwm(T_REMOTE_in[k]);
        }

        // NEW SW INPUT RECEIVED
        if (PWM_SW_INPUT_UPDATED) {
                PWM_SW_INPUT_UPDATED = false;               // reset trigger condition for next callback
                for(uint8_t k = 0;k<5;k++){
                        T_SW_in[k] = pwm_to_period(PPM_SW_in[k]);
                }
                UPDATE_OUTPUT = true;
                // CHECK FOR CONTROL STATES read_code(); to change control mode, reset energy calculation and others bit0 - idle, bit 1 - control mode, bit 2energy, bit3 - true (increases steering offset)
/* 
else if (CONTROL_REMOTE){ // USE with Caution, allows software override of remote interrupt (as start condition or to recover emergency interrupt)
ros_status = !ros_status;
digitalWrite(LED_BUILTIN, ros_status);
if((data.ctrl_code&0x20)>>5){
CONTROL_REMOTE=false;
CONTROL_SW=true;
} else {;}
if((data.ctrl_code&0x10)>>4){
CONTROL_REMOTE=false;
CONTROL_SW=true;
} else {;}
}
*/
        }
        else{
                if ((millis() - ti_sw_idle)> T_sw_idle) { // reset single channels if start idleing or loosing connection
                        ti_sw_idle = millis(); // reset timer to only trigger UPDATE_OUTPUT periodically
                        for(uint8_t k = 0;k<5;k++){
                                if (k==2){
                                        T_SW_in[k] = PWM_LOW;
                                }
                                else{
                                        T_SW_in[k] = PWM_NEUTRAL;
                                }
                        }
                        IDLE_SW_ACTIVE = true;
                        UPDATE_OUTPUT = true;
                }
                else{;}
        }
}

/*
 * STATE Machine to switch between remote and sw control - memory saving state machine (1byte global)
 */
inline void update_control_mode(){ 
        uint16_t T_C = millis() - ti_ctrl;
        bool condition[4] = {0, 0, 0};
        condition[0] = ( (NO_STATE(CTRL_REG)) && (IS_L1(CTRL_REG) || NOT_L1(CTRL_REG)) );
        condition[1] = ( S0(CTRL_REG) && (L1_IS_NOT_L2(CTRL_REG)) && (IS_L2(CTRL_REG) || NOT_L2(CTRL_REG)) );
        condition[2] = ( S1(CTRL_REG) && (L1_IS_L2(CTRL_REG)) && (IS_L2(CTRL_REG) || NOT_L2(CTRL_REG)) );
        
        if (READ_OK(CTRL_REG)){
                //CTRL_REG=RESET_READ_OK(CTRL_REG);
                
                if ( condition[0] ) { // SET STATE 0 - init
                        CTRL_REG=SET_S0(CTRL_REG);
                        CTRL_REG=CLEAR_L2_VALUES(CTRL_REG);
                        ti_ctrl = millis(); //USE FOR IDLE
                }
                else if ((T_C > T_ctrl)&&(!NO_STATE(CTRL_REG))) { // timer expired
                        
                        CTRL_REG=CLEAR_STATES(CTRL_REG);
                        CTRL_REG=CLEAR_VALUES(CTRL_REG);
                        CTRL_REG=RESET_ALL_VALUES(CTRL_REG);
                } 
                else if (condition[1]){ // SET STATE 1
                        ti_ctrl = millis(); // USE FOR RESETTING TIMER TO AVOID IDLE CONDITION BECOMING TRUE WHILE TRIGGERING
                        CTRL_REG=CLEAR_STATES(CTRL_REG);
                        CTRL_REG=CLEAR_L2_VALUES(CTRL_REG);
                        CTRL_REG=SET_S1(CTRL_REG);
                }
                else if (condition[2]){ // SET STATE 2
                        CTRL_REG=CLEAR_STATES(CTRL_REG);
                        CTRL_REG=SET_S2(CTRL_REG);
                }
                else if (S2(CTRL_REG) ){ // STATE 3 - back to first configuration
                        CTRL_REG=CLEAR_STATES(CTRL_REG);
                        CTRL_REG=CLEAR_VALUES(CTRL_REG);
                        CTRL_REG=RESET_ALL_VALUES(CTRL_REG);
                        
                        if (CONTROL_REMOTE){
                                CONTROL_SW = true;
                                CONTROL_REMOTE = false;
                        }
                        else if (CONTROL_SW){
                                CONTROL_SW = false;
                                for (int k = 0; k < 5; k++) {
                                        T_REMOTE_in[k] = PWM_NEUTRAL;
                                } 
                                CONTROL_REMOTE = true;              
                        }
                        else {;} 
                }
        }
        else {;}
        
        // Special case - emergency interrupt (braking or accelerating above specified margins)
        if (CONTROL_SW){
                if (((T_REMOTE_in[1] < PWM_EMERGENCY_CTRL_SWITCH_LOW))||(PWM_EMERGENCY_CTRL_SWITCH_HIGH < T_REMOTE_in[1])){
                        CONTROL_SW = false;
                        for (int k = 0; k < 5; k++) {
                                T_REMOTE_in[k] = PWM_NEUTRAL;
                        } 
                        CONTROL_REMOTE = true;  
                }
                else{;}
        }
        else{;}
   
}

/*
  Send actuation PPM
    considers idle and initialisation of remote input
    for software only the idle condition
*/
#if ACTUATE_EXTERNAL
inline void set_pwm_driver(uint8_t k, uint16_t ms){
        uint16_t off_tick = 0;
        uint8_t index = 0;
        //convert with separate function
        off_tick = uint16_t(PWM_EXTERNAL_MIN_TICK + ((ms-PWM_LOW)/(PWM_HIGH-PWM_LOW))*(PWM_EXTERNAL_MAX_TICK-PWM_EXTERNAL_MIN_TICK));
        
        off_tick-=27;
        if (k==0){
                //Serial.print("External PWM "); Serial.print(k); Serial.print(" | tick"); Serial.println(off_tick);
        }
        else{
        }
        index = k*2;
        external_pwm.setPWM(index, 0 , off_tick);
}
#endif

/*
 * 
 * Set ctrl-code for status bits for debuggirng via ROS network 
 * 
 */
inline uint8_t set_code(){
        uint8_t temp_code;
        // bit 1 : control state => 0 = remote | 1 = sw
        if(CONTROL_SW){
                temp_code |= 0x02;
        }
        else if(CONTROL_REMOTE){
                temp_code &= (~0x02);
        }
        else{
              temp_code |= 0xFF;  
        }
        // bit 0 : control state => 0 = active | 1 = idle ( based on current ctrl config )
        if(CONTROL_SW){
                temp_code &= (~0x01);
                if(IDLE_SW_ACTIVE){ //idle sw
                        temp_code |= 0x01;
                } else{;}
        }
        else if (CONTROL_REMOTE){
                temp_code &= (~0x01);
                for (uint8_t temp_code_idx=0;temp_code_idx<5;temp_code_idx++){
                        if (IDLE_ACTIVE[temp_code_idx]) { // idle remote
                                temp_code |=0x01;
                        } else{;}
                }
        }
        else{;}
        
        return temp_code;
        
}


/*
 *  Moving Average Filter to remove high frequency jitter from input
 *  Define Horizon with #define macros at top (1 input = 1/50Hz time window)
 * 
 */
uint16_t mv_avg_filter(uint8_t output, uint16_t new_variable) {
        // TODO: Test with static variables whether it works as expected
static uint16_t temp_lp[2][5]={{PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}, {PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}};
//volatile uint16_t temp_lp[2][10]={{PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}, {PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL, PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL,PWM_NEUTRAL}};
static uint32_t sum[2] = {PWM_NEUTRAL, PWM_NEUTRAL}; 

        static uint8_t current_idx[2];
        static uint8_t last_idx[2];
        static bool LP_INITIALIZED[2]={false,false};
        uint16_t return_value = 0;
#if DIAGNOSIS
#if PRINT_FILTER
      Serial.print("OUTPUT ");Serial.print(output);
      Serial.print(" | new_var ");Serial.print(new_variable);
      Serial.print("| Current idx");Serial.print(current_idx[output]);
      Serial.print("| Last idx");Serial.print(last_idx[output]);
      Serial.print("| Current sum");Serial.println (float(sum[output]));
#endif /* PRINT_FILTER */
#endif /* DIAGNOSIS */
        temp_lp[output][current_idx[output]] = new_variable;
        if (LP_INITIALIZED[output] ){
                //Serial.println("INITIALIZED");
                sum[output] -= temp_lp[output][last_idx[output]];
                sum[output] += temp_lp[output][current_idx[output]];
                return_value = uint16_t(sum[output]/float(MEDIAN_HORIZON));
                
                if(last_idx[output] >= MEDIAN_HORIZON_IDX){
                        last_idx[output]=0;
                }
                else{
                        last_idx[output]++;
                }
                
        }
        else{ 
                //Serial.println("NOT INITIALIZED");
                temp_lp[output][current_idx[output]] = new_variable;
                if (current_idx[output]>=MEDIAN_HORIZON_IDX){
                        for (uint8_t idx=0;idx<=MEDIAN_HORIZON_IDX;idx++){
                                sum[output] += temp_lp[output][idx];
                        }
                        return_value = uint16_t(sum[output]/float(MEDIAN_HORIZON));
                        float test = 0;
                        test = sum[output]/float(MEDIAN_HORIZON);
                        last_idx[output]++;
                        LP_INITIALIZED[output] = true;
                        
                #if DIAGNOSIS
                #if PRINT_FILTER
                        Serial.print("test output-float: ");Serial.println(test);
                        Serial.println("new INITIALIZED");
                #endif /* PRINT_FILTER */
                #endif /* DIAGNOSIS */
                }
                else{;}
                
        }
        
        // UPDATE current index
        if (current_idx[output]>=MEDIAN_HORIZON_IDX){
                current_idx[output] = 0;
        }
        else{
                current_idx[output]++;
        }

        // output
        if (LP_INITIALIZED[output] ){
                #if DIAGNOSIS
                #if PRINT_FILTER
                Serial.print("output: ");Serial.print(output);Serial.print(" | value : "); Serial.println(return_value);
                #endif /* PRINT_FILTER */
                #endif /* DIAGNOSIS */
                return return_value;
        }
        else{
                #if DIAGNOSIS
                #if PRINT_FILTER
                Serial.print("output: ");Serial.print(output);Serial.print(" | value : "); Serial.println(new_variable);
                #endif /* PRINT_FILTER */
                #endif /* DIAGNOSIS */
                return new_variable;
        }
        
}


#define T_TRANS_SAFETY 500
#define PWM_TRANSMISSION_THRESHOLD 250 // 1250-1750 is schmidt trigger hysteresis region  
#define PWM_TRANSMISSION_VEL_THRESHOLD 100 // definition for velocity region around PWM_NEUTRAL that allows switching gears
/*
 * 
 * Transform ppm in ms to gear logic state (gear 1 and gear 2 with bool - 0 and 1)
 * 
 */
inline bool ppm_to_gear(const uint16_t &T, bool&GEAR){
        if((PWM_LOW<=T)&&(T<=(PWM_NEUTRAL-PWM_TRANSMISSION_THRESHOLD))) {         // LOW (1000ms) - gear 1 
                GEAR = false;
                return true;
        }
        else if(((PWM_NEUTRAL+PWM_TRANSMISSION_THRESHOLD)<=T)&&(T<=PWM_HIGH)){   // HIGH (2000ms) - gear 2
                GEAR = true;
                return true;
        }
        else {
                return false;
        }
}
/*
 * 
 * Transform gear to ppm signal in ms
 * 
 */
inline uint16_t gear_to_ppm(const bool &GEAR){
        if(!GEAR) {                                     // LOW (1000ms) - gear 1 
                return uint16_t(PWM_LOW);
        }
        else {                                          // HIGH (2000ms) - gear 2
                return uint16_t(PWM_HIGH);
        }
}

/*
 * 
 * Transmission Failsafe Logic - expects a neutral / non-accelerating mode for at least T_TRANS_SAFETY before switching
 * 
 */
inline uint16_t set_transmission(const uint16_t &T_request){
        static bool GEAR_CURRENT = false;
        static bool GEAR_REQUEST = false;
        static bool TRIGGER_SAFETY_CONDITION=false;
        static long ti_trans_safety = 0;
        if (ppm_to_gear(T_request, GEAR_REQUEST)){ // if gear has a valid read and is clearly defined with a margin T_
                if( (ACTUATE[1]>(PWM_NEUTRAL-PWM_TRANSMISSION_VEL_THRESHOLD))&&(ACTUATE[1]<(PWM_NEUTRAL-PWM_TRANSMISSION_VEL_THRESHOLD))) {
                        if((millis()-ti_trans_safety)>T_TRANS_SAFETY){
                               TRIGGER_SAFETY_CONDITION = true; 
                               return gear_to_ppm(GEAR_REQUEST);
                               GEAR_CURRENT = GEAR_REQUEST;
                        }
                        else{;}
                }
                else {
                        ti_trans_safety = millis();
                        TRIGGER_SAFETY_CONDITION = false;
                        return gear_to_ppm(GEAR_CURRENT);
                }  
        }
        else{
                return uint16_t(PWM_NEUTRAL);
        }
        
 }


/*
  Send actuation PPM
    considers idle and initialisation of remote input
    for software only the idle condition

*/
inline void actuate() {
        //bool actuate_condition;
        //actuate_condition = UPDATE_OUTPUT || (( (IDLE_ACTIVE[0] || IDLE_ACTIVE[1] || IDLE_ACTIVE[2] || IDLE_ACTIVE[3] || IDLE_ACTIVE[4]) && CONTROL_REMOTE) || (CONTROL_SW && IDLE_SW_ACTIVE));
  //DEFINE OUTPUT SIGNAL BASED ON CONTROL MODE
  if (UPDATE_OUTPUT){
        UPDATE_OUTPUT = false; //only update if input has changed - see update_period function
        if (CONTROL_REMOTE) {
                //ACTIVATE ALL INTERRUPTS
                for (uint8_t k = 0; k < 5; k++) {
                        if (k==2) { 
                                ACTUATE[k] = PWM_LOW; // FIX TRANSMISSION TO LOWER GEAR 
                                //TODO: REMOVE OR INCLUDE FAILSAFE LOGIC; ONLY SWITCH GEAR WITHOUT VELOCITY
                                // REMOVE AND REPLACE with uint16_t temp transmission; set_pwm_driver(k, set_transmission(ACTUATE[k]);
                        }
                        else if (!IDLE_ACTIVE[k]){
                                ACTUATE[k] = T_REMOTE_in[k];
                        }
                        else {
                                ACTUATE[k] = PWM_NEUTRAL;
                        }
                }
        }
        else if (CONTROL_SW) {
                for (uint8_t k = 0; k < 5; k++) {
                        if (k==2) { 
                                ACTUATE[k] = PWM_LOW;
                                //TODO: REMOVE OR INCLUDE FAILSAFE LOGIC; ONLY SWITCH GEAR WITHOUT VELOCITY
                                // REMOVE AND REPLACE with uint16_t temp transmission; set_pwm_driver(k, set_transmission(ACTUATE[k]);
                        }
                        else if (!IDLE_SW_ACTIVE){
                                ACTUATE[k] = T_SW_in[k];
                        }
                        else {
                                ACTUATE[k] = PWM_NEUTRAL;
                        } 
                }
        }
        else {;}
        
#if ACTUATE_INTERNAL
        if ((millis()-ti_out)>T_out){ 
                ti_out = millis();    //commented out for debug   
                //ACTUATE OUTPUT SIGNAL TO output channels
                for (uint8_t k = 0 ; k < 5; k++) {
                        if(k==2){  set_pwm_driver(k, PWM_LOW);} else{ 
                                if (CONTROL_REMOTE){ // only use lowpass for remote control
                                        if (k<=1){
                                                uint16_t temp_output;
                                                temp_output = mv_avg_filter(k,ACTUATE[k];
                                                output[k].writeMicroseconds(temp_output);
                                        }
                                        else{
                                                output[k].writeMicroseconds(ACTUATE[k]);
                                        }
                                }
                                else{
                                       output[k].writeMicroseconds(ACTUATE[k]); 
                                }
                        }
                }
        }
        else{;}
#endif /* ACTUATE_INTERNAL */

#if ACTUATE_EXTERNAL
        if ((millis()-ti_out)>T_out){ 
                ti_out = millis();    //commented out for debug   
                for (uint8_t k = 0 ; k < 5; k++) {
                      if(k==2){  set_pwm_driver(k, PWM_LOW);} else{//TODO: REMOVE OR INCLUDE FAILSAFE LOGIC; ONLY SWITCH GEAR WITHOUT VELOCITY
                        // REMOVE AND REPLACE with uint16_t temp transmission; set_pwm_driver(k, set_transmission(ACTUATE[k]);
                                if (CONTROL_REMOTE){ // only use lowpass for remote control
                                        if (k<=1){
                                                uint16_t temp_output;
                                                temp_output = mv_avg_filter(k,ACTUATE[k]);
                                                set_pwm_driver(k, temp_output);
                                               
                                                //set_pwm_driver(k, ACTUATE[k]);
                                        }
                                        else{
                                                set_pwm_driver(k, ACTUATE[k]);
                                        }
                                }
                                else {
                                        set_pwm_driver(k, ACTUATE[k]); 
                                }
                      }
                        
                }
        }
        else{;}
#endif /* ACTUATE_EXTERNAL */

#if ROS_ACTIVE
        // update ROS publishing message
        // TODO: REMOVE ALL SPECIAL CASES AND ONLY TRANSLATE ACTUATE TO PWM
        if (CONTROL_REMOTE) {
                MSG_ACTUATED.steering = PPM_REMOTE_in[0];
                MSG_ACTUATED.velocity = PPM_REMOTE_in[1];
                MSG_ACTUATED.transmission = -100;//-fix to lower gearPPM_REMOTE_in[2];
                MSG_ACTUATED.differential_front = PPM_REMOTE_in[3];
                MSG_ACTUATED.differential_rear = PPM_REMOTE_in[4];
        }
        else if (CONTROL_SW) {
                if (IDLE_SW_ACTIVE) {
                        MSG_ACTUATED.steering = 0;
                        MSG_ACTUATED.velocity = 0;
                        MSG_ACTUATED.transmission = -100;// fix to lower gear
                        MSG_ACTUATED.differential_front = 0;
                        MSG_ACTUATED.differential_rear = 0;
                }
                else{ // ONLY USE THIS CASE (ACTUATE SHOULD REPRESENT THE IDLE CONDITION
                        MSG_ACTUATED.steering = period_to_pwm(ACTUATE[0]); //PPM_SW_in[0]; //period_to_pwm(T_SW_in[0]);
                        MSG_ACTUATED.velocity = period_to_pwm(ACTUATE[1]);//PPM_SW_in[1];
                        MSG_ACTUATED.transmission = period_to_pwm(ACTUATE[2]);//0;//FIX TO lower gear //PPM_SW_in[2];
                        MSG_ACTUATED.differential_front = period_to_pwm(ACTUATE[3]);//PPM_SW_in[3];
                        MSG_ACTUATED.differential_rear = period_to_pwm(ACTUATE[4]);//PPM_SW_in[4];
                }
        }
        else {;}
        MSG_ACTUATED.ctrl_code = set_code();
        MSG_ACTUATED.voltage = int8_t(voltage);
        MSG_ACTUATED.current = int8_t(current);
#endif /* ROS_ACTIVE */

  }
  else{;}
}


/*
  ROS publishing data
*/
inline void ROS_publish_data() {
#if ROS_ACTIVE
  //noInterrupts();
  if ((millis()-ti_pub)>T_pub){
    ti_pub=millis();
    ctrl_actuated.publish(&MSG_ACTUATED);
  }
  else{;}
  //interrupts();
#endif /* ROS_ACTIVE */
}

/*
 *  Read ADC (NOT USED ATM)
 * 
 */

inline uint16_t read_adc()
{
    ADCSRA |= (1<<ADSC);

    while(!(ADCSRA & (1<<ADIF)));

    uint8_t adcl = ADCL;
    uint8_t adch = ADCH;

    ADCSRA |= (1<<ADIF);

    return (adch<<8) | adcl;
}

/*
   Updating Measurements (inactive: TODO)

   Allows reading two analog channels without the usual delay algorithms between switching the multiplexer
*/
inline void update_measurements() {
        static bool state = false;
        uint16_t adc=0;
        if ((millis()-ti_sense)>T_sense){
                ti_sense = millis();
                if (state) {
                        state = false;
                        voltage = analogRead(A0);
                        //voltage = ADC_GAIN*ADC_REF*voltage/ADC_BIT_RES;
                        adc = analogRead(A0); // switch to next analog read to allow voltage at adc multiplexer to settle
                }
                else {
                        state = true;
                        //current = analogRead(A1);
                        current = ((5.0*current/ADC_BIT_RES)-2.5)/20.0; // voltage in V
                        current = current/0.05; // current in A
                        //adc = analogRead(A0); // switch to next analog read to allow voltage at adc multiplexer to settle
                }
        }
        else{;}
        // if voltage is smaller than threshold
        if (1) {
               if ((millis()-ti_alarm)>T_alarm){
                 ti_alarm = millis();
                 // toggle alarm on off here
               }
               else{;}
        }
        else{
                ; // Alarm off
        }
}

/*
   Adapting LED signaling state
*/
inline void blink_led() {
  static boolean STATUS = 0;
  if(CONTROL_REMOTE){
        IDX_LED_MODE = 1;
  }
  else if(CONTROL_SW){
        IDX_LED_MODE = 2;
  }
  else {;}
  if (( millis() - ti_led) > T_led[IDX_LED_MODE-1] ) {
    STATUS = !STATUS;
    digitalWrite(LED_BUILTIN, STATUS);
    ti_led = millis();
  }
  else {;}
}


#if ALARM_ACTIVE
/*
 * low-voltage watchdog 
 * 
 * if voltage drops to a certain level it triggers a certain ALARM level that activates the piezo buzzer and can be deactivated with the remote control 
 * 
 */
 inline void low_voltage_watchdog(){
        ;
 }
#endif /* ALARM_ACTIVE */
/*
   The main logic loop algorithm
*/
void loop()
{
  update_measurements();
  update_input_pwm_periods();
  actuate();
  ROS_publish_data();
  update_control_mode();
  print_diagnosis();
  blink_led();
  // low_voltage_watchdog();
  
#if ROS_ACTIVE
  //noInterrupts();
  nh.spinOnce();
  //interrupts();
#endif /* ROS_ACTIVE */
}

/*
   Configuration of Registers and User Variables
*/
void setup()
{
        //cli();
        
#if DIAGNOSIS
        Serial.begin(BAUD_RATE);
#endif /* DIAGNOSIS */

        // Configure Interrupt Pin change inputs
        // reset all configuration registers
        DDRB &= 0x00;
        DDRC &= 0x00;
        DDRD &= 0x00;
        PORTB &= 0x00;
        PORTC &= 0x00;
        PORTD &= 0x00;
        
        // set directions
        //define inputs
        PIN_MASK[0] = (0x01); // PINS on PortB FOR PCINT
        PIN_MASK[1] = (0x08); // PINS on PortC FOR PCINT
        PIN_MASK[2] = (0x04) | (0x10) | (0x80); // PINS on PortD FOR PCINT

        // timer specific interrupt pin (check for interrupt flag) - set at definition
        //PIN_INT_MASK[5] = {BIT3, BIT0, BIT7, BIT4, BIT2}; 
        
        // DDRB: output - PB1,PB2, PB3, PB6, (OC1A, OC1B,OC2A,TOSC1)
        //DDRB |= 0x02; DDRB |= 0x04; DDRB |= 0x08; DDRB |= 0x40; 
        DDRB &= (~PIN_MASK[0]);
        //DDRC &= 0x00; 
        DDRC &= (~PIN_MASK[1]);
        //DDRD &= 0x00; 
        DDRD &= (~PIN_MASK[2]);
        
        // configure pull-ups
        PORTB |= PIN_MASK[0];
        PORTC |= PIN_MASK[1];
        PORTD |= PIN_MASK[2];
        
        // read values - setting 1, changes input configuration in PORTxn
        // DON'T USE THIS ON INPUT PINS! -- CHANGES PULLUP CONFIGURATION - might causes floating pins
        //PINB |= 0x00;
        //PINC |= 0x00;
        //PIND |= 0x00;

        
        // configure all three interrupts
        PCICR |= 0x07;
        PCIFR = 0x00;
        PCMSK0 = 0x00; // PCINT[7:0]
        PCMSK1 = 0x00; // PCINT[15:8]
        PCMSK2 = 0x00; // PCINT[23:16]
        PCMSK0 |= 0x01; // PCINT[7:0]
        PCMSK1 |= 0x08; // PCINT[15:8]
        PCMSK2 |= 0x04; PCMSK2 |= 0x10; PCMSK2 |= 0x80; // PCINT[23:16] // activate PCINT16
        
        //Initialise Variables with current Pin Input ignore rest
        PIN_V_CURR[0] = PINB & (~PIN_MASK[0]);
        PIN_V_PREV[0] = PINB & (~PIN_MASK[0]);
        PIN_V_COMP[0] = 0x00;
        PIN_V_CURR[1] = PINC & (~PIN_MASK[1]);
        PIN_V_PREV[1] = PINC & (~PIN_MASK[1]);
        PIN_V_COMP[1] = 0x00;
        PIN_V_CURR[2] = PIND & (~PIN_MASK[2]);
        PIN_V_PREV[2] = PIND & (~PIN_MASK[2]);
        PIN_V_COMP[2] = 0x00;

        
        // Configure outputs
#if ACTUATE_INTERNAL
        output[0].attach(OUT_STEER);
        output[1].attach(OUT_VEL);
        output[2].attach(OUT_TRANS);
        output[3].attach(OUT_DIFF_F);
        output[4].attach(OUT_DIFF_R);
#endif /* ACTUATE_INTERNAL */

        // LED 13 
        pinMode(LED_BUILTIN,OUTPUT);


#if ACTUATE_EXTERNAL
        //Serial.println("Setting up I2C"); // use only with DIAGNOSIS 1
        external_pwm.begin();
        external_pwm.setPWMFreq(50);
        delay(200); // RANDOOM DELAY VALUE, do NOT TAMPER WITH IT!! ROSSERIAL ERRORS otherwise
        //Serial.println("I2C setup completed!");
#endif
        
#if ROS_ACTIVE
        // ROS setup
        nh.getHardware()->setBaud(BAUD_RATE);
        nh.initNode();
        nh.advertise(ctrl_actuated);
        nh.subscribe(ctrl_request);
#endif /* ROS_ACTIVE */

#if MEASUREMENT
        // configure analog pins as input
        DDRC &= (~0x03);
        //PORTC |= 0x03;
        // ADC - CONFIG - Voltage Sense
        ADMUX &= 0x00;
        ADMUX |= 0x40;
        // START CONVERSION WITH ADSC write 1, will be reset to 0 after conversion
        ADCSRA = 0x00; // reset register
        ADCSRA |= 0x80; // enable ADC with ADEN

        #if DIAGNOSIS
        //final ADC settings
        Serial.println("### ADC ");
        Serial.print("ADMUX: "); print_binary(ADMUX); // SHOWS WHAT CLOCK SOURCE TIMER USES
        Serial.print("ADCSRA: "); print_binary(ADCSRA);
        #endif /* DIAGNOSIS */
#endif /* MEASUREMENT */

#if ALARM_ACTIVE
        //pin-12 config
        PORTB = 0x00;
        DDRB = 0X00;

        DDRB |= 0x10; // set PB4 as an output
        PORTB |= 0x10; // set PB4 - D12 as high

        //PD3 -OC2B CONFIGURATION
        DDRD = 0x00;
        PORTD = 0x00;
        
        DDRD |= 0x08;
        PORTD |= 0x08;
        
        // TIMER - CONFIG
        // set mode (com) - set mode2 (wgm) - set prescaler - set compare
        TCCR2A = 0x10;
        // example for reset TCCR2A &= (~0x12);
        //TCCR2B &= 0x00; // set prescaler to 32 - 500kHz
        TCCR2B = 0x03; // set prescaler to 32 - 500kHz
        OCR2B = ALARM_TICK_COUNT;
        
        TIMSK2 = 0x04;
        #if DIAGNOSIS
        // FINAL timer settings
        Serial.println("### TIMER: ");
        Serial.print("ASSR: "); print_binary(ASSR); // SHOWS WHAT CLOCK SOURCE TIMER USES
        Serial.print("TCCR2A: "); print_binary(TCCR2A);
        Serial.print("TCCR2B: "); print_binary(TCCR2B);
        Serial.print("TIMSK: "); print_binary(TIMSK2);
        Serial.print("OCR2B: "); print_binary(OCR2B); 
        #endif /* DIAGNOSIS */

#endif /* ALARM_ACTIVE */

        //sei();
}

/*
   Print diagnosis (uncomment before Flashing)
*/
void print_diagnosis() {
#if DIAGNOSIS
    if ((millis() - ti_diag) > T_diag) {
      ti_diag = millis();
      clear_screen();
      
#if PRINT_BITS
        //print_binary(SREG);
        //print_binary(PCICR);
        //print_binary(PCIFR);
        Serial.println("REGISTER MCUCR");
        print_binary(MCUCR);
        Serial.println("pin mask");
        print_binary(PIN_MASK[0]);
        print_binary(PIN_MASK[1]);
        print_binary(PIN_MASK[2]);
        Serial.println("interrupt mask");
        print_binary(PCMSK0);
        print_binary(PCMSK1);
        print_binary(PCMSK2);
        Serial.println("data port");
        print_binary(DDRB);
        print_binary(DDRC);
        print_binary(DDRD);
        Serial.println("pullup");
        print_binary(PORTB);
        print_binary(PORTC);
        print_binary(PORTD);
        Serial.println("port values");
        print_binary(PINB);
        print_binary(PINC);
        print_binary(PIND);
#endif /* PRINT_BITS */

#if PRINT_CTRL_MODE
      Serial.print("CTRL_REMOTE: ");Serial.print(CONTROL_REMOTE);Serial.print(" CTRL_SW: ");Serial.println(CONTROL_SW);
      Serial.println("");
#endif /* PRINT_CTRL_MODE */
      
#if PRINT_CTRL
      Serial.print("NO S: ");Serial.print(NO_STATE(CTRL_REG));Serial.print(" | S0 : ");Serial.print(S0(CTRL_REG));Serial.print(" | S1 : ");Serial.print(S1(CTRL_REG));Serial.print(" | S2 : ");Serial.println(S2(CTRL_REG));
      Serial.println("");
      
      Serial.print("# | L1= ");Serial.print(IS_L1(CTRL_REG));
      Serial.print("# | NOT_L1= ");Serial.print(NOT_L1(CTRL_REG));
      Serial.print("# | L2= ");Serial.print(IS_L2(CTRL_REG));
      Serial.print("# | NOT_L2= ");Serial.print(NOT_L2(CTRL_REG));
      
      Serial.println("");
      Serial.print("# | L1_IS_L2= ");Serial.print(L1_IS_L2(CTRL_REG));
      Serial.print("# | L1_IS_NOT_L2= ");Serial.print(L1_IS_NOT_L2(CTRL_REG));
      
      Serial.println("");
      Serial.print("# T_ctrl="); Serial.print(millis() - ti_ctrl);
      Serial.println("");
      print_binary(CTRL_REG);
      Serial.println("");
      Serial.print("READ_OK= ");Serial.print(READ_OK(CTRL_REG));
#endif /* PRINT_CTRL */
      
#if PRINT_INPUT_PERIODS
      Serial.println("");
      for (int k = 0; k < 5; k++) {
        Serial.print("TI= " ); Serial.print(k);
        Serial.print(" NEW_T= "); Serial.print(PWM_REMOTE_INPUT_UPDATED[k]);
        Serial.print(": T= "); Serial.print(T_REMOTE_in[k]);;
        Serial.print(": IDLE-ACTIVE= " ); Serial.println(IDLE_ACTIVE[k]);
      }
#endif /* PRINT_INPUT_PERIODS */


#if PRINT_INPUT_PPMS
      Serial.println("");
      for (int k = 0; k < 5; k++) { 
        Serial.print(" --- PPM");Serial.print(k);Serial.print(": "); Serial.println(PPM_REMOTE_in[k]);
        Serial.print(" --- period to pwm ");Serial.print(k);Serial.print(" : pwm:"); Serial.println(period_to_pwm(T_REMOTE_in[k]));
        Serial.print(" --- pwm to period ");Serial.print(k);Serial.print(":  period:"); Serial.println(pwm_to_period(PPM_REMOTE_in[k]));
      }
#endif /*PRINT_INPUT_PPMS */


#if PRINT_OUTPUT
      Serial.println(""); Serial.println("Output-period:");
      for (int k = 0; k < 5; k++) { 
        Serial.print(" OUTPUT");Serial.print(k);Serial.print(": "); Serial.print(ACTUATE[k]);
      }
#endif /* PRINT_OUTPUT */

#if PRINT_MEASUREMENT
        Serial.print("Voltage: "); Serial.println(voltage);
        Serial.println("");
        
        Serial.print("Current: "); Serial.println(current);
#endif /* PRINT_MEASUREMENT */
      
    }
    else {
      ;
    }
#endif /* DIAGNOSIS */
}

