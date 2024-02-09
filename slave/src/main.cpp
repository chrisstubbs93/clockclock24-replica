#include <Wire.h>

#include "board_config.h"
#include "board.h"
#include "clock_state.h"
#include "i2c.h"

bool HallStates[8];
bool readyToZero;

const t_clock default_clock = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// int spin_num; //The spin lock number
spin_lock_t *spin_lock[3]; //The spinlock object that will be associated with spin_num

t_half_digit target_clocks_state;
t_half_digit current_clocks_state;

// I2C runs on main core (core 0)
void receiveEvent(int how_many)
{
  // Serial.println("Received something");
  if (how_many >= sizeof(half_digit))
  {
    t_half_digit tmp_state;
    I2C_readAnything (tmp_state);

    for (uint8_t i = 0; i < 3; i++)
    {
      spin_lock_unsafe_blocking(spin_lock[i]); //Acquire the spin lock without disabling interrupts
      target_clocks_state.clocks[i] = tmp_state.clocks[i];
      target_clocks_state.change_counter[i] = tmp_state.change_counter[i];
      spin_unlock_unsafe(spin_lock[i]); //Release the spin lock without re-enabling interrupts
    }
  }
}

void setup()
{  
  Serial.begin(115200);
  
  Serial.println("clockclock24 replica by Vallasc slave v1.0");
  delay(3000); //wait for the shit serial monitor
  Serial.println("waiting");
delay(3000); //wait for the shit serial monitor
  Serial.println("starting");

  board_begin();
  target_clocks_state = {{default_clock, default_clock, default_clock}, {0, 0, 0}};

  for (uint8_t i = 0; i < 3; i++)
  {
    int spin_num = spin_lock_claim_unused(true); //Claim a free spin lock. If true the function will panic if none are available
    spin_lock[i] = spin_lock_init(spin_num); //Initialise a spin lock
  }

  Wire.begin(get_i2c_address());
  Wire.onReceive(receiveEvent);

  //before doing anything, move any hands which are currently over hall sensors.
  for (uint8_t i = 0; i < 6; i++)
  {
    if(!digitalRead(HallPins[i])) //currently over hall
    {
      Serial.print("Motor ");
      Serial.print(i);
      Serial.println(" started on sensor. moving.");
      jogHandOffSensor(i);  //jog it
      delay(10000); //wait for move otherwise death
      set_hand_angle(i,0); // pretend that never happened
      setCurrentPos(i,0); // pretend that never happened
    }
  }
  readyToZero = true;


  for (uint8_t i = 0; i < 6; i++)
  {
    setCurrentPos(i,360*12*2);//trick it so it can turn backwards to 0
    Serial.println("going cw");
    run_clockwise(i);
  }
    //motor_identification();

}

bool gone_ccw;

void loop()
{
  if(!gone_ccw){
    bool all_started = true;
    for (uint8_t i = 0; i < 6; i++)
    {
      if(!is_hall_start_set(i)) all_started = false;
    }
    if(all_started) //all start posns found on CW rotation, switch to CCW
    {
      //stop();
      for (uint8_t i = 0; i < 6; i++)
      {
        Serial.println("going ccw");
        run_counterclockwise(i);
        gone_ccw  =true;
      }
    }
  }

  uint8_t hn= 0;
  for(uint8_t pin : HallPins){
    bool t = !digitalRead(pin);

    if(!isZeroed(hn) && readyToZero){
      if(t){ //rising edge
        if(HallStates[hn] != t){
          if(get_direction(hn)) //clockwise
          {
            Serial.println("Clockwise rising edge");
            if(!is_hall_start_set(hn)) set_hall_start(hn);
            zero_hand_with_offset(hn, 0); //i think this does nothing?
          }
          else
          {
            Serial.println("CounterClockwise rising edge");
            if(is_hall_start_set(hn) && !is_hall_stop_set(hn)) set_hall_stop(hn);
            Serial.print("Hall gap for motor ");
            Serial.print(hn);
            Serial.print(": ");
            Serial.print(get_hall_step_gap(hn));
            Serial.print(" start: ");
            Serial.print(get_hall_start_value(hn));
            Serial.print(" stop: ");
            Serial.println(get_hall_stop_value(hn));
            finish_zero(hn);
          }
          //blinken(hn+1);
        }
      }
    }
    HallStates[hn] = t;
    hn = hn+1;
  }
  //delay(1);
}

void setup1() 
{
  current_clocks_state = {{default_clock, default_clock, default_clock}, {0, 0, 0}};
}

// Steppers on core 1
void loop1()
{
  if (isZeroed(0)&&isZeroed(1)&&isZeroed(2)&&isZeroed(3)&&isZeroed(4)&&isZeroed(5))
  { //all zeroed run normal clock code
    board_loop();

    for (uint8_t i = 0; i < 3; i++)
    {
      if(!clock_is_running(i) && current_clocks_state.change_counter[i] != target_clocks_state.change_counter[i])
      {
        //Serial.printf("Inside clock %d\n", i);
        spin_lock_unsafe_blocking(spin_lock[i]);
        current_clocks_state.clocks[i] = target_clocks_state.clocks[i];
        current_clocks_state.change_counter[i] = target_clocks_state.change_counter[i];
        spin_unlock_unsafe(spin_lock[i]);

        if(current_clocks_state.clocks[i].mode_h == ADJUST_HAND)
          adjust_h_hand(i, current_clocks_state.clocks[i].adjust_h);

        if(current_clocks_state.clocks[i].mode_m == ADJUST_HAND)
          adjust_m_hand(i, current_clocks_state.clocks[i].adjust_m);

        if(current_clocks_state.clocks[i].mode_h <= MAX_DISTANCE3)
          set_clock(i, current_clocks_state.clocks[i]);
      }
    }
  }
  else
  { // not all zeroed, run zero code
    board_loop_setup();
  }

}