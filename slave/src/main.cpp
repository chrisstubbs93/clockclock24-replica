#include <Wire.h>

#include "board_config.h"
#include "board.h"
#include "clock_state.h"
#include "i2c.h"

bool HallStates[8];

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

  board_begin();
  target_clocks_state = {{default_clock, default_clock, default_clock}, {0, 0, 0}};

  for (uint8_t i = 0; i < 3; i++)
  {
    int spin_num = spin_lock_claim_unused(true); //Claim a free spin lock. If true the function will panic if none are available
    spin_lock[i] = spin_lock_init(spin_num); //Initialise a spin lock
  }

  Wire.begin(get_i2c_address());
  Wire.onReceive(receiveEvent);
}



void loop()
{
uint8_t hn= 0;
  for(uint8_t pin : HallPins){
    bool t = !digitalRead(pin);
    if(HallStates[hn] != t){
      if(t){ //rising edge
        blinken(hn+1);
        zero_hand(hn);
      }
      HallStates[hn] = t;
    }
    hn = hn+1;
  }
  delay(10);
}

void setup1() 
{
  current_clocks_state = {{default_clock, default_clock, default_clock}, {0, 0, 0}};
}

// Steppers on core 1
void loop1()
{
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