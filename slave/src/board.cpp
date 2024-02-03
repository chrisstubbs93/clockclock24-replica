#include "board.h"
#include <string>

// Define a stepper and the pins it will use
ClockAccelStepper _motors[6] = {
  ClockAccelStepper(ClockAccelStepper::DRIVER, F_STEP, F_DIR), // 0 -> h clock 0
  ClockAccelStepper(ClockAccelStepper::DRIVER, E_STEP, E_DIR), // 1 -> m clock 0
  ClockAccelStepper(ClockAccelStepper::DRIVER, D_STEP, D_DIR), // 2 -> h clock 1
  ClockAccelStepper(ClockAccelStepper::DRIVER, C_STEP, C_DIR), // 3 -> m clock 1
  ClockAccelStepper(ClockAccelStepper::DRIVER, B_STEP, B_DIR), // 4 -> h clock 2
  ClockAccelStepper(ClockAccelStepper::DRIVER, A_STEP, A_DIR)  // 5 -> m clock 2
};

uint8_t _i2c_address = 0;

static int sanitize_angle(int angle)
{
  angle = angle % 360;
  return angle < 0 ? 360 + angle : angle;
}

void blinken(uint8_t n){
for (uint8_t i = 0; i < n; i++){
      digitalWrite(LED_BUILTIN, HIGH);
      delay(25);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
}

void board_begin()
{

  // hall sensor stuff
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(25);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);      
  digitalWrite(LED_BUILTIN, HIGH);
  delay(25);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);


  for(uint8_t pin : HallPins){
    pinMode(pin, INPUT_PULLUP);
  }

  // Reset motor controllers
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);

  // Init motors
  for(int i = 0; i < 6; i++)
  {
    if(i % 2 == 1)
      _motors[i].setReverse(true);
    _motors[i].setMaxMotorSteps(STEPS);
    _motors[i].setHandAngle(INIT_HANDS_ANGLE);
    _motors[i].setMinPulseWidth(0);
  }

  pinMode(ADDR_1, INPUT_PULLUP);
  pinMode(ADDR_2, INPUT_PULLUP);
  pinMode(ADDR_3, INPUT_PULLUP);
  pinMode(ADDR_4, INPUT_PULLUP);
  _i2c_address = !digitalRead(ADDR_1) + 
               (!digitalRead(ADDR_2) << 1) + 
               (!digitalRead(ADDR_3) << 2) + 
               (!digitalRead(ADDR_4) << 3);
}

void board_loop()
{
  for(int i = 0; i < 6; i++)
    _motors[i].run();
}

void board_loop_setup()
{
  for(int i = 0; i < 6; i++)
  {
    if(_motors[i].getClockwiseBool()) _motors[i].run();
    if(_motors[0].getZeroedBool() && _motors[1].getZeroedBool() && _motors[2].getZeroedBool() && _motors[3].getZeroedBool() && _motors[4].getZeroedBool() && _motors[5].getZeroedBool())
    {
      Serial.println("all motors zeroed");
      _motors[i].moveToZero(_motors[i].getZeroOffset());
    } 
  }
}

void motor_identification()
{
  while(1 == 1){
  for(int i = 0; i < 6; i++)
  {
    Serial.println("Moving Motor");
    Serial.println(i);
    //_motors[i].run();
    _motors[i].move(100);
    _motors[i].runToPosition();
    delay(5000);
  }
  }

}

uint8_t get_i2c_address()
{
  return _i2c_address;
}

bool clock_is_running(int index)
{
  if( index < 0 || index > 2)
    return false;

  return _motors[index*2].distanceToGo() != 0 || 
         _motors[index*2 + 1].distanceToGo();
}

void set_clock(int index, t_clock state)
{
  int angle_h = sanitize_angle(state.angle_h + state.adjust_h);
  _motors[index*2].setMaxSpeed(state.speed_h);
  _motors[index*2].setAcceleration(state.accel_h);
  _motors[index*2].moveToAngle(angle_h, state.mode_h);

  int angle_m = sanitize_angle(state.angle_m + state.adjust_m);
  _motors[index*2 + 1].setMaxSpeed(state.speed_m);
  _motors[index*2 + 1].setAcceleration(state.accel_m);
  _motors[index*2 + 1].moveToAngle(angle_m, state.mode_m);
}

void adjust_h_hand(int index, signed char amount)
{
  int steps = amount * STEPS / 360;
  _motors[index*2 + 1].move(steps);
  _motors[index*2 + 1].runToPosition();
}

void adjust_m_hand(int index, signed char amount)
{
  int steps = amount * STEPS / 360;
  _motors[index*2].move(-steps);
  _motors[index*2].runToPosition();
}

void zero_hand_with_offset(int index, int offset)
{
  if(index == 0 || index == 2 || index == 4) { // bottom hand
    offset = offset * 2;
  }
  //_motors[index].zeroCurrentPositionWithOffset(offset);
  Serial.print("Hand ");
  Serial.print(index);
  Serial.println(" zeroed");
}

bool get_direction(int index)
{
  return _motors[index].getCurrentDirection();
}

void run_clockwise(int index)
{
  _motors[index].setClockwiseBool(true);
    if(index == 0 || index == 2 || index == 4) { // bottom hand
    _motors[index].runClockwiseUntilZero(5000);
  } else if(index == 1 || index == 3 || index == 5) { 
    _motors[index].setTopHandBool(true);
    _motors[index].runClockwiseUntilZero(5000);
  }
  // _motors[index].setMaxSpeed(100);
  // _motors[index].setAcceleration(15);
  // _motors[index].move(2000);
  // _motors[index].runToPosition();
  //_motors[index].runToPosition();
}


void set_clock_test(int index, t_clock state)
{
  int angle_h = sanitize_angle(state.angle_h + state.adjust_h);
  _motors[index].setMaxSpeed(state.speed_h);
  _motors[index].setAcceleration(state.accel_h);
  //_motors[index*2].moveToAngle(angle_h, state.mode_h);

  int angle_m = sanitize_angle(state.angle_m + state.adjust_m);
  _motors[index].setMaxSpeed(state.speed_m);
  _motors[index].setAcceleration(state.accel_m);
  //_motors[index*2 + 1].moveToAngle(angle_m, state.mode_m);
}

bool is_hall_start_set(int index)
{
  return _motors[index].getHallStartValue() != -10000 ? true : false;
}

bool is_hall_stop_set(int index)
{
  return _motors[index].getHallStopValue() != -10000 ? true : false;
}

void set_hall_start(int index)
{
  _motors[index].setHallStartValue();
  Serial.println("Hall Start set");
}

void set_hall_stop(int index)
{
  _motors[index].setHallStopValue();
  Serial.println("Hall stop set");
}

long get_hall_step_gap(int index)
{
  if(_motors[index].getHallStartValue() > _motors[index].getHallStopValue())
  {
    return _motors[index].getHallStartValue() - _motors[index].getHallStopValue();
  } else {
    return _motors[index].getHallStopValue() - _motors[index].getHallStartValue();
  }
}

long get_hall_start_value(int index)
{
  return _motors[index].getHallStartValue();
}

long get_hall_stop_value(int index)
{
  return _motors[index].getHallStopValue();
}

void finish_zero(int index)
{
  _motors[index].setZeroOffset(get_hall_step_gap(index) / 2);
  _motors[index].setNewZeroWithOffset(get_hall_step_gap(index) / 2);
}