#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <Servo.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET 	-1 // This display does not have a reset pin accessible

#define TAPE_SENSOR_R PA3
#define TAPE_SENSOR_L PA2
#define p_pot PA4
#define d_pot PA5

#define MOTOR_L_F PB_6 
#define MOTOR_L_R PB_7 
#define MOTOR_R_F PB_8
#define MOTOR_R_R PB_9

#define SERVO PA7 //whacker
#define SERVO1 PB0 //elevator
#define SERVO2 PB1 //rear dump

#define ECHO PA0
#define TRIG PA1

#define PAUSE_DUMP PA12
#define CSC_THRESH PA11

#define PWMfreq 2000
#define max_input 1023
#define midpoint 512

#define ON_OFF_THRESHOLD 180 //280 //can investigate this number more... above this number is when it is on tape
#define ON_ELEVATED_TAPE 45 //878 does not move backwards, 875 accidentally moves back
#define STARTING_DUMMY_VALUE 10000

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

Servo servo;
Servo servo1;
Servo servo2;

//PID
int previous_error = STARTING_DUMMY_VALUE; //dummy starting value
int dist_between_sensors = 15; //may have to do more precise measurements on this...
//need to have it be far enough away for this error to have meaning but also for there to be big enough range in reflectance values...
double gain_scaling_factor = 1.0; //scales down the gain input from the potentiometers.
int current_state_count = 1; //beginning of counting
int previous_state_count = STARTING_DUMMY_VALUE; //dummy starting value
int previous_state = STARTING_DUMMY_VALUE; //dummy starting value;
double correction_scaling_factor = 1.25; //play around with this value to get g value to fit within duty range.
int max_motor_duty = 65535; //max number the duty can be in motor format
int min_motor_duty = 1; //min number used for linearization?
int nominal_motor_L_duty = 30500; // this value should be larger than R
int nominal_motor_R_duty = 29500;
int nominal_backwards_L_duty = -30000; //L is in fact R
int nominal_backwards_R_duty = -33000; //-45000, this is 48000
int num_loops = 0;
double slope_scaling_factor = 100.0;
int backing_up = 0;
int straight_factor = 500;

//Sonar
double distance = STARTING_DUMMY_VALUE;
int distance_detect = 10;

//interrupt
void intrpt_PAUSE_DUMP();
volatile int loopcount =0;
volatile int pause_duration = 6000; //6 seconds
volatile int drop_hatch = 0;
volatile int rear_hatch = 0;

//interrupt
void intrpt_CSC_THRESH();
volatile int current_state_thresh = 50; //if in the state more than x iterations, will assume on a straight line (no backspin)

//servos
int position_current;

int position_initial_R = 0;
int position_final_R = 80;
int position_initial_W = 0;
int position_final_W = 176;
int position_initial_E = 52;
int position_final_E = 180;

int d_time_R_servo = 25;
int d_time_W_servo = 10;
int d_time_E_servo = 75;

int incr_R_servo = 1;
int incr_W_servo = 1;
int incr_E_servo = 2;

void run_motor(int duty, PinName motorPin_F, PinName motorPin_B) 
{
  //duty: if > 0, turn motor forward as described above
  //      if < 0, turn motor backward as described above
  if (duty > 0) {
    pwm_start(motorPin_B,PWMfreq,1,TICK_COMPARE_FORMAT);
    pwm_start(motorPin_F,PWMfreq,duty,TICK_COMPARE_FORMAT);
  } else {
    duty = duty*(-1);
    pwm_start(motorPin_F,PWMfreq,1,TICK_COMPARE_FORMAT);
    if (duty == 0) {
      duty = 1;
    }
    pwm_start(motorPin_B,PWMfreq,duty,TICK_COMPARE_FORMAT);
  }
}

void stop_motion() 
{
  run_motor(min_motor_duty,MOTOR_L_F,MOTOR_L_R);
  run_motor(min_motor_duty,MOTOR_R_F,MOTOR_R_R);
}

void move_backwards()
{
  run_motor(nominal_backwards_R_duty,MOTOR_R_F,MOTOR_R_R);
  run_motor(nominal_backwards_L_duty,MOTOR_L_F,MOTOR_L_R);
}

void intrpt_PAUSE_DUMP()
{
  stop_motion();
  rear_hatch++;

  for (int time = 0;time<pause_duration;time = time + 1000) {
    display.clearDisplay();
    display.setCursor(0,0);
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.println(pause_duration-time);

    //Proportional/Derivative Gain
    display.print("pg: ");
    int proportional_gain = analogRead(p_pot);
    display.println(proportional_gain);
    display.setCursor(0,30);
    display.print("dg: ");
    int differential_gain = analogRead(d_pot);
    display.println(differential_gain);

    //Tape Sensor
    int position_L_analog = analogRead(TAPE_SENSOR_L);
    int position_R_analog = analogRead(TAPE_SENSOR_R);
    display.print("pos_L: ");
    display.println(position_L_analog);
    display.print("pos_R: ");
    display.println(position_R_analog);

    //Current state threshold
    display.print("csc: ");
    display.println(current_state_thresh);

    display.display();
    delay(1000);
  }
}

void intrpt_CSC_THRESH()
{
  current_state_thresh = current_state_thresh + 50;
}

void Servo_Run(char letter, int pos_start, int pos_write, int d_time, int incr) {
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.print(letter);
  display.print(" to: ");
  display.println(pos_write);
  display.display();

if (letter == 'W') {
    servo.write(pos_start);
    position_current = pos_start;
    delay(250);

    if (pos_start < pos_write) {
      while (position_current < pos_write) {
        servo.write(position_current);
        delay(d_time);
        position_current += incr;
      }
    } else {
      while (position_current > pos_write) {
        servo.write(position_current);
        delay(d_time);
        position_current -= incr;
      }
    }
  }

  if (letter == 'E') {
    servo1.write(pos_start);
    position_current = pos_start;
    delay(250);

    if (pos_start < pos_write) {
      while (position_current < pos_write) {
        servo1.write(position_current);
        delay(d_time);
        position_current += incr;
      }
    } else {
      while (position_current > pos_write) {
        servo1.write(position_current);
        delay(d_time);
        position_current -= incr;
      }
    }
  }

  if (letter == 'R') {
    servo2.write(pos_start);
    position_current = pos_start;
    delay(250);

    if (pos_start < pos_write) {
      while (position_current < pos_write) {
        servo2.write(position_current);
        delay(d_time);
        position_current += incr;
      }
    } else {
      while (position_current > pos_write) {
        servo2.write(position_current);
        delay(d_time);
        position_current -= incr;
      }
    }
  }
  delay(300);
}

void setup() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  delay(2000);

  servo.attach(SERVO);
  servo1.attach(SERVO1);
  servo2.attach(SERVO2);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Hello world!");
  display.display();

  pinMode(TAPE_SENSOR_L, INPUT);
  pinMode(TAPE_SENSOR_R, INPUT);
  pinMode(p_pot,INPUT);
  pinMode(d_pot,INPUT);

  pinMode(MOTOR_L_F, OUTPUT);
  pinMode(MOTOR_L_R, OUTPUT);
  pinMode(MOTOR_R_F, OUTPUT);
  pinMode(MOTOR_R_R, OUTPUT);

  run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
  run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);

  //2mm switch
  pinMode(PAUSE_DUMP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PAUSE_DUMP), intrpt_PAUSE_DUMP, RISING);

  //push button
  pinMode(CSC_THRESH, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(CSC_THRESH), intrpt_CSC_THRESH, RISING);

  //sonar
  pinMode(ECHO, INPUT);
  pinMode(TRIG, OUTPUT);
}

void loop() {

  if (num_loops == 2) {
    servo.write(position_initial_W);
    servo1.write(position_initial_E);
    servo2.write(position_initial_R);
  }

  //Rear Dump
  if (drop_hatch < rear_hatch) {
    stop_motion();
    Servo_Run('R', position_initial_R, position_final_R, d_time_R_servo, incr_R_servo);
    Servo_Run('R', position_final_R, position_initial_R, d_time_R_servo, incr_R_servo);
    rear_hatch = 0;
    backing_up = 0;
  }

  if (backing_up != 1) {
    /*
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.print("distance: ");
    display.println(distance);
    display.display();
    */

    if (current_state_count > 100 && previous_error == 0) {
    // if (num_loops % 25 == 0) { //400
    //   //Sonar setup
    //   digitalWrite(TRIG, LOW);
    //   delay(1);
    //   digitalWrite(TRIG, HIGH);
    //   delay(5);
    //   digitalWrite(TRIG, LOW);
    //   double duration = pulseIn(ECHO, HIGH);
    //   distance = duration*0.034/2;
    // }

      //Sonar setup
      digitalWrite(TRIG, LOW);
      delay(1);
      digitalWrite(TRIG, HIGH);
      delay(5);
      digitalWrite(TRIG, LOW);
      double duration = pulseIn(ECHO, HIGH);
      distance = duration*0.034/2;

      // Sees can
      if (distance < distance_detect) {
        stop_motion();

        Servo_Run('W', position_initial_W, position_final_W, d_time_W_servo, incr_W_servo);
        Servo_Run('W', position_final_W, position_initial_W, d_time_W_servo, incr_W_servo);

        Servo_Run('E', position_initial_E, position_final_E, d_time_E_servo, incr_E_servo);
        Servo_Run('E', position_final_E, position_initial_E, d_time_E_servo, incr_E_servo);

        distance = STARTING_DUMMY_VALUE;
      }
    }

    //PID
    int position_L_analog = analogRead(TAPE_SENSOR_L);
    int position_R_analog = analogRead(TAPE_SENSOR_R);
    int position_L;
    int position_R;
    int error;    

    //convert analog tape sensor reading to digital position
    if (position_L_analog > ON_OFF_THRESHOLD) {
      position_L = 1; //on tape
    } else {
      position_L = 0; //off tape
    }

    if (position_R_analog > ON_OFF_THRESHOLD) {
      position_R = 1; //on tape
    } else {
      position_R = 0; //off tape
    }

    if (position_L == 1 and position_R == 1) {
      error = 0; //no error since both are on tape
    } else if (position_L == 0 and position_R == 1) {
      error = -1; //left sensor is off tape and right is on tape... think of negative when robot is moving left
    } else if (position_L == 1 and position_R == 0) {
      error = 1; //left sensor is on tape and right is off tape... think of positive when robot is moving right
    } else if (previous_error == -1 || previous_error == -dist_between_sensors) { //at this point both sensors must be off the tape. Must remember where it was the previous loop...
      error = -dist_between_sensors; //need an error proportional to the distance between the sensors so you get linear PID control
      //negative means that the left tape sensor left the tape first, therefore the robot must turn right to get back on track
    } else if (previous_error == 1 || previous_error == dist_between_sensors) {
      error = dist_between_sensors;
      //positive means that the right tape sensor left the tape first, therefore the robot must turn left to get back on track
    } else if (previous_error == 0) {
      error = dist_between_sensors; // pretty sure this case is impossible, but just in case, I'm going with the 50% chance that the robot will be on the right side of the tape...
    }

    int p_gain = analogRead(p_pot); //purely for testing - once we have the final values we will remove the potentiometers
    int d_gain = analogRead(d_pot);

    p_gain = p_gain * gain_scaling_factor;
    d_gain = d_gain * gain_scaling_factor;

    //proportional control
    double p = error*p_gain;

    //derivative control - wrote when i was very tired so I may have made mistakes
    double slope = 0;
    double d = 0; //for very first state the derivative control should be 0

    //should this go before or after the d calculation block? if before, then am taking the difference in time including the 
    if (previous_error == error) {
      current_state_count++;
    } else if (previous_error != STARTING_DUMMY_VALUE) {
      previous_state_count = current_state_count; // if a change in state has occurred, the current number of instances now becomes the number of instances in the previous state.
      previous_state = previous_error; //store the previous state when change occurs
      current_state_count = 1; //start at 1 because 1 instance has occurred.
    }

    if (previous_state_count != STARTING_DUMMY_VALUE) {
      slope = (double)(error - previous_state) / (previous_state_count + current_state_count - 1) * slope_scaling_factor; //-1 because i want it to be the true difference in time (time final - time initial), not with an extra increment (look at the graph at the end of lecture 5 to understand what I mean)
      d = slope*d_gain;
    }
    previous_error = error;
    //potential bug: derivative does not go to 0 as soon as tape sensors are aligned on tape. Have to test and see if this is an issue or not.

    int g = (p+d) * correction_scaling_factor;

    if (num_loops % 2000 == 0) { //shouldnt print to screen every loop -> slows down tape following execution
      display.clearDisplay();
      display.setCursor(0,0);
      display.print("pg");
      display.setCursor(25,0);
      display.print(p_gain);
      display.setCursor(63,0);
      display.print("dg");
      display.setCursor(88,0);
      display.print(d_gain);
      display.setCursor(0,20);
      display.print("p");
      display.setCursor(13,20);
      display.print((int)p);
      display.setCursor(0,40);
      display.print("d");
      display.setCursor(13,40);
      display.print((int)d);
      display.setCursor(50,40);
      display.print((int)g);
      display.setCursor(0,55);
      display.print("csc");
      display.setCursor(25,55);
      display.print(current_state_thresh);
      display.display();
    }

    if (position_L_analog <= ON_ELEVATED_TAPE && position_R_analog <= ON_ELEVATED_TAPE) {
      move_backwards();
      delay(1000);
      backing_up = 1;
    } else {
      if (current_state_count >= current_state_thresh) {
        if (error > 0) {
          run_motor(nominal_motor_R_duty + g*(1.5),MOTOR_R_F,MOTOR_R_R); //+1.1
          run_motor(nominal_motor_L_duty - g*(1.45),MOTOR_L_F,MOTOR_L_R); //+1.25
          //run_motor(min_motor_duty,MOTOR_L_F,MOTOR_L_R);
          //run_motor(-nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R); //+1.25
        } else if (error < 0) {
          // g is negative for error < 0 so that's why we multiply it by a -1
          run_motor(nominal_motor_L_duty + g*(-1.4),MOTOR_L_F,MOTOR_L_R); //-1.1
          run_motor(nominal_motor_R_duty - g*(-1.6),MOTOR_R_F,MOTOR_R_R); //-1.25
          //run_motor(min_motor_duty,MOTOR_R_F,MOTOR_R_R); 
          //run_motor(-nominal_motor_L_duty,MOTOR_R_F,MOTOR_R_R); 
        } else {
          run_motor(nominal_motor_R_duty + straight_factor,MOTOR_R_F,MOTOR_R_R);
          run_motor(nominal_motor_L_duty + straight_factor,MOTOR_L_F,MOTOR_L_R);
        }
      } else {
        //backspin if current_state_count < current_state_thresh
        if (error > 0) {
          run_motor(nominal_motor_R_duty + g*(1.4),MOTOR_R_F,MOTOR_R_R); //+1.1
          run_motor(-nominal_motor_L_duty - g*(1.45),MOTOR_L_F,MOTOR_L_R); //+1.25//+0.9
        } else if (error < 0) {
          run_motor(nominal_motor_L_duty + g*(-1.5),MOTOR_L_F,MOTOR_L_R); //-1.1
          run_motor(-nominal_motor_R_duty - g*(+1.45),MOTOR_R_F,MOTOR_R_R); //+1.25//+0.9
        } else {
          run_motor(nominal_motor_R_duty,MOTOR_R_F,MOTOR_R_R);
          run_motor(nominal_motor_L_duty,MOTOR_L_F,MOTOR_L_R);
        }
      }
    }
  } else {
    move_backwards();
    delay(1000);
  }
  num_loops++;
};