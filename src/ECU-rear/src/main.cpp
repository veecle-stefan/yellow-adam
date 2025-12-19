#include <Arduino.h>
#include <SimpleFOC.h>

constexpr int PP = 15;


//18——Corresponding Board Pins SCL_0
//19——Corresponding Board Pins SDA_0
//15——Corresponding Board Pins I_0
//5——Corresponding Board Pins SCL_0
//23——Corresponding Board Pins SDA_0
//13——Corresponding Board Pins I_0
HallSensor sensor1 = HallSensor(5, 23, 13, PP); // U V W Pole Pairs
HallSensor sensor2 = HallSensor(18, 19, 15, PP);// U V W Pole Pairs
void doA(){sensor1.handleA();}
void doB(){sensor1.handleB();}
void doC(){sensor1.handleC();}


void doA1(){sensor2.handleA();}
void doB1(){sensor2.handleB();}
void doC1(){sensor2.handleC();}

//Motor parameters: Set the number of pole pairs according to the motor
BLDCMotor motor1 = BLDCMotor(PP);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor2 = BLDCMotor(PP);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(26, 27, 14, 21);

InlineCurrentSense current_sense1 = InlineCurrentSense(0.01f, 50.0f, 39, 36);
InlineCurrentSense current_sense2 = InlineCurrentSense(0.01f, 50.0f, 35, 34);

Commander command = Commander(Serial);
void doMotor1(char* cmd) { command.motor(&motor1, cmd); }
void doMotor2(char* cmd) { command.motor(&motor2, cmd); }

uint32_t lastUpdate = 0;

float target = 0.0f;

float m1angle = 0.f;
float m2angle = 0.f;

void setup() {
  Serial.begin(115200);
  SimpleFOCDebug::enable(&Serial);

  sensor1.pullup = Pullup::USE_INTERN;
  sensor2.pullup = Pullup::USE_INTERN;

  sensor1.init();
  sensor2.init();
  sensor1.enableInterrupts(doA, doB, doC);
  sensor2.enableInterrupts(doA1, doB1, doC1);

  
  //Connect the Motor Object to the Sensor Object
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  //Supply voltage setting [V]
  driver1.voltage_power_supply = 12;
  driver1.init();

  driver2.voltage_power_supply = 12;
  driver2.init();
  //Connect the Motor and Driver Objects
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);
  current_sense1.linkDriver(&driver1);
  current_sense1.skip_align = true;
  current_sense1.init();
  motor1.linkCurrentSense(&current_sense1);
  current_sense1.offset_ia = 1.53f;
  current_sense1.offset_ib = 1.56f;
  

  // after driver2.init();
  current_sense2.linkDriver(&driver2);
  current_sense2.skip_align = true;
  current_sense2.init();
  motor2.linkCurrentSense(&current_sense2);
  current_sense2.offset_ia = 1.54f;
  current_sense2.offset_ib = 1.52f;
  
  //Motion Control Mode Settings
  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor2.torque_controller = TorqueControlType::voltage; 
  motor2.controller = MotionControlType::torque;

  motor1.voltage_limit = 6;
  motor2.voltage_limit = 6;

  motor1.PID_velocity.output_ramp = 1000;
  motor2.PID_velocity.output_ramp = 1000;
  
  
  //Set maximum speed limit
  motor1.velocity_limit = 45;
  motor2.velocity_limit = 45;
  motor1.phase_resistance = 1.0f;
  motor2.phase_resistance = 1.0f;

    // FOC PID
   motor1.PID_current_q.P = 0.5f;                      
   motor1.PID_current_q.I= 50;
   motor1.PID_current_q.limit = 2;
   motor1.PID_current_q.output_ramp = 1000;
   motor1.PID_current_d.limit = 2;
   motor1.PID_current_d.P= 2;
   motor1.PID_current_d.I = 50;
   motor1.PID_current_d.output_ramp = 1000;
   motor1.LPF_current_q.Tf = 0.005; // 1ms default
   motor1.LPF_current_d.Tf = 0.005; // 1ms default

   motor2.PID_current_q.P = 0.5f;
   motor2.PID_current_q.I= 50;
   motor2.PID_current_q.limit = 2;
   motor2.PID_current_d.limit = 2;
   motor2.PID_current_q.output_ramp = 1000;
   motor2.PID_current_d.P= 2;
   motor2.PID_current_d.I = 50;
   motor2.PID_current_d.output_ramp = 1000;
   motor2.LPF_current_q.Tf = 0.005; // 1ms default
   motor2.LPF_current_d.Tf = 0.005; // 1ms default

    // PID Velocity
    motor1.PID_velocity.P = 0.1;
    motor1.PID_velocity.I = 1;
    motor1.PID_velocity.D = 0;

    motor2.PID_velocity.P = 0.1;
    motor2.PID_velocity.I = 1;
    motor2.PID_velocity.D = 0;
  
  motor1.zero_electric_angle  = 2.094395f;
  motor1.sensor_direction     = Direction::CCW;
  motor2.zero_electric_angle = 2.094395;
  motor2.sensor_direction    = Direction::CCW;
  
  //Initialize Motor
  motor1.init();
  motor2.init();
  //Initialize FOC
  motor1.initFOC();
  motor2.initFOC();

  motor1.enable();

  command.add('A',doMotor1,"motor1");
  command.add('B',doMotor2,"motor1");

  motor1.useMonitoring(Serial); 
  motor2.useMonitoring(Serial);
  motor1.monitor_downsample = 100; // disable monitor at first - optional
  motor2.monitor_downsample = 100; // disable monitor at first - optional
  motor1.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;
  motor2.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE | _MON_CURR_Q;


  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}

void loop() {
  // put your main code here, to run repeatedly:
  motor1.loopFOC();
  motor2.loopFOC();

  motor1.move();
  motor2.move();
  motor1.monitor();
  motor2.monitor();
  command.run();


}
