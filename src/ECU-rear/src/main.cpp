#include <Arduino.h>
#include <SimpleFOC.h>
//18——Corresponding Board Pins SCL_0
//19——Corresponding Board Pins SDA_0
//15——Corresponding Board Pins I_0
//1——Number of Pole Pairs
HallSensor sensor = HallSensor(18, 19, 15, 1);// U V W Pole Pairs
void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}
//5——Corresponding Board Pins SCL_0
//23——Corresponding Board Pins SDA_0
//13——Corresponding Board Pins I_0
//1——Pole Pairs
HallSensor sensor1 = HallSensor(5, 23, 13, 1); // U V W Pole Pairs
void doA1(){sensor1.handleA();}
void doB1(){sensor1.handleB();}
void doC1(){sensor1.handleC();}

//Motor parameters: Set the number of pole pairs according to the motor
BLDCMotor motor = BLDCMotor(1);
BLDCDriver3PWM driver = BLDCDriver3PWM(32, 33, 25, 22);

BLDCMotor motor1 = BLDCMotor(1);
BLDCDriver3PWM driver1 = BLDCDriver3PWM(26, 27, 14, 21);

//Command Settings
float target_velocity = 5;
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }

float m1angle = 0.f;
float m2angle = 0.f;

void setup() {
  Serial.begin(115200);

  sensor.init();
  sensor1.init();
  sensor.enableInterrupts(doA, doB, doC);
  sensor1.enableInterrupts(doA1, doB1, doC1);

  
  //Connect the Motor Object to the Sensor Object
  motor.linkSensor(&sensor);
  motor1.linkSensor(&sensor1);

  //Supply voltage setting [V]
  driver.voltage_power_supply = 24;
  driver.init();

  driver1.voltage_power_supply = 24;
  driver1.init();
  //Connect the Motor and Driver Objects
  motor.linkDriver(&driver);
  motor1.linkDriver(&driver1);

  // aligning voltage [V]
  motor.voltage_sensor_align = 3;
  // index search velocity [rad/s]
  motor.velocity_index_search = 3;
  
  //Motion Control Mode Settings
  motor.controller = MotionControlType::velocity;
  motor1.controller = MotionControlType::velocity;

  //Speed PI Loop Settings
  motor.PID_velocity.P = 0.01;
  motor1.PID_velocity.P = 0.01;
  motor.PID_velocity.I = 0.1;
  motor1.PID_velocity.I = 0.1;
  motor.PID_velocity.D = 0;
  motor1.PID_velocity.D = 0;
  //Angle P Ring Settings
  motor.P_angle.P = 20;
  motor1.P_angle.P = 20;
  //Maximum Motor Limit Voltage
  motor.voltage_limit = 6;
  motor1.voltage_limit = 6;

  motor.PID_velocity.output_ramp = 1000;
  motor1.PID_velocity.output_ramp = 1000;
  
  //Velocity low pass filter time constant
  motor.LPF_velocity.Tf = 0.01f;
  motor1.LPF_velocity.Tf = 0.01f;

  //Set maximum speed limit
  motor.velocity_limit = 45;
  motor1.velocity_limit = 45;

  motor.useMonitoring(Serial);
  motor1.useMonitoring(Serial);

  
  //Initialize Motor
  motor.init();
  motor1.init();
  //Initialize FOC
  motor.initFOC();
  motor1.initFOC();
  command.add('T', doTarget, "target velocity");

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target velocity using serial terminal:"));
}

void loop() {
  // put your main code here, to run repeatedly:
 motor.loopFOC();
  motor1.loopFOC();

  motor.move(target_velocity);
  motor1.move(target_velocity);

  command.run();

  float newAngle1 = sensor.getAngle();
  float newAngle2 = sensor1.getAngle();

  if (newAngle1 != m1angle) {
    m1angle = newAngle1;
    Serial.print("M1=");
    Serial.println(newAngle1);
  }
  if (newAngle2 != m2angle) {
    m2angle = newAngle2;
    Serial.print("M2=");
    Serial.println(newAngle2);
  }
}

