#include<Arduino.h>
#include <SimpleFOC.h>

/** Annoyingly some i2c sensors (e.g. AS5600) have a fixed chip address.  This means only one of these devices can be addressed on a single bus
 * This example shows how a second i2c bus can be used to communicate with a second sensor.  
 */ 
#define AS5600_I2C_SDA 21
#define AS5600_I2C_SCL 22

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2C_AS5600 = TwoWire(0);


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(27, 26, 25, 19);

// angle set point variable
float target_voltage = 0;
// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_voltage, cmd);}
void sendvel(char* cmd)  {Serial.println(motor.shaft_velocity);}  //只接收目标值
void send_angle(char* cmd)  {Serial.println(motor.shaft_angle);}

void setup() {

  Serial.begin(9600);
  _delay(750);



  // Normally SimpleFOC will call begin for i2c but with esp32 begin() is the only way to set pins!
  // It seems safe to call begin multiple times
  I2C_AS5600.begin(AS5600_I2C_SDA, AS5600_I2C_SCL, (uint32_t)400000);
  sensor.init(&I2C_AS5600);
  

  // // link the motor to the sensor
  motor.linkSensor(&sensor);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // choose FOC modulation (optional)
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;

  // contoller configuration
  // default parameters in defaults.h

  // velocity PI controller parameters
  motor.PID_velocity.P = 0.2f;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0;
  // maximal voltage to be set to the motor
  motor.voltage_limit = 6;

  // velocity low pass filtering time constant
  // the lower the less filtered
  motor.LPF_velocity.Tf = 0.01f;

  // angle P controller
  motor.P_angle.P = 20;
  // maximal velocity of the position control
  motor.velocity_limit = 20;

  
  // comment out BELOW if not needed
  motor.useMonitoring(Serial);
//display variables
//  motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE; 
// downsampling
  motor.monitor_downsample = 100; // default 10



  // initialize motor
  motor.init();
  // align sensor and start FOC
  motor.initFOC(5.07,CCW);


// add target command T
  command.add('T', doTarget, "target voltage");
  command.add('S', sendvel,(char *)"target");     //接收电机的运动指令
  command.add('A', send_angle,(char *)"target");  //接收电机的运动指令


}

void loop() {
  // iterative function updating the sensor internal variables
  // it is usually called in motor.loopFOC()
  // this function reads the sensor hardware and 
  // has to be called before getAngle nad getVelocity
  // sensor0.update();
  // //sensor1.update();
  
  // _delay(200);
  // Serial.print(sensor0.getAngle()); 
  // Serial.print(" - "); 
  // //Serial.print(sensor1.getAngle());
  // Serial.println();

  // main FOC algorithm function
  // the faster you run this function the better
  // Arduino UNO loop  ~1kHz
  // Bluepill loop ~10kHz
  motor.loopFOC();

  // Motion control function
  // velocity, position or voltage (defined in motor.controller)
  // this function can be run at much lower frequency than loopFOC() function
  // You can also use motor.move() and set the motor.target in the code
  motor.move(target_voltage);

  // MONITOR FUNC
  motor.monitor();


// user communication
  command.run();

}