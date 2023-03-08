
// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//   ####  ####    ###   ####   ####   #####  ####
//  #      #   #    #    #   #  #   #  #      #   #
//  # ###  ####     #    ####   ####   ####   ####
//  #   #  #   #    #    #      #      #      #   #
//   ###   #   #   ###   #      #      #####  #   #
//
//  Gripper subsystem
//
//  pneumatic solenoid valve for grabber cylinders
//  pneumatic solenoid valve for gripper rotation
//  infrared laser rangefinder
//
package frc.robot.gripper;

//  operator control is an ATTACK 3 joystick
import edu.wpi.first.wpilibj.Joystick;

//  telemetry goes to the Dashboard for display
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//  actuator is a pair of pneumatic cylinders plumbed to a double solenoid wired to a Pneumatic Control Module
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
//  sensor is a rangefinder with PWM output, readable by a RoboRIO counter on a digital input
import edu.wpi.first.wpilibj.Counter;

//  configuration constants (PCM ports, Digital Input IDs, etc.) are in the Config.java file
import frc.robot.config.*;


public class Gripper {

//   ###   #       ###   ####    ###   #       ####  
//  #      #      #   #  #   #  #   #  #      #      
//  #  ##  #      #   #  ####   #####  #       ###   
//  #   #  #      #   #  #   #  #   #  #          #  
//   ###   #####   ###   ####   #   #  #####  ####   
//
//  global variables

  //  operator control is an ATTACK 3 joystick
  Joystick control;

  // driver controller is an xbox controller
  XboxController driver;

  //  servo for grabber
  Servo grabber;

  //  motorized theta for rotation
  MotorController rotator;

  //  encoder to measure theta
  AnalogPotentiometer thetaAngle;

  //  motorized intakes
  MotorController leftIntake;
  MotorController rightIntake;

  //  rangefinder to detect game pieces in grabber
  Counter lidar;

  //  configuration constants
  Config config = new Config();

  //  automatic grab mode -- based on gamepiece sensor
  boolean autograb = false;

  //  rotation set point: true is for forward, false is for backwards
  boolean rotation = true;

  PIDController thetaPID = new PIDController(config.kP_theta, config.kI_theta, config.kD_theta);


//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

  //  close grabber claws
  boolean c_cube() {
    return driver.getRightTriggerAxis() > 0.2 || driver.getYButton();
  }

  //  open grabber claws
  boolean c_cone() {
    return driver.getLeftTriggerAxis() > 0.2 || driver.getXButton();
  }

  boolean c_rotateForward() {
    return false;
    // return control.getRawButton(config.kj_centerleft);
  }

  boolean c_rotateReverse() {
    return false;
    // return control.getRawButton(config.kj_centerright);
  }

  boolean c_intake() {
    return control.getRawButton(config.kj_leftnear);
  }

  boolean c_release() {
    return control.getRawButton(config.kj_leftfar);
  }

  void setIntake(double speed) {
    leftIntake.set(speed);
    rightIntake.set(speed);
  }

//
//   ###    ###   #####  #   #   ###   #####  #####  
//  #   #  #   #    #    #   #  #   #    #    #      
//  #####  #        #    #   #  #####    #    ####   
//  #   #  #   #    #    #   #  #   #    #    #      
//  #   #   ###     #     ###   #   #    #    #####  
//
//  private functions for motor/pneumatic/servo output

  //  close grabber claws
  void grabCone() {
    grabber.set(-.5);
    SmartDashboard.putString("grip/state", "CONE");
  }

  //  open grabber claws
  public void grabCube() {
    grabber.set(.5);
    SmartDashboard.putString("grip/state", "CUBE");
  }

  public void intake() {
    setIntake(1);
    SmartDashboard.putString("intake state", "Intaking");
  }

  public void release() {
    setIntake(-1);
    SmartDashboard.putString("intake state", "Releasing");
  }

  public void hold() {
    setIntake(0);
    SmartDashboard.putString("intake state", "Holding");
  }

  //  rotate grabber
  void rotate(boolean val) {
    rotation = val;
  }

  //  This function runs the PID loop to rotate the gripper
  void theta() {
    double position;
    if (rotation) {
      position = 0; //TODO: set actual values
    }
    else {
      position = 1;
    }
    rotator.set(thetaPID.calculate(thetaAngle.get(), position));
  }

//
//   ####  #####  #   #   ####  #####  
//  #      #      ##  #  #      #      
//   ###   ####   # # #   ###   ####   
//      #  #      #  ##      #  #      
//  ####   #####  #   #  ####   #####  
//
//  private functions for sensor feedback

// Pololu Distance Sensor, 50 cm max:
// d = (3 mm/4 us)*(t-1000us)
// Pololu Distance Sensor, 300 cm max:
// d = (4 mm/1 us)*(t-1000us)
double gamepieceInches() {
  // counter is in Semi-period mode measuring pulse width from rising edge to falling edge
  // returned value is measured pulse width in seconds so needs to be multiplied by 10^6
  // computed value is in millimeters so needs to be divided by 25.4 to return inches
  double pulsewidth = lidar.getPeriod();
  return (4./1.) * ((pulsewidth*1000000.)-1000.) / 25.4;
}

//
//   ###   #   #  #####   ###   
//  #   #  #   #    #    #   #  
//  #####  #   #    #    #   #  
//  #   #  #   #    #    #   #  
//  #   #   ###     #     ###   
//
//  public functions for autonomous input

  /* Deprecated
  // true to close claw; false to open claw
  public void auto_grip(boolean value) {
    if (value) {
      grab();
    } else {
      release();
    }
  
  }
  */


//
//   ###    ###   #   #   ####  #####  ####   #   #   ###   #####   ###   ####
//  #   #  #   #  ##  #  #        #    #   #  #   #  #   #    #    #   #  #   #
//  #      #   #  # # #   ###     #    ####   #   #  #        #    #   #  ####
//  #   #  #   #  #  ##      #    #    #   #  #   #  #   #    #    #   #  #   #
//   ###    ###   #   #  ####     #    #   #   ###    ###     #     ###   #   #
//
//  creates a new Gripper 

  public Gripper(Joystick userControl, XboxController driver) {
    // initialize

    control = userControl;
    this.driver = driver;
    
    grabber = new Servo(0);
    rotator = new Spark(0);

    // Create a new Counter object in two-pulse mode
    lidar = new Counter(Counter.Mode.kSemiperiod);
    // Set up the input channel for the counter
    lidar.setUpSource(config.kdi_gamepiecesense);
    // Set the encoder to count pulse duration from rising edge to falling edge
    lidar.setSemiPeriodMode(true);
    
    SmartDashboard.putNumber("gripper/kP",thetaPID.getP());
    SmartDashboard.putNumber("gripper/kI",thetaPID.getI());
    SmartDashboard.putNumber("gripper/kD",thetaPID.getD());
  }


//
//  ####   #   #  #   #
//  #   #  #   #  ##  #
//  ####   #   #  # # #
//  #   #  #   #  #  ##
//  #   #   ###   #   #
//
//  does everything necessary when the robot is enabled, either autonomous or teleoperated
  public void run() {
    if (c_cube()) {
      grabCone();
    }
    if (c_cone()) {
      grabCube();
    }
    if (c_intake() && !c_release() && gamepieceInches() > config.kk_holdrange /*TODO: replace this with an actual value */) {
      intake();
    }
    else if (c_release() && !c_intake()) {
      release();
    }
    else {
      hold();
    }
    double armAngle = SmartDashboard.getNumber("elevation/angle", 0);
    if (armAngle < -5 ) {
      rotate(true);
    }
    if (armAngle > 5) {
      rotate(false);
    }
    theta();
  }


//
//   ###   ####   #      #####  
//    #    #   #  #      #      
//    #    #   #  #      ####   
//    #    #   #  #      #      
//   ###   ####   #####  #####  
//
//  does everything necessary when the robot is running, either enabled or disabled
  public void idle() {
    SmartDashboard.putNumber("grip/sense Inches", gamepieceInches());
    SmartDashboard.putBoolean("grip/autograb", autograb);
  }

  
// 
//  #####  #####   ####  #####
//    #    #      #        #
//    #    ####    ###     #
//    #    #          #    #
//    #    #####  ####     #
//
//  provides special support for testing individual subsystem functionality
  public void test() {
    // there's nothing special to do because normal operation is already sufficient to test the single actuator
    run();
  }

// end of Gripper class
}
