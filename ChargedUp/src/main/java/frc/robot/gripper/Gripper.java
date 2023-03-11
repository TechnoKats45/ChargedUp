
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
//  sensor is a rangefinder with PWM output, readable by a RoboRIO counter on a digital input
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
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
  TalonSRX rotator;

  //  encoder to measure theta
  Encoder thetaAngle;

  //  motorized intakes
  TalonSRX leftIntake;
  TalonSRX rightIntake;

  //  rangefinder to detect game pieces in grabber
  Counter lidar;

  //  configuration constants
  Config config = new Config();

  //  automatic grab mode -- based on gamepiece sensor
  boolean autograb = false;

  //  rotation set point: true is for forward, false is for backwards
  boolean rotation = true;

  GripperState state = GripperState.CONEIN;

  Timer releaseTimer = new Timer();

  PIDController thetaPID = new PIDController(config.kP_theta, config.kI_theta, config.kD_theta);

  // limit output to a specified maximum
private double clamp (double in, double range) {
  if (in > range) {
    in = range;
  }
  if (in < -range) {
    in = -range;
  }
  return in;
}


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
    leftIntake.set(ControlMode.PercentOutput,speed);
    rightIntake.set(ControlMode.PercentOutput,speed);
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
    grabber.set(0.01);
    SmartDashboard.putString("grip/state", "CONE");
    state = GripperState.CONEOUT;
  }

  //  open grabber claws
  public void grabCube() {
    grabber.set(0.99);
    SmartDashboard.putString("grip/state", "CUBE");
    state = GripperState.CUBEOUT;
  }

  public void intake() {
    setIntake(-0.5);
    SmartDashboard.putString("intake state", "Intaking");
    if (state == GripperState.CUBEOUT) {
      state = GripperState.CUBEIN;
    }
    else if (state == GripperState.CONEOUT) {
      state = GripperState.CONEIN;
    }
  }

  public void release() {
    setIntake(0.25);
    SmartDashboard.putString("intake state", "Releasing");
    if (state == GripperState.CUBEIN) {
      state = GripperState.CUBEOUT;
    }
    else if (state == GripperState.CONEIN) {
      state = GripperState.CONEOUT;
    }
  }

  public void hold() {
    setIntake(-0.1);
    SmartDashboard.putString("intake state", "Holding");
  }

  public void rest() {
    setIntake(0);
    SmartDashboard.putString("intake state", "Resting");
  }

  //  rotate grabber
  void rotate(boolean val) {
    rotation = val;
  }

  //  This function runs the PID loop to rotate the gripper
  void theta() {
    double position;
    if (rotation) {
      position = 0;
    }
    else {
      position = 22;
    }
    double speed = clamp(thetaPID.calculate(thetaAngle.get(), position), 0.3); 
    rotator.set(ControlMode.PercentOutput,speed);
    SmartDashboard.putNumber("grip/rotate", speed);
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

  public void auto_release () {
    state = GripperState.AUTORELEASE;
    releaseTimer.reset();
    releaseTimer.start();
  }

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
    
    grabber = new Servo(config.ksp_grip);

    // Create a new Counter object in two-pulse mode
    lidar = new Counter(Counter.Mode.kSemiperiod);
    // Set up the input channel for the counter
    lidar.setUpSource(config.kdi_gamepiecesense);
    // Set the encoder to count pulse duration from rising edge to falling edge
    lidar.setSemiPeriodMode(true);

    thetaAngle = new Encoder(9, 8);

    leftIntake = new TalonSRX(config.kmc_leftintake);
    rightIntake = new TalonSRX(config.kmc_rightintake);
    rotator = new TalonSRX(config.kmc_theta);
    leftIntake.setNeutralMode(NeutralMode.Brake);
    rightIntake.setNeutralMode(NeutralMode.Brake);
    
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
      grabCube();
    }
    if (c_cone()) {
      grabCone();
    }
    if (c_intake() && !c_release()) {
      intake();
    }
    else if (c_release() && !c_intake()) {
      release();
    }
    else {
      switch (state) {
        case CUBEIN:
          hold();
          break;
        case AUTORELEASE:
          release();
          if(releaseTimer.hasElapsed(0.5)) {
            state = GripperState.CONEOUT;
          }
          break;
        default:
          rest();
      }
    }
    double armAngle = SmartDashboard.getNumber("elevation/angle", 0);
    if (armAngle < -5 ) {
      rotate(false);
    }
    if (armAngle > 5) {
      rotate(true);
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
    SmartDashboard.putNumber("grip/theta", thetaAngle.get());
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

  enum GripperState {
    CUBEIN, CUBEOUT, CONEIN, CONEOUT, AUTORELEASE;
  }

// end of Gripper class
}