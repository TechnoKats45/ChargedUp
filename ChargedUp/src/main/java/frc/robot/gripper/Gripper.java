
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

// the following three lines are for the dual redline motors on Gripper II
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// the following three lines are for the single NEO550 motor on Gripper III
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

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
  Servo grabberclaw;

  //  motorized theta for rotation
  TalonSRX rotator;

  //  encoder to measure theta
  Encoder thetaAngle;

  //  motorized intakes
  TalonSRX leftIntake;
  TalonSRX rightIntake;

  CANSparkMax intake;

  //  rangefinder to detect game pieces in grabber
  Counter lidar;

  //  configuration constants
  Config config = new Config();

  //  automatic grab mode -- based on gamepiece sensor
  boolean autograb = false;

  //  rotation set point: true is for forward, false is for backwards
  boolean rotation = true;

  GripperState clawState = GripperState.CONEIN;

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

  boolean c_gripflip() {
    return control.getRawButton(config.kj_trig);
  }

  void setIntake(double speed) {
    if (config.grab_twinredline) {
      leftIntake.set(ControlMode.PercentOutput,speed);
      rightIntake.set(ControlMode.PercentOutput,speed);  
    }
    if (config.grab_neo550) {
      intake.set(speed);
    }
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
    grabberclaw.set(config.kk_servocone);
    SmartDashboard.putString("gripper/state", "CONE");
    clawState = GripperState.CONEOUT;
  }

  //  open grabber claws
  public void grabCube() {
    grabberclaw.set(config.kk_servocube);
    SmartDashboard.putString("gripper/state", "CUBE");
    clawState = GripperState.CUBEOUT;
  }

  public void intake() {
    if (config.grab_twinredline) {
      setIntake(-0.5);
    }
    if (config.grab_neo550) {
      setIntake(-1.0);
    }
    SmartDashboard.putString("intake state", "Intaking");
    if (clawState == GripperState.CUBEOUT) {
      clawState = GripperState.CUBEIN;
    } else if (clawState == GripperState.CONEOUT) {
      clawState = GripperState.CONEIN;
    }
  }

  public void release() {
    if (config.grab_twinredline) {
      setIntake(0.25);
    }
    if (config.grab_neo550) {
      setIntake(0.25);
    }
    SmartDashboard.putString("intake state", "Releasing");
    if (clawState == GripperState.CUBEIN) {
      clawState = GripperState.CUBEOUT;
    } else if (clawState == GripperState.CONEIN) {
      clawState = GripperState.CONEOUT;
    }
  }

  public void hold() {
    if (config.grab_twinredline) {
      setIntake(-0.1);
    } 
    if (config.grab_neo550) {
      setIntake(0);
    }
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
    } else {
      position = 22;
    }
    double speed = clamp(thetaPID.calculate(thetaAngle.get(), position), 0.3); 
    rotator.set(ControlMode.PercentOutput,speed);
    SmartDashboard.putNumber("gripper/rotate", speed);
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
    clawState = GripperState.AUTORELEASE;
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
    
    grabberclaw = new Servo(config.ksp_grip);

    // Create a new Counter object in two-pulse mode
    lidar = new Counter(Counter.Mode.kSemiperiod);
    // Set up the input channel for the counter
    lidar.setUpSource(config.kdi_gamepiecesense);
    // Set the encoder to count pulse duration from rising edge to falling edge
    lidar.setSemiPeriodMode(true);

    thetaAngle = new Encoder(9, 8);

    if (config.grab_twinredline) {
      leftIntake = new TalonSRX(config.kmc_leftintake);
      rightIntake = new TalonSRX(config.kmc_rightintake);  
      leftIntake.setNeutralMode(NeutralMode.Brake);
      rightIntake.setNeutralMode(NeutralMode.Brake);
    }
    if (config.grab_neo550) {
      intake = new CANSparkMax(config.kmc_intake, MotorType.kBrushless);
      intake.setInverted(config.kk_intakeinvert);
      intake.setIdleMode(IdleMode.kBrake);
      intake.setSmartCurrentLimit(config.kk_grabcurrentlimit);
    }
    rotator = new TalonSRX(config.kmc_theta);
    
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
    } else if (c_release() && !c_intake()) {
      release();
    } else {
      switch (clawState) {
        case CONEIN:
        case CUBEIN:
          hold();
          break;
        case AUTORELEASE:
          release();
          if(releaseTimer.hasElapsed(0.5)) {
            clawState = GripperState.CONEOUT;
          }
          break;
        default:
          rest();
      }
    }
    double armAngle = SmartDashboard.getNumber("elevation/angle", 0);
    if (!c_gripflip()) {
      if (armAngle < -5 ) {
        rotate(false);
      }
      if (armAngle > 5) {
        rotate(true);
      }
      theta();
    } else {
      rotator.set(ControlMode.PercentOutput, -control.getX());
      thetaAngle.reset();
    }
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
    SmartDashboard.putNumber("gripper/sense Inches", gamepieceInches());
    SmartDashboard.putBoolean("gripper/autograb", autograb);
    SmartDashboard.putNumber("gripper/theta", thetaAngle.get());
    if (config.grab_twinredline) {
      SmartDashboard.putNumber("gripper/grab current", leftIntake.getSupplyCurrent());
    }
    if (config.grab_neo550) {
      SmartDashboard.putNumber("gripper/grab current", intake.getOutputCurrent());
    }
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
    // control claw 
    double throttle = control.getThrottle();
    grabberclaw.set(throttle);
    SmartDashboard.putNumber("gripper/claw", throttle);
    // normal operation is already sufficient to test intake and wrist
    run();
  }

  enum GripperState {
    CUBEIN, CUBEOUT, CONEIN, CONEOUT, AUTORELEASE;
  }

// end of Gripper class
}