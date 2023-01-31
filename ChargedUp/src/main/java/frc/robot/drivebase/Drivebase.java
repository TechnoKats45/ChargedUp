
// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//  ####   ####    ###   #   #  #####  ####    ###    ####  #####
//  #   #  #   #    #    #   #  #      #   #  #   #  #      #
//  #   #  ####     #    #   #  ####   ####   #####   ###   ####
//  #   #  #   #    #     # #   #      #   #  #   #      #  #
//  ####   #   #   ###     #    #####  ####   #   #  ####   #####
//
//  description
//
//  notes
//
//  warnings
// 
//  behavior
//  actuators
//  sensors

package frc.robot.drivebase;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.config.*;


//  imports (controllers, actuators, sensors, communication)

public class Drivebase {

  // controllers
  // actuators
  // sensors
  // communication

//   ###   #       ###   ####    ###   #       ####  
//  #      #      #   #  #   #  #   #  #      #      
//  #  ##  #      #   #  ####   #####  #       ###   
//  #   #  #      #   #  #   #  #   #  #          #  
//   ###   #####   ###   ####   #   #  #####  ####   
//
//  global variables
    XboxController control;

    CANSparkMax motor_left1;
    CANSparkMax motor_left2;
    CANSparkMax motor_left3;
    CANSparkMax motor_right1;
    CANSparkMax motor_right2;
    CANSparkMax motor_right3;

    Config config = new Config();

    // driving
    double odometerOrigin = 0;
    double directionOrigin = 0;
    enum BrakeModeType {BRAKE, COAST}
    BrakeModeType motorBrakeMode = BrakeModeType.BRAKE;
    enum DriverInputModeType {TANK, ARCADE}
    DriverInputModeType driverInputMode = DriverInputModeType.ARCADE;
  

    // test mode //
    boolean test_runleft = false;
    boolean test_runright = false;
    double test_speed = 0;
    // test mode //
//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

// accommodate a deadband around zero for joystick inputs
private double deadband(double in, double band) {
  double value = 0;
  if (in > band) {
    value = in-band;
  }
  if (in <-band) {
    value = in+band;
  }
  return value / (1-band);
}

// tank control (individual left and right throttles)
  private double c_leftthrottle() {
    return deadband(-control.getLeftY(),0.2);
  }
  private double c_rightthrottle() {
    return deadband(-control.getRightY(),0.2);
  }

// split arcade control (speed on left, steer on right)
  private double c_speed() {
    return deadband(-control.getLeftY(),0.2);
  }
  private double c_steer() {
    return deadband(control.getRightX(),0.2)/5.0;
  }

// check driver input mode
  private DriverInputModeType c_inputMode() {
    if (control.getStartButtonPressed()) {
      if (control.getPOV() == 90) {
        driverInputMode = DriverInputModeType.ARCADE;
      }
      if (control.getPOV() == 270) {
        driverInputMode = DriverInputModeType.TANK;
      }
    }
    return driverInputMode;
  }

// check motor control break/coast mode
  private BrakeModeType c_brakeMode() {
    if (control.getStartButtonPressed()) {
      if (control.getPOV() == 0) {
        motorBrakeMode = BrakeModeType.COAST;
      }
      if (control.getStartButtonPressed()) {
        motorBrakeMode = BrakeModeType.BRAKE;
      }
    }
    return motorBrakeMode;
  }

//
//   ###    ###   #####  #   #   ###   #####  #####  
//  #   #  #   #    #    #   #  #   #    #    #      
//  #####  #        #    #   #  #####    #    ####   
//  #   #  #   #    #    #   #  #   #    #    #      
//  #   #   ###     #     ###   #   #    #    #####  
//
//  private functions for motor/pneumatic/servo output

// set 
  private void leftspeed(double speed) {
    SmartDashboard.putNumber("left speed", speed);
    motor_left1.set(speed);
    motor_left2.set(speed);
    motor_left3.set(speed);
  }

  private void rightspeed(double speed) {
    SmartDashboard.putNumber("right speed:", speed);
    motor_right1.set(speed);
    motor_right2.set(speed);
    motor_right3.set(speed);

  }

//
//   ####  #####  #   #   ####  #####  
//  #      #      ##  #  #      #      
//   ###   ####   # # #   ###   ####   
//      #  #      #  ##      #  #      
//  ####   #####  #   #  ####   #####  
//
//  private functions for sensor feedback

  // read accumulated left side motor position
  private double odoleft() {
    return motor_left1.getEncoder().getPosition();
  }

  // read accumulated right side motor position
  private double odoright() {
    return -motor_right1.getEncoder().getPosition();
  }

  // read the number of inches traveled since the last reset
  private double distance() {
    // raw robot odometer is average of left and right side raw values
    double odoRaw = (odoleft()+odoright())/2.0;
    return (odoRaw - odometerOrigin) * config.kk_inchesPerCount;
  }
  // reset the odometer to zero
  private void distanceReset() {
    odometerOrigin = (odoleft()+odoright())/2.0;
  }

  // read the number of degrees turned since the last reset
  private double direction() {
    // degrees per count is computed from robot wheel size, wheelbase, gearbox ratio, and encoder counts per motor revolution
    // raw robot direction is difference between left and right side raw values
    double dirRaw = (odoleft()-odoright());
    return (dirRaw - directionOrigin) * config.kk_degreesPerCount;
  }
// reset the direction to zero
  private void directionReset() {
    directionOrigin = (odoleft()-odoright());
  }


//
//   ###   #   #  #####   ###   
//  #   #  #   #    #    #   #  
//  #####  #   #    #    #   #  
//  #   #  #   #    #    #   #  
//  #   #   ###     #     ###   
//
//  public functions for autonomous control

//
// utility functions

// create a new motor controller for a NEO brushless motor
  private CANSparkMax newNEO(int canID, boolean isInverted) {
    CANSparkMax motor = new CANSparkMax(canID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(isInverted);
    return motor;
  }

//
//   ###    ###   #   #   ####  #####  ####   #   #   ###   #####   ###   ####
//  #   #  #   #  ##  #  #        #    #   #  #   #  #   #    #    #   #  #   #
//  #      #   #  # # #   ###     #    ####   #   #  #        #    #   #  ####
//  #   #  #   #  #  ##      #    #    #   #  #   #  #   #    #    #   #  #   #
//   ###    ###   #   #  ####     #    #   #   ###    ###     #     ###   #   #
//
//  creates a new Drivebase 

  public Drivebase(XboxController driverControl) {
    // initialize
    control = driverControl;

    motor_left1 = newNEO(config.kmc_left1, config.kk_leftinvert);
    motor_left2 = newNEO(config.kmc_left2, config.kk_leftinvert);
    motor_left3 = newNEO(config.kmc_left3, config.kk_leftinvert);
    motor_right1 = newNEO(config.kmc_right1, config.kk_rightinvert);
    motor_right2 = newNEO(config.kmc_right2, config.kk_rightinvert);
    motor_right3 = newNEO(config.kmc_right3, config.kk_rightinvert);
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
    SmartDashboard.putNumber("POV", control.getPOV());
    BrakeModeType oldBrakeMode = motorBrakeMode;
    if (oldBrakeMode != c_brakeMode()) {
      switch (motorBrakeMode) {
        case BRAKE:
          motor_left1.setIdleMode(IdleMode.kBrake);
          motor_left2.setIdleMode(IdleMode.kBrake);
          motor_left3.setIdleMode(IdleMode.kBrake);
          motor_right1.setIdleMode(IdleMode.kBrake);
          motor_right2.setIdleMode(IdleMode.kBrake);
          motor_right3.setIdleMode(IdleMode.kBrake);
          break;
        case COAST:
          motor_left1.setIdleMode(IdleMode.kCoast);
          motor_left2.setIdleMode(IdleMode.kCoast);
          motor_left3.setIdleMode(IdleMode.kCoast);
          motor_right1.setIdleMode(IdleMode.kCoast);
          motor_right2.setIdleMode(IdleMode.kCoast);
          motor_right3.setIdleMode(IdleMode.kCoast);
        break;
      }
    }
    driverInputMode = c_inputMode();

    double left = 0;
    double right = 0;
    switch (driverInputMode) {
      case TANK:
        left = c_leftthrottle();
        right = c_rightthrottle();
        break;
      case ARCADE:
        left = c_speed()+c_steer();
        right = c_speed()-c_steer();
        break;
      default:
        left = 0;
        right = 0;
        break;
    }
    leftspeed(left);
    rightspeed(right);
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
    switch (motorBrakeMode) {
      case BRAKE:
        SmartDashboard.putString("brake mode", "BRAKE");
        break;
      case COAST:
        SmartDashboard.putString("brake mode", "COAST");
        break;
    }
    switch (driverInputMode) {
      case TANK:
        SmartDashboard.putString("drive mode", "TANK");
        break;
      case ARCADE:
        SmartDashboard.putString("drive mode", "ARCADE");
        break;
    }
    SmartDashboard.putNumber("odo left raw", odoleft());
    SmartDashboard.putNumber("odo right raw", odoright());
    SmartDashboard.putNumber("db distance", distance());
    SmartDashboard.putNumber("db direction", direction());
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
    if (control.getAButton()) {
      test_speed = control.getRightTriggerAxis() - control.getLeftTriggerAxis();
    }
    if (control.getBackButton()) {
      test_runleft = false;
      test_runright = false;
    }
    if (control.getLeftBumper()) {
      test_runleft = true;
    }
    if (control.getRightBumper()) {
      test_runright = true;
    }
    if (test_runleft) {
      leftspeed(test_speed);
    } else {
      leftspeed(0);
    }
    if (test_runright) {
      rightspeed(test_speed);
    } else {
      rightspeed(0);
    }
  }

// end of Drivebase class
}