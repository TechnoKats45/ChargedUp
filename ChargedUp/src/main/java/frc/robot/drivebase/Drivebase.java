
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

    CANSparkMax leftmotor1;
    CANSparkMax leftmotor2;
    CANSparkMax leftmotor3;
    CANSparkMax rightmotor1;
    CANSparkMax rightmotor2;
    CANSparkMax rightmotor3;

    Config config = new Config();

    // driving
    double odometerOrigin = 0;
    double directionOrigin = 0;
    enum BrakeModeType {BRAKE, COAST}
    BrakeModeType motorBrakeMode = BrakeModeType.BRAKE;
    enum DriverInputModeType {TANK, ARCADE}
    DriverInputModeType driverInputMode = DriverInputModeType.ARCADE;
  

    // test mode //
    boolean runleft = false;
    boolean runright = false;
    double testspeed = 0;
    // test mode //
//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

// tank control (individual left and right throttles)
  private double c_leftthrottle() {
    double val = -control.getLeftY();
    if ((-0.2 < val) && (val < 0.2)) {
      val = 0;
    }
    return val;
  }
  private double c_rightthrottle() {
    double val = -control.getRightY();
    if ((-0.2 < val) && (val < 0.2)) {
      val = 0;
    }
    return val;  
  }

// split arcade control (speed on left, steer on right)
  private double c_speed() {
    double val = -control.getLeftY();
    if ((-0.2 < val) && (val < 0.2)) {
      val = 0;
    } 
    return val;
  }
  private double c_steer() {
    double val = control.getRightX();
    if ((-0.2 < val) && (val < 0.2)) {
      val = 0;
    } 
    return val/5.0;
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
    leftmotor1.set(speed);
    leftmotor2.set(speed);
    leftmotor3.set(speed);
  }

  private void rightspeed(double speed) {
    rightmotor1.set(speed);
    rightmotor2.set(speed);
    rightmotor3.set(speed);

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
    return leftmotor1.getEncoder().getPosition();
  }

  // read accumulated right side motor position
  private double odoright() {
    return -rightmotor1.getEncoder().getPosition();
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
//  public functions for autonomous input

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

    leftmotor1 = newNEO(config.kmc_left1, config.kk_leftinvert);
    leftmotor2 = newNEO(config.kmc_left2, config.kk_leftinvert);
    leftmotor3 = newNEO(config.kmc_left3, config.kk_leftinvert);
    rightmotor1 = newNEO(config.kmc_right1, config.kk_rightinvert);
    rightmotor2 = newNEO(config.kmc_right2, config.kk_rightinvert);
    rightmotor3 = newNEO(config.kmc_right3, config.kk_rightinvert);
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
          leftmotor1.setIdleMode(IdleMode.kBrake);
          leftmotor2.setIdleMode(IdleMode.kBrake);
          leftmotor3.setIdleMode(IdleMode.kBrake);
          rightmotor1.setIdleMode(IdleMode.kBrake);
          rightmotor2.setIdleMode(IdleMode.kBrake);
          rightmotor3.setIdleMode(IdleMode.kBrake);
          break;
        case COAST:
          leftmotor1.setIdleMode(IdleMode.kCoast);
          leftmotor2.setIdleMode(IdleMode.kCoast);
          leftmotor3.setIdleMode(IdleMode.kCoast);
          rightmotor1.setIdleMode(IdleMode.kCoast);
          rightmotor2.setIdleMode(IdleMode.kCoast);
          rightmotor3.setIdleMode(IdleMode.kCoast);
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
    SmartDashboard.putNumber("left speed", left);
    SmartDashboard.putNumber("right speed:", right);
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
      testspeed = control.getRightTriggerAxis() - control.getLeftTriggerAxis();
    }
    if (control.getBackButton()) {
      runleft = false;
      runright = false;
    }
    if (control.getLeftBumper()) {
      runleft = true;
    }
    if (control.getRightBumper()) {
      runright = true;
    }
    if (runleft) {
      leftspeed(testspeed);
    } else {
      leftspeed(0);
    }
    if (runright) {
      rightspeed(testspeed);
    } else {
      rightspeed(0);
    }
  }

// end of Drivebase class
}