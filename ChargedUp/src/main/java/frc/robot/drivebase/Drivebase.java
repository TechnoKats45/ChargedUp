// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//  ####   ####    ###   #   #  #####  ####    ###    ####  #####
//  #   #  #   #    #    #   #  #      #   #  #   #  #      #
//  #   #  ####     #    #   #  ####   ####   #####   ###   ####
//  #   #  #   #    #     # #   #      #   #  #   #      #  #
//  ####   #   #   ###     #    #####  ####   #   #  ####   #####
//
//  Drivebase subsystem
//
//  left side and right side motors
//  NavX accelerometer and "gyro" compass
//
package frc.robot.drivebase;

//  driver control is an Xbox gamepad
import edu.wpi.first.wpilibj.XboxController;

//  telemetry goes to the Dashboard for display
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//  drive motors are NEO brushless motors using SPARK MAX speed controllers  
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//  NavX is plugged in to the roboRIO expansion port and uses SPI communication
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

//  auto drive and steering uses PID control
import edu.wpi.first.math.controller.PIDController;

//  configuration constants (PID gains, motor IDs, etc) are in the Config.java file
import frc.robot.config.*;


public class Drivebase {

//   ###   #       ###   ####    ###   #       ####  
//  #      #      #   #  #   #  #   #  #      #      
//  #  ##  #      #   #  ####   #####  #       ###   
//  #   #  #      #   #  #   #  #   #  #          #  
//   ###   #####   ###   ####   #   #  #####  ####   
//
//  global variables

    //  driver control is an Xbox gamepad
    XboxController control;

    //  three motors for each side of the drivebase
    CANSparkMax motor_left1;
    CANSparkMax motor_left2;
    CANSparkMax motor_left3;
    CANSparkMax motor_right1;
    CANSparkMax motor_right2;
    CANSparkMax motor_right3;

    //  configuration constants 
    Config config = new Config();

    //  position/direction tracking
    double odometerOrigin = 0;
    double directionOrigin = 0;

    //  motor speed controllers can be configured either for BRAKE or COAST mode when at neutral
    enum BrakeModeType {BRAKE, COAST}
    BrakeModeType motorBrakeMode = BrakeModeType.BRAKE;
    //  driver can use either TANK control (individual left and right throttles) or ARCADE control (separate throttle and steer)
    enum DriverInputModeType {TANK, ARCADE}
    DriverInputModeType driverInputMode = DriverInputModeType.ARCADE;
    
    //  auto driving will control motors to reach the target position
    int autodrive_target = 0;
    boolean autodrive_active = false;
    //  auto steering will control motors to turn to the target direction
    int autoturn_target = 0;
    boolean autoturn_active = false;

    //  auto driving and steering uses PID control
    PIDController drive_pid = new PIDController(config.kP_drive, config.kI_drive, config.kD_drive);
    PIDController turn_pid = new PIDController(config.kP_turn, config.kI_turn, config.kD_turn);
    
    //  NavX accelerometer and compass information comes from the Attitude and Heading Reference System
    AHRS navx;
  

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

// update drive pid gains
private void c_update_drive_pid() {
  double kP_drive = SmartDashboard.getNumber("kP drive",config.kP_drive);
  double kI_drive = SmartDashboard.getNumber("kI drive",config.kI_drive);
  double kD_drive = SmartDashboard.getNumber("kD drive",config.kD_drive);
  if (kP_drive != drive_pid.getP()) {
    drive_pid.setP(kP_drive);
  }
  if (kI_drive != drive_pid.getI()) {
    drive_pid.setI(kI_drive);
  }
  if (kD_drive != drive_pid.getD()) {
    drive_pid.setD(kD_drive);
  }
}

// update turn pid gains
private void c_update_turn_pid() {
  double kP_turn = SmartDashboard.getNumber("kP turn",config.kP_turn);
  double kI_turn = SmartDashboard.getNumber("kI turn",config.kI_turn);
  double kD_turn = SmartDashboard.getNumber("kD turn",config.kD_turn);
  if (kP_turn != turn_pid.getP()) {
    turn_pid.setP(kP_turn);
  }
  if (kI_turn != turn_pid.getI()) {
    turn_pid.setI(kI_turn);
  }
  if (kD_turn != turn_pid.getD()) {
    turn_pid.setD(kD_turn);
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

  // set all three left side motors
  private void leftspeed(double speed) {
    SmartDashboard.putNumber("drive/left speed", speed);
    motor_left1.set(speed);
    motor_left2.set(speed);
    motor_left3.set(speed);
  }

  // set all three right side motors
  private void rightspeed(double speed) {
    SmartDashboard.putNumber("drive/right speed:", speed);
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
    return motor_right1.getEncoder().getPosition();
  }

  // read the number of inches traveled since the last reset
  private double distance() {
    // raw robot odometer is average of left and right side raw values
    double odoRaw = (odoleft()+odoright())/2.0;
    return (odoRaw - odometerOrigin) * config.kk_WheelInchesPerCount;
  }
  // reset the odometer to zero
  private void distanceReset() {
    odometerOrigin = (odoleft()+odoright())/2.0;
  }

  // read the number of degrees turned since the last reset
  private double direction() {
    // old way:
    // degrees per count is computed from robot wheel size, wheelbase, gearbox ratio, and encoder counts per motor revolution
    // raw robot direction is difference between left and right side raw values
    // double dirRaw = (odoleft()-odoright());
    // return (dirRaw - directionOrigin) * config.kk_TurnDegreesPerCount;
    // new way:
    // read the value directly from the NavX
    return navx.getAngle() - directionOrigin;
  }
// reset the direction to zero
  private void directionReset() {
    // old way:
    // directionOrigin = (odoleft()-odoright());
    // new way:
    directionOrigin = navx.getAngle();
  }

  private double tilt() {
    // compute tilt from accelerometer axes
    // robot is rotated from navx default
    double x = navx.getPitch();
    SmartDashboard.putNumber("attitude/roll", x);
    double y = navx.getRoll();
    SmartDashboard.putNumber("attitude/pitch", y);
    // up to a certain value, 
    // tilt is just the hypotenuse of the right triangle having pitch and roll as the legs
    return Math.sqrt(x*x + y*y);
  }


//
//   ###   #   #  #####   ###   
//  #   #  #   #    #    #   #  
//  #####  #   #    #    #   #  
//  #   #  #   #    #    #   #  
//  #   #   ###     #     ###   
//
//  public functions for autonomous control

  // drive straight forward or backward to a specified target distance
  public void auto_drive(int distance) {
    SmartDashboard.putNumber("drive/target", distance);
    distanceReset();
    drive_pid.reset();
    drive_pid.setSetpoint(distance);
    drive_pid.setTolerance(1);
    autodrive_target = distance;
    autodrive_active = true;
  }

  // turn in place left or right to a specified target angle
  public void auto_turn(int direction) {
    SmartDashboard.putNumber("turn/target", direction);
    directionReset();
    turn_pid.reset();
    turn_pid.setSetpoint(direction);
    turn_pid.setTolerance(1);
    autoturn_target = direction;
    autoturn_active = true;
  }

  public boolean stopped() {
    return !autodrive_active;
  }

//
// utility functions

// create a new motor controller for a NEO brushless motor
  private CANSparkMax newNEO(int canID, boolean isInverted) {
    CANSparkMax motor = new CANSparkMax(canID, MotorType.kBrushless);
    motor.setIdleMode(IdleMode.kBrake);
    motor.setInverted(isInverted);
    motor.burnFlash();
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

    navx = new AHRS(SPI.Port.kMXP);

    motor_left1 = newNEO(config.kmc_left1, config.kk_leftinvert);
    motor_left2 = newNEO(config.kmc_left2, config.kk_leftinvert);
    motor_left3 = newNEO(config.kmc_left3, config.kk_leftinvert);
    motor_right1 = newNEO(config.kmc_right1, config.kk_rightinvert);
    motor_right2 = newNEO(config.kmc_right2, config.kk_rightinvert);
    motor_right3 = newNEO(config.kmc_right3, config.kk_rightinvert);

    distanceReset();
    directionReset();
    
    SmartDashboard.putNumber("drive/kP",drive_pid.getP());
    SmartDashboard.putNumber("drive/kI",drive_pid.getI());
    SmartDashboard.putNumber("drive/kD",drive_pid.getD());
    SmartDashboard.putNumber("turn/kP",turn_pid.getP());
    SmartDashboard.putNumber("turn/kI",turn_pid.getI());
    SmartDashboard.putNumber("turn/kD",turn_pid.getD());

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
    if ((left != 0) | (right != 0)) {
      autodrive_active = false;
      autoturn_active = false;
    }
    if (autodrive_active) {
      double throttle = drive_pid.calculate(distance(), autodrive_target);
      if (drive_pid.atSetpoint()) {
        autodrive_active = false;
      }
      left = throttle;
      right = throttle;
    }
    if (autoturn_active) {
      double rudder = turn_pid.calculate(direction(), autoturn_target);
      if (turn_pid.atSetpoint()) {
        autoturn_active = false;
      }
      left = left + rudder;
      right = right - rudder;
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
    if (control.getLeftStickButton()) {
      distanceReset();
    }
    if (control.getRightStickButton()) {
      directionReset();
    }
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
    switch (motorBrakeMode) {
      case BRAKE:
        SmartDashboard.putString("brake mode", "BRAKE");
        break;
      case COAST:
        SmartDashboard.putString("brake mode", "COAST");
        break;
    }
    
    driverInputMode = c_inputMode();
    switch (driverInputMode) {
      case TANK:
        SmartDashboard.putString("drive mode", "TANK");
        break;
      case ARCADE:
        SmartDashboard.putString("drive mode", "ARCADE");
        break;
    }
    SmartDashboard.putNumber("drive/odo left raw", odoleft());
    SmartDashboard.putNumber("drive/odo right raw", odoright());
    SmartDashboard.putNumber("drive/distance", distance());
    SmartDashboard.putNumber("turn/direction", direction());
    
    SmartDashboard.putBoolean("drive/auto", autodrive_active);
    SmartDashboard.putBoolean("turn/auto", autoturn_active);

    SmartDashboard.putNumber("attitude/tilt", tilt());
  }


// 
//  #####  #####   ####  #####
//    #    #      #        #
//    #    ####    ###     #
//    #    #          #    #
//    #    #####  ####     #
//
//  provides special support for testing individual subsystem functionality

  // to exercise auto_drive and auto_turn functions:
  // while holding down POV control, press and release the START button
  public void test() {
    if (control.getStartButtonPressed()) {
      switch (control.getPOV()) {
        case 0: auto_drive(48); break;
        case 90: auto_turn(90); break;
        case 180: auto_drive(-48); break;
        case 270: auto_turn(-90); break;
      }
    }
    c_update_drive_pid();
    c_update_turn_pid();
    run();
  }

/* run motors without constant driver joystick command
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
*/

// end of Drivebase class
}