
// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//   ###   ####   #   #
//  #   #  #   #  ## ##
//  #####  ####   # # #
//  #   #  #   #  #   #
//  #   #  #   #  #   #
//
//  Arm subsystem
//
//  shoulder elevation motor with limit switches and analog position measurement connected via motor controller
//  arm extension motor with limit switches and analog position measurement connected via motor controller
//  arm slide motor with limit switches and analog position measurement connected via roboRIO ports
//
package frc.robot.arm;

//  operator control is an ATTACK 3 joystick
import edu.wpi.first.wpilibj.Joystick;

//  telemetry goes to the Dashboard for display
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//  elevation and extension are NEO brushless motors using SPARK MAX speed controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//  elevation and extension limit switches are connected to the SPARK MAX
import com.revrobotics.SparkMaxLimitSwitch;
//  elevation and extension position feedback are connected to the SPARK MAX
import com.revrobotics.SparkMaxAnalogSensor;

import edu.wpi.first.math.controller.PIDController;

//  slide is a snowblower motor using a SPARK speed controller
import edu.wpi.first.wpilibj.motorcontrol.Spark;
//  slide limit switches are connected to the roboRIO
import edu.wpi.first.wpilibj.DigitalInput;
//  slide position feedback is an analog voltage connected to the roboRIO
import edu.wpi.first.wpilibj.AnalogPotentiometer;

//  configuration constants (PID gains, motor IDs, etc) are in the Config.java file
import frc.robot.config.*;


public class Arm {

//   ###   #       ###   ####    ###   #       ####  
//  #      #      #   #  #   #  #   #  #      #      
//  #  ##  #      #   #  ####   #####  #       ###   
//  #   #  #      #   #  #   #  #   #  #          #  
//   ###   #####   ###   ####   #   #  #####  ####   
//
//  global variables

//  operator control is an ATTACK 3 joystick
Joystick control;

CANSparkMax elevationmotor;
Spark slidemotor;
CANSparkMax extensionmotor;

private Config config = new Config();

private SparkMaxLimitSwitch elevation_forwardLimit;
private SparkMaxLimitSwitch elevation_reverseLimit;
private SparkMaxAnalogSensor elevation_sense;
private SparkMaxLimitSwitch extension_nearLimit;
private SparkMaxLimitSwitch extension_farLimit;
private SparkMaxAnalogSensor extension_sense;
private DigitalInput slide_leftLimit;
private DigitalInput slide_rightLimit;
private AnalogPotentiometer slide_sense;

double angleOrigin = 0;
double inchesOrigin = 0;
double slideOrigin = 0;

PIDController elevation_pid = new PIDController(config.kP_elevation, config.kI_elevation, config.kD_elevation);
PIDController slide_pid = new PIDController(config.kP_slide, config.kI_slide, config.kD_slide);
PIDController extension_pid = new PIDController(config.kP_extension, config.kI_extension, config.kD_extension);

double elevation_target = 0;
double slide_target = 0;
double extension_target = 0;

//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

boolean c_extend() { // extend arm
    return control.getRawButton(config.kj_up);
}

boolean c_retract() { // retract arm
    return control.getRawButton(config.kj_down);
}

boolean c_left() { // slide arm left
    return control.getRawButton(config.kj_left);
}

boolean c_right() { // slide arm right
    return control.getRawButton(config.kj_right);
}

double c_elevate() { // elevation motor control
    return control.getY();
}

boolean c_enable() { // enable manual arm control
    return control.getRawButton(config.kj_trig);
}

boolean c_resetangle() { // set elevation zero location (ideally straight up)
    return control.getRawButton(config.kj_leftfar);
}

boolean c_resetextension() { // set extension zero location (ideally fully retracted)
    return control.getRawButton(config.kj_leftnear);
}

boolean c_resetslide() { // set slide zero location (ideally centered)
    return control.getRawButton(config.kj_rightnear);
}

double c_throttle() { // scaled to be a value from 0 to 1
    return (control.getThrottle() + 1) / 2.0;
} 

  // update elevation pid gains
  private void c_update_elevation_pid() {
    double kP_elevation = SmartDashboard.getNumber("elevation/kP",config.kP_elevation);
    double kI_elevation = SmartDashboard.getNumber("elevation/kI",config.kI_elevation);
    double kD_elevation = SmartDashboard.getNumber("elevation/kD",config.kD_elevation);
    if (kP_elevation != elevation_pid.getP()) {
      elevation_pid.setP(kP_elevation);
    }
    if (kI_elevation != elevation_pid.getI()) {
      elevation_pid.setI(kI_elevation);
    }
    if (kD_elevation != elevation_pid.getD()) {
      elevation_pid.setD(kD_elevation);
    }
  }

  // update extension pid gains
  private void c_update_extension_pid() {
    double kP_extension = SmartDashboard.getNumber("extension/kP",config.kP_extension);
    double kI_extension = SmartDashboard.getNumber("extension/kI",config.kI_extension);
    double kD_extension = SmartDashboard.getNumber("extension/kD",config.kD_extension);
    if (kP_extension != extension_pid.getP()) {
      extension_pid.setP(kP_extension);
    }
    if (kI_extension != extension_pid.getI()) {
      extension_pid.setI(kI_extension);
    }
    if (kD_extension != extension_pid.getD()) {
      extension_pid.setD(kD_extension);
    }
  }

  // update slide pid gains
  private void c_update_slide_pid() {
    double kP_slide = SmartDashboard.getNumber("slide/kP",config.kP_slide);
    double kI_slide = SmartDashboard.getNumber("slide/kI",config.kI_slide);
    double kD_slide = SmartDashboard.getNumber("slide/kD",config.kD_slide);
    if (kP_slide != slide_pid.getP()) {
      slide_pid.setP(kP_slide);
    }
    if (kI_slide != slide_pid.getI()) {
      slide_pid.setI(kI_slide);
    }
    if (kD_slide != slide_pid.getD()) {
      slide_pid.setD(kD_slide);
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

void elevate(double speed) {
    SmartDashboard.putNumber("elevation/speed", speed);
    elevationmotor.set(speed);
}
void extend(double speed) {
    SmartDashboard.putNumber("extension/speed", speed);
    extensionmotor.set(speed);
}
void slide(double speed) {
    SmartDashboard.putNumber("slide/speed", speed);
    slidemotor.set(speed);
}



//
//   ####  #####  #   #   ####  #####  
//  #      #      ##  #  #      #      
//   ###   ####   # # #   ###   ####   
//      #  #      #  ##      #  #      
//  ####   #####  #   #  ####   #####  
//
//  private functions for sensor feedback

private double elevationangle() {
    double angleRaw = elevationmotor.getEncoder().getPosition();
    return (angleRaw - angleOrigin) * config.kk_ArmDegreesPerCount;
}

private void anglereset(double value) {
    angleOrigin = elevationmotor.getEncoder().getPosition() + value/config.kk_ArmDegreesPerCount;
}

private double extensioninches() {
    double inchesRaw = extensionmotor.getEncoder().getPosition();
    return (inchesRaw - inchesOrigin) * config.kk_ExtensionInchesPerCount;
}

private void inchesreset(double value) {
    inchesOrigin = extensionmotor.getEncoder().getPosition() + value/config.kk_ExtensionInchesPerCount;
}

private double slideinches() {
    double slideRaw = slide_sense.get();
    return (slideRaw - slideOrigin) * config.kk_SlideInchesPerCount;
}

private void slidereset(double value) {
    slideOrigin = slide_sense.get() + value/config.kk_SlideInchesPerCount;
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
//   ###    ###   #   #   ####  #####  ####   #   #   ###   #####   ###   ####
//  #   #  #   #  ##  #  #        #    #   #  #   #  #   #    #    #   #  #   #
//  #      #   #  # # #   ###     #    ####   #   #  #        #    #   #  ####
//  #   #  #   #  #  ##      #    #    #   #  #   #  #   #    #    #   #  #   #
//   ###    ###   #   #  ####     #    #   #   ###    ###     #     ###   #   #
//
//  creates a new Subsystem 

  public Arm(Joystick userController) {
    // initialize
    control = userController;
    elevationmotor = new CANSparkMax(config.kmc_elevate, MotorType.kBrushless);
    elevationmotor.setInverted(true);
    elevationmotor.setIdleMode(IdleMode.kBrake);
    elevation_forwardLimit = elevationmotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    elevation_reverseLimit = elevationmotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    elevation_sense = elevationmotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    anglereset(0);

    slidemotor = new Spark(config.kmp_slide);
    slide_leftLimit = new DigitalInput(config.kdi_slideleft);
    slide_rightLimit = new DigitalInput(config.kdi_slideright);
    slide_sense = new AnalogPotentiometer(config.kai_slide);

    extensionmotor = new CANSparkMax(config.kmc_extend, MotorType.kBrushless);
    extensionmotor.setIdleMode(IdleMode.kCoast);
    extension_farLimit = extensionmotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extension_nearLimit = extensionmotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extension_sense = extensionmotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    inchesreset(0);

    SmartDashboard.putNumber("elevation/kP",elevation_pid.getP());
    SmartDashboard.putNumber("elevation/kI",elevation_pid.getI());
    SmartDashboard.putNumber("elevation/kD",elevation_pid.getD());

    SmartDashboard.putNumber("extension/kP",extension_pid.getP());
    SmartDashboard.putNumber("extension/kI",extension_pid.getI());
    SmartDashboard.putNumber("extension/kD",extension_pid.getD());

    
  }


//
//  ####   #   #  #   #
//  #   #  #   #  ##  #
//  ####   #   #  # # #
//  #   #  #   #  #  ##
//  #   #   ###   #   #
//
//  does everything necessary when the robot is enabled, either autonomous or teleoperated

  // arm shoulder elevation 
  private void run_elevation() {
    if (c_resetangle()) {
        anglereset(0);
    }
    double elevate = elevation_pid.calculate(elevationangle(), elevation_target);
    elevationmotor.set(elevate);
  }
  // arm extension
  private void run_extension() {
    if (c_resetextension()) {
        inchesreset(0); 
    }
    double extend = extension_pid.calculate(extensioninches(), extension_target);
    extensionmotor.set(extend);
  }
  // arm slide
  private void run_slide() {
    if (c_resetslide()) {
        slidereset(0);
    }
    // @@ to do: compute slide target position based on camera tracking of scoring location
    double slide = slide_pid.calculate(slideinches(), slide_target);
    slidemotor.set(slide);
  }
  public void run() {
    run_elevation();
    run_extension();
    run_slide();
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
    SmartDashboard.putNumber("elevation/angle", elevationangle());
    SmartDashboard.putBoolean("elevation/fwd limit", elevation_forwardLimit.isPressed());
    SmartDashboard.putBoolean("elevation/rev limit", elevation_reverseLimit.isPressed());
    SmartDashboard.putNumber("elevation/sense", elevation_sense.getPosition());

    SmartDashboard.putNumber("extension/inches", extensioninches());
    SmartDashboard.putBoolean("extension/near limit", extension_nearLimit.isPressed());
    SmartDashboard.putBoolean("extension/far limit", extension_farLimit.isPressed());
    SmartDashboard.putNumber("extension/sense", extension_sense.getPosition());

    SmartDashboard.putNumber("slide/inches", slideinches());
    SmartDashboard.putBoolean("slide/left limit", slide_leftLimit.get());
    SmartDashboard.putBoolean("slide/right limit", slide_rightLimit.get());
    SmartDashboard.putNumber("slide/sense", slide_sense.get());
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
    if (!c_enable()) {
        SmartDashboard.putBoolean("arm test enable", true);

        if (c_extend()) {
            extend(0.5);
        } else if (c_retract()) {
            extend(-0.5);
        } else {
            extend(0);
        }
        if (c_left()) {
            slide(0.5);
        } else if (c_right()) {
            slide(-0.5);
        } else {
            slide(0);
        }
        elevate(-c_elevate());
    }
    
    else{
        SmartDashboard.putBoolean("arm test enable", false);
        if (c_left()){
            c_update_elevation_pid();
            elevation_target = c_throttle() * (config.kk_elevationmax - config.kk_elevationmin) + config.kk_elevationmin;
            run_elevation();
        } else {
            elevate(0);  
        }
        if (c_right()){
            c_update_extension_pid();
            extension_target = c_throttle() * (config.kk_extensionmax - config.kk_extensionmin) + config.kk_extensionmin;
            run_extension();
        } else {
            extend(0);
        }
        if (c_retract()){
            c_update_slide_pid();
            slide_target = c_throttle() * (config.kk_slidemax - config.kk_slidemin) + config.kk_slidemin;
            run_slide();
        } else {
            slide(0);
        }
        
        slide(0);
    }


}

// end of Arm class
}