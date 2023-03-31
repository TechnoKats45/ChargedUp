
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

//  elevation and extension use PID control
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;

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

private DigitalInput force_coast;

double angleOrigin = 0;
double inchesOrigin = 0;
double slideOrigin = 0;

PIDController elevation_pid = new PIDController(config.kP_elevation, config.kI_elevation, config.kD_elevation);
PIDController slide_pid = new PIDController(config.kP_slide, config.kI_slide, config.kD_slide);
PIDController extension_pid = new PIDController(config.kP_extension, config.kI_extension, config.kD_extension);
SlewRateLimiter elevation_rate;
SlewRateLimiter extension_rate;

double elevation_target = 0;
double slide_target = 0;
double extension_target = 0;

boolean autoelevate = false;
boolean autoslide = false;
boolean autoextend = false;

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

boolean c_extend() { // extend arm
    return (control.getRawButton(config.kj_up) && extensioninches() < 20);
}

boolean c_retract() { // retract arm
    return (control.getRawButton(config.kj_down) && extensioninches() > 0.5);
}

boolean c_left() { // slide arm left
    return control.getRawButton(config.kj_left);
}

boolean c_right() { // slide arm right
    return control.getRawButton(config.kj_right);
}

double c_elevate() { // elevation motor control
    return deadband(-control.getY(), 0.2);
}

boolean c_enable() { // enable manual arm control
    return control.getRawButton(config.kj_trig);
}

boolean c_resetangle() { // set elevation zero location (ideally straight up)
    return false;
    // return control.getRawButton(config.kj_leftfar);
}

boolean c_resetextension() { // set extension zero location (ideally fully retracted)
    return false;
    // control.getRawButton(config.kj_leftnear);
}

boolean c_resetslide() { // set slide zero location (ideally centered)
    return false;
    // control.getRawButton(config.kj_rightnear);
}

double c_throttle() { // scaled to be a value from 0 to 1
    return (control.getThrottle() + 1) / 2.0;
} 

boolean c_preset0() { // full forward
  return control.getRawButton(config.kj_centerleft);
}
boolean c_preset1() { // floor level
  return control.getRawButton(config.kj_centerright);
}
boolean c_preset2() { // middle node level
  return control.getRawButton(config.kj_rightnear);
}
boolean c_preset3() { // top node level
  return control.getRawButton(config.kj_rightfar);
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
      System.out.print("ext P");
    }
    if (kI_extension != extension_pid.getI()) {
      extension_pid.setI(kI_extension);
      System.out.print("ext I");
    }
    if (kD_extension != extension_pid.getD()) {
      extension_pid.setD(kD_extension);
      System.out.print("ext D");
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
    angleOrigin = elevationmotor.getEncoder().getPosition() - value/config.kk_ArmDegreesPerCount;
}

private double extensioninches() {
    double inchesRaw = extensionmotor.getEncoder().getPosition();
    return (inchesRaw - inchesOrigin) * config.kk_ExtensionInchesPerCount;
}

private void inchesreset(double value) {
    inchesOrigin = extensionmotor.getEncoder().getPosition() - value/config.kk_ExtensionInchesPerCount;
}

private double slideinches() {
    double slideRaw = slide_sense.get();
    return (slideRaw - slideOrigin) * config.kk_SlideInchesPerCount;
}

private void slidereset(double value) {
    slideOrigin = slide_sense.get() - value/config.kk_SlideInchesPerCount;
}

//
//   ###   #   #  #####   ###   
//  #   #  #   #    #    #   #  
//  #####  #   #    #    #   #  
//  #   #  #   #    #    #   #  
//  #   #   ###     #     ###   
//
//  public functions for autonomous input

public boolean elevation_at_target() {
  return elevation_pid.atSetpoint();
}

public boolean extension_at_target() {
  return extension_pid.atSetpoint();
}

public void auto_elevate(double angle) {
  elevation_target = angle;
  autoelevate = true;
}

public void auto_extend(double position) {
  extension_target = position;
  autoextend = true;
}

public void auto_slide(double position) {
  slide_target = position;
  autoslide = true;
}

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
    elevationmotor.setInverted(false);
    elevationmotor.setIdleMode(IdleMode.kBrake);
    elevationmotor.burnFlash();
    elevation_forwardLimit = elevationmotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    elevation_reverseLimit = elevationmotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    elevation_sense = elevationmotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    anglereset(0);
    elevation_rate = new SlewRateLimiter(config.kk_elevate_rate);
    elevation_pid.setTolerance(3);;

    slidemotor = new Spark(config.kmp_slide);
    slide_leftLimit = new DigitalInput(config.kdi_slideleft);
    slide_rightLimit = new DigitalInput(config.kdi_slideright);
    slide_sense = new AnalogPotentiometer(config.kai_slide);

    extensionmotor = new CANSparkMax(config.kmc_extend, MotorType.kBrushless);
    extensionmotor.setInverted(false);
    extensionmotor.setIdleMode(IdleMode.kCoast);
    extensionmotor.burnFlash();
    extension_farLimit = extensionmotor.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extension_nearLimit = extensionmotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
    extension_sense = extensionmotor.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
    inchesreset(0);
    extension_rate = new SlewRateLimiter(config.kk_extend_rate);
    extension_pid.setTolerance(1);

    force_coast = new DigitalInput(config.kdi_forcearmcoast);

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
    if (c_preset0()) {
      auto_elevate(config.kk_elevation_preset0);
    }
    if (c_preset1()) {
      auto_elevate(config.kk_elevation_preset1);
    }
    if (c_preset2()) {
      auto_elevate(config.kk_elevation_preset2);
    }
    if (c_preset3()) {
      auto_elevate(config.kk_elevation_preset3);
    }
    double val = deadband(c_elevate(),0.1);
    if (val != 0) {
      autoelevate = false;
    } else if (!autoelevate) {
      auto_elevate(elevationangle());
    }
    if (autoelevate) {
      val = elevation_pid.calculate(elevationangle(), elevation_target);
    }
    elevate(val);
  }
  // arm extension
  private void run_extension() {
    if (c_resetextension()) {
        inchesreset(0); 
    }
    if (c_preset0()) {
      auto_extend(config.kk_extension_preset0);
    }
    if (c_preset1()) {
      auto_extend(config.kk_extension_preset1);
    }
    if (c_preset2()) {
      auto_extend(config.kk_extension_preset2);
    }
    if (c_preset3()) {
      auto_extend(config.kk_extension_preset3);
    }
    double val = 0;
    if (c_extend()) {
      val = 0.5;
    }
    if (c_retract()) {
      val = -0.5;
    }
    if (val != 0) {
      autoextend = false;
    } else if (!autoextend) {
      auto_extend(extensioninches());
    }
    bounding();
    if (autoextend) {
      val = extension_pid.calculate(extensioninches(), extension_target);
    }
    extend(val);
  }
  // arm slide
  private void run_slide() {
    if (c_resetslide()) {
        slidereset(0);
    }
    double val = 0;
    if (c_left()) {
      val = 0.5;
      autoslide = false;
    }
    if (c_right()) {
      val = -0.5;
      autoslide = false;
    }
    if (autoslide) {
      // @@ to do: compute slide target position based on camera tracking of scoring location
      val = slide_pid.calculate(slideinches(), slide_target);
    }
    slide(val);
  }
  public void run() {
    run_elevation();
    run_extension();
    run_slide();
  } 

  private void bounding() {
    double elevationRadians = (elevationangle()/180) * Math.PI + Math.PI/2;
    if ((Math.sin(elevationRadians) * (extensioninches() + config.kpc_armlength)) > config.kpc_maxheight) {
      auto_extend(extensioninches() - 1);
    }
    else if (elevationRadians < (Math.PI / 2) && (Math.cos(elevationRadians)*(extensioninches() + config.kpc_armlength)) > config.kpc_maxlength) {
      auto_extend(extensioninches() - 1);
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
    c_update_elevation_pid();
    if (!force_coast.get()) {
      elevationmotor.setIdleMode(IdleMode.kCoast);
      if (config.kk_armGREENmin < elevationangle() && elevationangle() < config.kk_armGREENmax) {
        SmartDashboard.putString("elevation/color", "GREEN");
      } else {
        SmartDashboard.putString("elevation/color", "WHITE");
      }
    } else {
      elevationmotor.setIdleMode(IdleMode.kBrake);
      SmartDashboard.putString("elevation/color", "NONE");
    }

    SmartDashboard.putNumber("elevation/angle", elevationangle());
    SmartDashboard.putNumber("elevation/target", elevation_target);
    SmartDashboard.putBoolean("elevation/fwd limit", elevation_forwardLimit.isPressed());
    SmartDashboard.putBoolean("elevation/rev limit", elevation_reverseLimit.isPressed());
    SmartDashboard.putNumber("elevation/sense", elevation_sense.getPosition());
    SmartDashboard.putBoolean("elevation/auto", autoelevate);

    if (elevation_forwardLimit.isPressed()) {
      anglereset(config.kk_elevationmax);
    }
    if (elevation_reverseLimit.isPressed()) {
      anglereset(config.kk_elevationmin);
    }

    c_update_extension_pid();
    if (!force_coast.get()) {
      extensionmotor.setIdleMode(IdleMode.kCoast);
      extensionmotor.set(0);
    } else {
      extensionmotor.setIdleMode(IdleMode.kBrake);
    }

    SmartDashboard.putNumber("extension/inches", extensioninches());
    SmartDashboard.putNumber("extension/target", extension_target);
    SmartDashboard.putBoolean("extension/near limit", extension_nearLimit.isPressed());
    SmartDashboard.putBoolean("extension/far limit", extension_farLimit.isPressed());
    SmartDashboard.putNumber("extension/sense", extension_sense.getPosition());
    SmartDashboard.putBoolean("extension/auto", autoextend);

    if (extension_nearLimit.isPressed()) {
      inchesreset(config.kk_extensionmin);
    }
    if (extension_farLimit.isPressed()) {
      inchesreset(config.kk_extensionmax);
    }

    SmartDashboard.putNumber("slide/inches", slideinches());
    SmartDashboard.putNumber("slide/target", slide_target);
    SmartDashboard.putBoolean("slide/left limit", !slide_leftLimit.get());
    SmartDashboard.putBoolean("slide/right limit", !slide_rightLimit.get());
    SmartDashboard.putNumber("slide/sense", slide_sense.get());

    if (!slide_leftLimit.get()) {
      slidereset(config.kk_slidemin);
    }
    if (!slide_rightLimit.get()) {
      slidereset(config.kk_slidemax);
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
    if (!c_enable()) {
        SmartDashboard.putBoolean("arm test enable", false);

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
        elevate(c_elevate());
    } else {
        SmartDashboard.putBoolean("arm test enable", true);
        if (c_left()){
            c_update_elevation_pid();
            auto_elevate(c_throttle() * (config.kk_elevationmax - config.kk_elevationmin) + config.kk_elevationmin);
            elevate(elevation_pid.calculate(elevationangle(), elevation_target));
        } else {
            elevate(0);  
        }
        if (c_right()){
            c_update_extension_pid();
            auto_extend(c_throttle() * (config.kk_extensionmax - config.kk_extensionmin) + config.kk_extensionmin);
            extend(extension_pid.calculate(extensioninches(), extension_target));
        } else {
            extend(0);
        }
        if (c_retract()){
            c_update_slide_pid();
            auto_slide(c_throttle() * (config.kk_slidemax - config.kk_slidemin) + config.kk_slidemin);
            slide(slide_pid.calculate(slideinches(), slide_target));
        } else {
            slide(0);
        }
    }


}

// end of Arm class
}