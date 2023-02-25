
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
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

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

  //  pneumatic cylinders for grabber
  DoubleSolenoid grabber;

  //  pneumatic cylinder for rotation
  DoubleSolenoid rotator;

  //  rangefinder to detect game pieces in grabber
  Counter lidar;

  //  configuration constants
  Config config = new Config();

  //  automatic grab mode -- based on gamepiece sensor
  boolean autograb = false;


//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

  //  close grabber claws
  boolean c_grab() {
    return control.getRawButtonPressed(config.kj_leftnear);
  }
  boolean c_grabbing() {
    return control.getRawButton(config.kj_leftnear);
  }

  //  open grabber claws
  boolean c_release() {
    return control.getRawButtonPressed(config.kj_leftfar);
  }
  boolean c_releasing() {
    return control.getRawButton(config.kj_leftfar);
  }

  boolean c_rotateForward() {
    return control.getRawButton(config.kj_centerleft);
  }

  boolean c_rotateReverse() {
    return control.getRawButton(config.kj_centerright);
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
  void grab() {
    grabber.set(kReverse);
    SmartDashboard.putString("grip/state", "GRAB");
  }

  //  open grabber claws
  void release() {
    grabber.set(kForward);
    SmartDashboard.putString("grip/state", "RELEASE");
  }

  //  rotate grabber
  void rotate(boolean val) {
    if (val) {
      rotator.set(kForward);
    } else {
      rotator.set(kReverse);
    }
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

  // true to close claw; false to open claw
  public void auto_grip(boolean value) {
    if (value) {
      grab();
    } else {
      release();
    }
  
  }


//
//   ###    ###   #   #   ####  #####  ####   #   #   ###   #####   ###   ####
//  #   #  #   #  ##  #  #        #    #   #  #   #  #   #    #    #   #  #   #
//  #      #   #  # # #   ###     #    ####   #   #  #        #    #   #  ####
//  #   #  #   #  #  ##      #    #    #   #  #   #  #   #    #    #   #  #   #
//   ###    ###   #   #  ####     #    #   #   ###    ###     #     ###   #   #
//
//  creates a new Gripper 

  public Gripper(Joystick userControl) {
    // initialize

    control = userControl;
    
    grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, config.kpd_grab_in, config.kpd_grab_out);
    rotator = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, config.kpd_rotate_fwd, config.kpd_rotate_rev);

    // Create a new Counter object in two-pulse mode
    lidar = new Counter(Counter.Mode.kSemiperiod);
    // Set up the input channel for the counter
    lidar.setUpSource(config.kdi_gamepiecesense);
    // Set the encoder to count pulse duration from rising edge to falling edge
    lidar.setSemiPeriodMode(true);

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
    if (c_grab()) {
      if (c_releasing()) {
        release();
        autograb = true;
      } else {
        grab();
        autograb = false;
      }
    }
    if (c_release()) {
      if (c_grabbing()) {
        release();
        autograb = true;
      } else {
        release();
        autograb = false;
      }
    }
    if (autograb && (gamepieceInches() < config.kk_grabrange)) {
      grab();
      autograb = false;
    }
    double armAngle = SmartDashboard.getNumber("elevation/angle", 0);
    if (armAngle < -5 ) {
      rotate(true);
    }
    if (armAngle > 5) {
      rotate(false);
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
