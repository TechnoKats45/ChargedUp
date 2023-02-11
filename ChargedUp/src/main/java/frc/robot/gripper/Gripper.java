
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

  //  rangefinder to detect game pieces in grabber
  Counter lidar;

  //  configuration constants
  Config config = new Config();


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
    return control.getRawButton(config.kj_leftnear);
  }

  //  open grabber claws
  boolean c_release() {
    return control.getRawButton(config.kj_leftfar);
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
    grabber.set(kForward);
  }

  //  open grabber claws
  void release() {
    grabber.set(kReverse);
  }

//
//   ####  #####  #   #   ####  #####  
//  #      #      ##  #  #      #      
//   ###   ####   # # #   ###   ####   
//      #  #      #  ##      #  #      
//  ####   #####  #   #  ####   #####  
//
//  private functions for sensor feedback

// Pololu Distance Sensor:
// d = (3 mm/4 us)*(t-1000us)
double gamepieceInches() {
  // counter is in Semi-period mode measuring pulse width from rising edge to falling edge
  // returned value is measured pulse width in seconds so needs to be multiplied by 10^6
  // computed value is in millimeters so needs to be divided by 25.4 to return inches
  double pulsewidth = lidar.getPeriod();
  return (3./4.) * ((pulsewidth*1000000.)-1000.) / 25.4;
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
    grabber = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, config.kpd_grab_in, config.kpd_grab_out);
    
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
      grab();
    }
    if (c_release()) {
      release();
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
