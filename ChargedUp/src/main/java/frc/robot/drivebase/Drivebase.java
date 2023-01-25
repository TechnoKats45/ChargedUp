
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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


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


//
//   ###    ###   #####  #   #   ###   #####  #####  
//  #   #  #   #    #    #   #  #   #    #    #      
//  #####  #        #    #   #  #####    #    ####   
//  #   #  #   #    #    #   #  #   #    #    #      
//  #   #   ###     #     ###   #   #    #    #####  
//
//  private functions for motor/pneumatic/servo output


//
//   ####  #####  #   #   ####  #####  
//  #      #      ##  #  #      #      
//   ###   ####   # # #   ###   ####   
//      #  #      #  ##      #  #      
//  ####   #####  #   #  ####   #####  
//
//  private functions for sensor feedback


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

  public Drivebase(XboxController driverControl) {
    // initialize
    control = driverControl;
    leftmotor1 = new CANSparkMax(1, MotorType.kBrushless);
    leftmotor2 = new CANSparkMax(3, MotorType.kBrushless);
    leftmotor3 = new CANSparkMax(5, MotorType.kBrushless);
    rightmotor1 = new CANSparkMax(2, MotorType.kBrushless);
    rightmotor2 = new CANSparkMax(4, MotorType.kBrushless);
    rightmotor3 = new CANSparkMax(6, MotorType.kBrushless);
    rightmotor1.setInverted(true);
    rightmotor2.setInverted(true);
    rightmotor3.setInverted(true);
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
    double left = control.getLeftY();
    double right = control.getRightY();
    leftmotor1.set(left);
    leftmotor2.set(left);
    rightmotor1.set(right);
    rightmotor2.set(right);
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
      leftmotor1.set(testspeed);
      leftmotor2.set(testspeed);  
      leftmotor3.set(testspeed);
    } else {
      leftmotor1.set(0);
      leftmotor2.set(0);  
      leftmotor3.set(0);
    }
    if (runright) {
      rightmotor1.set(testspeed);
      rightmotor2.set(testspeed);  
      rightmotor3.set(testspeed);
    } else {
      rightmotor1.set(0);
      rightmotor2.set(0);  
      rightmotor3.set(0);
    }
  }

// end of Drivebase class
}
