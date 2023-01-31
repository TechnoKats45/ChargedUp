
// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//   ###   #   #  #####   ###   #   #   ###   #   #   ###   #   #   ####
//  #   #  #   #    #    #   #  ##  #  #   #  ## ##  #   #  #   #  #
//  #####  #   #    #    #   #  # # #  #   #  # # #  #   #  #   #   ###
//  #   #  #   #    #    #   #  #  ##  #   #  #   #  #   #  #   #      #
//  #   #   ###     #     ###   #   #   ###   #   #   ###    ###   ####
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

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.drivebase.*;
import frc.robot.arm.*;
import frc.robot.gripper.*;

//  imports (controllers, actuators, sensors, communication)

public class Autonomous {

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

private static final String kDefaultAuto = "Default";
private static final String kCustomAuto = "My Auto";

private final SendableChooser<String> m_chooser = new SendableChooser<>();  

private Drivebase drivebase;
private Arm arm;
private Gripper gripper;
private String mode = "";
private String state = "";

private void setstate(String p_state) {
  state = p_state;
  SmartDashboard.putString("auto state",state);
}

//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

/* Autonomous mode does not have any driver or operator controls! */

//
//   ###    ###   #####  #   #   ###   #####  #####  
//  #   #  #   #    #    #   #  #   #    #    #      
//  #####  #        #    #   #  #####    #    ####   
//  #   #  #   #    #    #   #  #   #    #    #      
//  #   #   ###     #     ###   #   #    #    #####  
//
//  private functions for motor/pneumatic/servo output

/* Autonomous mode  */

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

  public Autonomous(Drivebase p_drivebase, Arm p_arm, Gripper p_gripper) {
    // initialize
    drivebase = p_drivebase;
    arm = p_arm;
    gripper = p_gripper;
    // initialize
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  public void init() {
    mode = m_chooser.getSelected();
    SmartDashboard.putString("auto mode", mode);
    System.out.println("Auto selected: " + mode);
    setstate("Start");
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
    SmartDashboard.putString("auto state", mode+" "+state);
    switch (mode) {
        case kCustomAuto:
          // Put custom auto code here
          break;
        case kDefaultAuto:
        default:
          // Put default auto code here
          break;
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
  }

// end of Autonomous class
}
