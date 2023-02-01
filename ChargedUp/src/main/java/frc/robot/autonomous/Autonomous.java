
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

private static final String kDefault = "Default"; // 0
private static final String kCustom = "Custom";
private static final String kLeave = "Leave"; // 1
private static final String kScoreLeave = "Score & Leave"; // 2
private static final String kScoreLeaveScoresamerow = "Score & Leave & Score same row"; // 3
private static final String kScoreLeaveScoredifferent = "Score & Leave & Score different row"; // 4
private static final String kScoreDocked = "Score & Docked"; // 5
private static final String kScoreLeaveDocked = "Score & Leave & Docked"; // 6
private static final String kScoreLeaveScoreDocked = "Score & Leave & Score & Docked"; // 7
private static final String kScoreLeaveScoreScoresamerow = "Score & Leave & Score & Score same row"; // 8
private static final String kScoreLeaveScoreScoredifferent = "Score & Leave & Score & Score differnt row"; // 9

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
//  ####    ###   #   #  #####   ###   #   #  #####   ####
//  #   #  #   #  #   #    #      #    ##  #  #      #
//  ####   #   #  #   #    #      #    # # #  ####    ###
//  #   #  #   #  #   #    #      #    #  ##  #          #
//  #   #   ###    ###     #     ###   #   #  #####  ####
//
//  public functions for autonomous input

private void doDefault() {
  // do nothing
}

private void doCustom() {
  switch (state) {
    case "Start":
      drivebase.drive(-60);
      setstate("Driving");
      break;
   case "Driving":
      if (drivebase.stopped()) {
        setstate("End");
      }
      break;
    case "End":
      break;
    default:
      setstate ("End");
      break;  
  }
}

private void doLeave() {

}

private void doScoreLeave() {

}

private void doScoreLeaveScoresamerow() {

}

private void doScoreLeaveScoredifferent() {

}

private void doScoreDocked() {

}

private void doScoreLeaveDocked() {

}

private void doScoreLeaveScoreDocked() {

}

private void doScoreLeaveScoreScoresamerow() {

}

private void doScoreLeaveScoreScoredifferent() {

}

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
    m_chooser.setDefaultOption("Default Auto", kDefault);
    m_chooser.addOption("Custom Auto", kCustom);
    m_chooser.addOption(kLeave,kLeave);
    m_chooser.addOption(kScoreLeave,kScoreLeave);
    m_chooser.addOption(kScoreLeaveScoresamerow,kScoreLeaveScoreScoresamerow);
    m_chooser.addOption(kScoreLeaveScoredifferent,kScoreLeaveScoredifferent);
    m_chooser.addOption(kScoreDocked,kScoreDocked);
    m_chooser.addOption(kScoreLeaveDocked,kScoreLeaveDocked);
    m_chooser.addOption(kScoreLeaveScoreDocked,kScoreLeaveScoreDocked);
    m_chooser.addOption(kScoreLeaveScoreScoresamerow,kScoreLeaveScoresamerow);
    m_chooser.addOption(kScoreLeaveScoreScoredifferent,kScoreLeaveScoreScoredifferent);
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
        case kCustom:
          doCustom();
          // Put custom auto code here
          break;
        case kLeave:
          doLeave();
          break;
        case kScoreLeave:
          doScoreLeave();
          break;
        case kScoreLeaveScoresamerow:
          doScoreLeaveScoresamerow();
          break;
        case kScoreLeaveScoredifferent:
          doScoreLeaveScoredifferent();
          break;
        case kScoreDocked:
          doScoreDocked();
          break;
        case kScoreLeaveDocked:
          doScoreLeaveDocked();
          break;
        case kScoreLeaveScoreDocked:
          doScoreLeaveScoreDocked();
          break;
        case kScoreLeaveScoreScoresamerow:
          doScoreLeaveScoreScoresamerow();
          break;
        case kScoreLeaveScoreScoredifferent:
          doScoreLeaveScoreScoredifferent();
          break;
        case kDefault:
        default:
          // Put default auto code here
          doDefault();
          break;
      }
  }

// end of Autonomous class
}
