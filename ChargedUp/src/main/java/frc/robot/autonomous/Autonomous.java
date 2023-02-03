
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.*;

import frc.robot.drivebase.*;
import frc.robot.arm.*;
import frc.robot.gripper.*;

//  imports (controllers, actuators, sensors, communication)

public class Autonomous {


//   ###   #       ###   ####    ###   #       ####  
//  #      #      #   #  #   #  #   #  #      #      
//  #  ##  #      #   #  ####   #####  #       ###   
//  #   #  #      #   #  #   #  #   #  #          #  
//   ###   #####   ###   ####   #   #  #####  ####   
//
//  global variables

private static final String autoDefault = "Default"; // 0
private static final String autoCustom = "Custom";
private static final String autoLeav = "Leave"; // 1
private static final String autoScorLeav = "Score > Leave"; // 2
private static final String autoScorLeavScorsame = "Score > Leave > Score same row"; // 3
private static final String autoScorLeavScordiff = "Score > Leave > Score different row"; // 4
private static final String autoScorDock = "Score > Docked"; // 5
private static final String autoScorLeavDock = "Score > Leave > Docked"; // 6
private static final String autoScorLeavScorDock = "Score > Leave > Score > Docked"; // 7
private static final String autoScorLeavScorScorsame = "Score > Leave > Score > Score same row"; // 8
private static final String autoScorLeavScorScordiff = "Score > Leave > Score > Score different row"; // 9

private final SendableChooser<String> m_chooser = new SendableChooser<>();  
private Alliance alliance = Alliance.Invalid;
private String mode = "";
private String state = "";

private Drivebase drivebase;
private Arm arm;
private Gripper gripper;

//
//  #####  #   #  #   #   ###   #####   ###    ###   #   #   ####
//  #      #   #  ##  #  #   #    #      #    #   #  ##  #  #
//  ####   #   #  # # #  #        #      #    #   #  # # #   ###
//  #      #   #  #  ##  #   #    #      #    #   #  #  ##      #
//  #       ###   #   #   ###     #     ###    ###   #   #  ####
//
private void setstate(String p_state) {
  state = p_state;
  SmartDashboard.putString("auto state",state);
}

//
//   ###   #   #  #####   ###          ####   #####  #####   ###   #   #  #      #####
//  #   #  #   #    #    #   #         #   #  #      #      #   #  #   #  #        #
//  #####  #   #    #    #   #         #   #  ####   ####   #####  #   #  #        #
//  #   #  #   #    #    #   #         #   #  #      #      #   #  #   #  #        #
//  #   #   ###     #     ###          ####   #####  #      #   #   ###   #####    #
//
// Autonomous: do nothing
private void doDefault() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ###   #   #   ####  #####   ###   #   #
//  #   #  #   #    #    #   #         #   #  #   #  #        #    #   #  ## ##
//  #####  #   #    #    #   #         #      #   #   ###     #    #   #  # # #
//  #   #  #   #    #    #   #         #   #  #   #      #    #    #   #  #   #
//  #   #   ###     #     ###           ###    ###   ####     #     ###   #   #
//
// Autonomous: work in progress
private void doCustom() {
  switch (state) {
    case "Start":
      drivebase.drive(48);
      setstate("Forward");
      break;
   case "Forward":
      if (drivebase.stopped()) {
        drivebase.turn(90);
        setstate("Turn Right");
      }
      break;
    case "Turn Right":
      if (drivebase.stopped()) {
        drivebase.drive(24);
        setstate("Drive Right");
      }
      break;
    case "Drive Right":
      if (drivebase.stopped()) {
        drivebase.turn(90);
        setstate("Turn Back");
      }
      break;
    case "Turn Back":
      if (drivebase.stopped()) {
        drivebase.drive(48);
        setstate("Back");
      }
    case "Back":
      if (drivebase.stopped()) {
        setstate("End");
      }
    case "End":
      break;
    default:
      setstate ("Error");
      break;  
  }
}

//
//   ###   #   #  #####   ###          #      #####
//  #   #  #   #    #    #   #         #      #
//  #####  #   #    #    #   #         #      ####
//  #   #  #   #    #    #   #         #      #
//  #   #   ###     #     ###          #####  #####
//
// Autonomous: Leave
private void doLeav() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}
//
//   ###   #   #  #####   ###           ####   ###          #      #####
//  #   #  #   #    #    #   #         #      #   #         #      #
//  #####  #   #    #    #   #          ###   #             #      ####
//  #   #  #   #    #    #   #             #  #   #         #      #
//  #   #   ###     #     ###          ####    ###          #####  #####
//
// Autonomous: Score, Leave
private void doScorLeav() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          #      #####          ####   ####
//  #   #  #   #    #    #   #         #      #   #         #      #             #      #
//  #####  #   #    #    #   #          ###   #             #      ####           ###    ###
//  #   #  #   #    #    #   #             #  #   #         #      #                 #      #
//  #   #   ###     #     ###          ####    ###          #####  #####         ####   ####
//
// Autonomous: Score, Leave, Score same row
private void doScorLeavScorsame() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          #      #####          ####  ####
//  #   #  #   #    #    #   #         #      #   #         #      #             #      #   #
//  #####  #   #    #    #   #          ###   #             #      ####           ###   #   #
//  #   #  #   #    #    #   #             #  #   #         #      #                 #  #   #
//  #   #   ###     #     ###          ####    ###          #####  #####         ####   ####
//
// Autonomous: Score, Leave, Score different row
private void doScorLeavScordiff() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          ####    ###
//  #   #  #   #    #    #   #         #      #   #         #   #  #   #
//  #####  #   #    #    #   #          ###   #             #   #  #   #
//  #   #  #   #    #    #   #             #  #   #         #   #  #   #
//  #   #   ###     #     ###          ####    ###          ####    ###
//
// Autonomous: Score, Dock
private void doScorDock() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          #      #####         ####    ###
//  #   #  #   #    #    #   #         #      #   #         #      #             #   #  #   #
//  #####  #   #    #    #   #          ###   #             #      ####          #   #  #   #
//  #   #  #   #    #    #   #             #  #   #         #      #             #   #  #   #
//  #   #   ###     #     ###          ####    ###          #####  #####         ####    ###
//
// Autonomous: Score, Leave, Dock
private void doScorLeavDock() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          #      #####          ####   ###          ####    ###
//  #   #  #   #    #    #   #         #      #   #         #      #             #      #   #         #   #  #   #
//  #####  #   #    #    #   #          ###   #             #      ####           ###   #             #   #  #   #
//  #   #  #   #    #    #   #             #  #   #         #      #                 #  #   #         #   #  #   #
//  #   #   ###     #     ###          ####    ###          #####  #####         ####    ###          ####    ###
//
// Autonomous: Score, Leave, Score, Dock
private void doScorLeavScorDock() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          #      #####          ####   ###           ####   ####
//  #   #  #   #    #    #   #         #      #   #         #      #             #      #   #         #      #
//  #####  #   #    #    #   #          ###   #             #      ####           ###   #              ###    ###
//  #   #  #   #    #    #   #             #  #   #         #      #                 #  #   #             #      #
//  #   #   ###     #     ###          ####    ###          #####  #####         ####    ###          ####   ####
//
// Autonomous: Score, Leave, Score, Score same row
private void doScorLeavScorScorsame() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###   #   #  #####   ###           ####   ###          #      #####          ####   ###           ####  ####
//  #   #  #   #    #    #   #         #      #   #         #      #             #      #   #         #      #   #
//  #####  #   #    #    #   #          ###   #             #      ####           ###   #              ###   #   #
//  #   #  #   #    #    #   #             #  #   #         #      #                 #  #   #             #  #   #
//  #   #   ###     #     ###          ####    ###          #####  #####         ####    ###          ####   ####
//
// Autonomous: Score, Leave, Score, Score different row
private void doScorLeavScorScordiff() {
  switch (state) {
    case "Start":
    case "End":
    default:
      setstate ("Error");
      break;
  }
}

//
//   ###    ###   #   #   ####  #####  ####   #   #   ###   #####   ###   ####
//  #   #  #   #  ##  #  #        #    #   #  #   #  #   #    #    #   #  #   #
//  #      #   #  # # #   ###     #    ####   #   #  #        #    #   #  ####
//  #   #  #   #  #  ##      #    #    #   #  #   #  #   #    #    #   #  #   #
//   ###    ###   #   #  ####     #    #   #   ###    ###     #     ###   #   #
//
//  creates the Autonomous object 

// auto routine chooser: add option to list of choices and set it as default
  private void menuOptionDefault (String option) {
    m_chooser.setDefaultOption(option, option);
  }
// auto routine chooser: add option to list of choices
  private void menuOption (String option) {
    m_chooser.addOption(option, option);
  }

  public Autonomous(Drivebase p_drivebase, Arm p_arm, Gripper p_gripper) {
    // subsystems
    drivebase = p_drivebase;
    arm = p_arm;
    gripper = p_gripper;

    // populate auto routine chooser
    menuOptionDefault(autoDefault);
    menuOption(autoCustom);
    menuOption(autoLeav);
    menuOption(autoScorLeav);
    menuOption(autoScorLeavScorsame);
    menuOption(autoScorLeavScordiff);
    menuOption(autoScorDock);
    menuOption(autoScorLeavDock);
    menuOption(autoScorLeavScorDock);
    menuOption(autoScorLeavScorsame);
    menuOption(autoScorLeavScorScordiff);
    SmartDashboard.putData("Auto choice", m_chooser);

  }

//
//   ###   #   #   ###   #####   ###    ###   #       ###   #####  #####
//    #    ##  #    #      #      #    #   #  #        #       #   #
//    #    # # #    #      #      #    #####  #        #      #    ####
//    #    #  ##    #      #      #    #   #  #        #     #     #
//   ###   #   #   ###     #     ###   #   #  #####   ###   #####  #####
//
  public void init() {
    alliance = DriverStation.getAlliance();
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
//  dispatch to appropriate function for the selected mode
  public void run() {
    SmartDashboard.putString("auto state", state);
    switch (mode) {
        case autoCustom:
          // custom routine
          doCustom();
          break;
        case autoLeav:
          doLeav();
          break;
        case autoScorLeav:
          doScorLeav();
          break;
        case autoScorLeavScorsame:
          doScorLeavScorsame();
          break;
        case autoScorLeavScordiff:
          doScorLeavScordiff();
          break;
        case autoScorDock:
          doScorDock();
          break;
        case autoScorLeavDock:
          doScorLeavDock();
          break;
        case autoScorLeavScorDock:
          doScorLeavScorDock();
          break;
        case autoScorLeavScorScorsame:
          doScorLeavScorScorsame();
          break;
        case autoScorLeavScorScordiff:
          doScorLeavScorScordiff();
          break;
        case autoDefault:
        default:
          // default routine is expected to do nothing
          doDefault();
          break;
    }
  }


// end of Autonomous class
}