
// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//   ###   #   #  #####   ###   #   #   ###   #   #   ###   #   #   ####
//  #   #  #   #    #    #   #  ##  #  #   #  ## ##  #   #  #   #  #
//  #####  #   #    #    #   #  # # #  #   #  # # #  #   #  #   #   ###
//  #   #  #   #    #    #   #  #  ##  #   #  #   #  #   #  #   #      #
//  #   #   ###     #     ###   #   #   ###   #   #   ###    ###   ####
//
//  state machines for autonomous routines

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

//  menu of autonomous routines to be presented on Dashboard for selection  
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
private static final String autoScorOnly = "Score"; //10

private final SendableChooser<String> m_chooser = new SendableChooser<>();  

//  the field is mirrored, so some directions will need to be reversed for some autonomous routines based on red vs blue alliance
private Alliance alliance = Alliance.Invalid;

//  state machines have persistent states
private String routine = "";
private String state = "";

//  subsystems to be controlled autonomously
private Drivebase drivebase;
private Arm arm;
private Gripper gripper;

//  peristent measurement variables
private long timestamp;

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
// Use this function to develop an autonomous routine, then copy it to the routine's function when it works satisfactorily
private void doCustom() {
  switch (state) {
    case "Start":
      drivebase.auto_drive(48);
      setstate("Forward");
      break;
    case "Forward":
      if (drivebase.attarget()) {
        arm.auto_elevate(90);
        setstate("Arm Straight");
      }
      break;
    case "Arm Straight":
      if (arm.elevation_at_target()) {
        drivebase.auto_turn(90);
        setstate("Turn Right");
      }
      break;
    case "Turn Right":
      if (drivebase.attarget()) {
        drivebase.auto_drive(24);
        setstate("Drive Right");
      }
      break;
    case "Drive Right":
      if (drivebase.attarget()) {
        drivebase.auto_turn(90);
        setstate("Turn Back");
      }
      break;
    case "Turn Back":
      if (drivebase.attarget()) {
        drivebase.auto_drive(48);
        setstate("Back");
      }
    case "Back":
      if (drivebase.attarget()) {
        arm.auto_elevate(0);
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
// @@ to be implemented
private void doLeav() {
  switch (state) {
    case "Start":
      drivebase.auto_drive(168);
      setstate("End");
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
// @@ to be implemented
// >>> set robot back against substation, arm lined up with node, gamepiece in gripper
// start:
//  extend arm 10 (or whatever makes sense)
//  elevate arm -60 (or whatever makes sense)
// arm back:
//  wait for elevation at setpoint
//  open gripper
//  extend arm 0
//  elevate arm 90
// arm forward:
//  wait for elevation at setpoint
//  drive forward 14 feet
// end
private void doScorLeav() {
  switch (state) {
    case "Start":
      arm.auto_extend(10);
      arm.auto_elevate(-85);
      setstate("Arm Set");
      break;
    case "Arm Set":
      if(arm.elevation_at_target() && arm.extension_at_target()) {
        timestamp = System.currentTimeMillis();
        setstate("Settled");
      }
      break;
    case "Settled":
      if(timestamp + 1000 <= System.currentTimeMillis()) {
        gripper.release();
        timestamp = System.currentTimeMillis();
        setstate("Placed");
      }
      break;
    case "Placed":
      if (timestamp + 500 <= System.currentTimeMillis()) {
        arm.auto_extend(0);
        arm.auto_elevate(90);
        setstate("Scored");
      }
      break;
    case "Scored":
      if(arm.elevation_at_target() && arm.extension_at_target()) {
        drivebase.auto_drive(168);
        setstate("End");
      }
      break;
    case "End":
      break;
    default:
      setstate ("Error");
      break;
  }
}

public void doScor() {
  switch (state) {
    case "Start":
      arm.auto_extend(10);
      arm.auto_elevate(-85);
      setstate("Arm Set");
      break;
    case "Arm Set":
      if(arm.elevation_at_target() && arm.extension_at_target()) {
        timestamp = System.currentTimeMillis();
        setstate("Settled");
      }
      break;
    case "Settled":
      if(timestamp + 1000 <= System.currentTimeMillis()) {
        gripper.release();
        timestamp = System.currentTimeMillis();
        setstate("Placed");
      }
      break;
    case "Placed":
      if (timestamp + 500 <= System.currentTimeMillis()) {
        arm.auto_extend(0);
        arm.auto_elevate(90);
        setstate("End");
      }
      break;
    case "End":
      break;
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
// @@ to be implemented
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
// @@ to be implemented
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
// @@ to be implemented
private void doScorDock() {
  switch (state) {
    case "Start":
      arm.auto_extend(10);
      arm.auto_elevate(-85);
      setstate("Arm Set");
      break;
    case "Arm Set":
      if(arm.elevation_at_target() && arm.extension_at_target()) {
        timestamp = System.currentTimeMillis();
        setstate("Settled");
      }
      break;
    case "Settled":
      if(timestamp + 1000 <= System.currentTimeMillis()) {
        gripper.release();
        timestamp = System.currentTimeMillis();
        setstate("Placed");
      }
      break;
    case "Placed":
      if (timestamp + 500 <= System.currentTimeMillis()) {
        arm.auto_extend(0);
        arm.auto_elevate(90);
        setstate("Scored");
      }
      break;
    case "Scored":
      drivebase.auto_drive(105);
      setstate("On Dock");
      break;
    case "On Dock":
      if (drivebase.attarget()) {
        if (drivebase.pitch() > 0.5) {
          drivebase.auto_drive(1);
        }
        else if (drivebase.pitch() < -0.5) {
          drivebase.auto_drive(-1);
        }
        else {
          setstate("End");
        }
      }
      break;
    case "End":
      break;
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
// @@ to be implemented
private void doScorLeavDock() {
  
  switch (state) {
    case "Start":
      arm.auto_extend(10);
      arm.auto_elevate(-85);
      setstate("Arm Set");
      break;
    case "Arm Set":
      if(arm.elevation_at_target() && arm.extension_at_target()) {
        timestamp = System.currentTimeMillis();
        setstate("Settled");
      }
      break;
    case "Settled":
      if(timestamp + 1000 <= System.currentTimeMillis()) {
        gripper.release();
        timestamp = System.currentTimeMillis();
        setstate("Placed");
      }
      break;
    case "Placed":
      if (timestamp + 500 <= System.currentTimeMillis()) {
        arm.auto_extend(0);
        arm.auto_elevate(90);
        setstate("Scored");
      }
      break;
    case "Scored":
      drivebase.auto_drive(190);
      setstate("Out");
      break;
    case "Out":
      if(drivebase.attarget()){
        drivebase.auto_drive(-100);
        setstate("On Dock");
      }
      break;
      case "On Dock":
      if (drivebase.attarget()) {
        if (drivebase.pitch() > 0.5) {
          drivebase.auto_drive(1);
        }
        else if (drivebase.pitch() < -0.5) {
          drivebase.auto_drive(-1);
        }
        else {
          setstate("End");
        }
      }
      break;
    case "End":
      break;
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
// @@ to be implemented
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
// @@ to be implemented
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
// @@ to be implemented
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
    menuOption(autoScorOnly);
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
    routine = m_chooser.getSelected();
    SmartDashboard.putString("auto routine", routine);
    System.out.println("Auto selected: " + routine);
    setstate("Start");
  }


//
//  ####   #   #  #   #
//  #   #  #   #  ##  #
//  ####   #   #  # # #
//  #   #  #   #  #  ##
//  #   #   ###   #   #
//
//  dispatch to appropriate function for the selected routine
//  @@ this could be made much simpler with function pointers, but Java doesn't quite have those
//  @@ it could also be shortened using individual "routine" objects, but that's more work than doing it this way
  public void run() {
    SmartDashboard.putString("auto state", state);
    switch (routine) {
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
        case autoScorOnly:
          doScor();
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