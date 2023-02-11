// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//  ####    ###   ####    ###   #####  
//  #   #  #   #  #   #  #   #    #    
//  ####   #   #  ####   #   #    #    
//  #   #  #   #  #   #  #   #    #    
//  #   #   ###   ####    ###     #    
//
//  Robot object - based on TimedRobot
//
//  implements init() and periodic() functions:
//    robotInit(), robotPeriodic()
//    autonomousInit(), autonomousPeriodic()
//    disabledInit(), disabledPeriodic()
//    teleopInit(), teleopPeriodic()
//    testInit(), testPeriodic()
//  all implemented functions call the appropriate method of each subsystem's object
//
//  defines driver and operator controllers (gamepad, joystick, etc)
//  and passes the controller to the appropriate subsystem so that multiple subsystems can use the same controller
//
//  instantiates an object for each subsystem:
//    drivebase
//    arm
//    gripper
//
//  instantiates an autonomous object and passes it all of the subsystems to allow for autonomous control
//
//  each subsystem implements three required methods:
//    idle() is called from robotPeriodic() and is intended for Dashboard telemetry
//    run() is called from teleopPeriodic() and is intended for normal driver-controlled operation
//    test() is called from testPeriodic() and is intended for testing of individual subsystem functions and softare debugging
//  in addition each subsystem implements auto_() functions for the autonomous routines to call in order to control the subsystem
//
//

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.drivebase.*;
import frc.robot.arm.*;
import frc.robot.gripper.*;
import frc.robot.autonomous.*;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private XboxController drivercontroller = new XboxController(0);
  private Drivebase drivebase = new Drivebase(drivercontroller);
  private Joystick operatorcontroller = new Joystick(1);
  private Arm arm = new Arm(operatorcontroller);
  private Gripper gripper = new Gripper(operatorcontroller);
  private Autonomous autonomous = new Autonomous(drivebase, arm, gripper);
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // all of the initialization functions are done in the constructor for each subsystem
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    drivebase.idle();
    arm.idle();
    gripper.idle();
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    autonomous.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    autonomous.run();
    drivebase.run();
    arm.run();
    gripper.run();
 }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drivebase.run();
    arm.run();
    gripper.run();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    drivebase.test();
    arm.test();
    gripper.test();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
