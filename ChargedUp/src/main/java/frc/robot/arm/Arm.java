
// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//   ###   ####   #   #
//  #   #  #   #  ## ##
//  #####  ####   # # #
//  #   #  #   #  #   #
//  #   #  #   #  #   #
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

package frc.robot.arm;

import java.sql.ResultSet;

import org.ejml.data.ElementLocation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.*;
//  imports (controllers, actuators, sensors, communication)

public class Arm {

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
Joystick control;

CANSparkMax elevationmotor;
Spark slidemotor;
CANSparkMax extensionmotor;
private Config config = new Config();

double angleOrigin = 0;
double inchesOrigin = 0;

PIDController elevation_pid = new PIDController(config.kP_elevation, config.kI_elevation, config.kD_elevation);
PIDController slide_pid = new PIDController(config.kP_slide, config.kI_slide, config.kD_slide);
PIDController extension_pid = new PIDController(config.kP_extension, config.kI_extension, config.kD_extension);

double extension_target = 0;

//
//   ###    ###   #   #  #####  ####    ###   #      
//  #   #  #   #  ##  #    #    #   #  #   #  #      
//  #      #   #  # # #    #    ####   #   #  #      
//  #   #  #   #  #  ##    #    #  #   #   #  #      
//   ###    ###   #   #    #    #   #   ###   #####  
//
//  private functions for driver/operator input

boolean c_extend(){
    return control.getRawButton(3);
}

boolean c_retract(){
    return control.getRawButton(2);
}

boolean c_left(){
    return control.getRawButton(4);
}

boolean c_right(){
    return control.getRawButton(5);
}

double c_elevate(){
    return control.getY();
}

boolean c_enable(){
    return control.getTrigger();
}

boolean c_resetangle(){
    return control.getRawButton(9);
}

boolean c_resetextension() {
    return control.getRawButton(10);
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
    elevationmotor.set(speed);
}
void extend(double speed) {
    extensionmotor.set(speed);
}
void slide(double speed) {
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
    return 0;
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
     elevationmotor.setIdleMode(IdleMode.kBrake);
     anglereset(0);
     slidemotor = new Spark(config.kmp_slide);
     extensionmotor = new CANSparkMax(config.kmc_extend, MotorType.kBrushless);
     extensionmotor.setIdleMode(IdleMode.kCoast);
     inchesreset(0);
     
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
    // set arm angle to requested arm angle
    // set arm extension to requested arm extension
    if (c_resetextension()) {
        inchesreset(0); 
    }
    double extend = extension_pid.calculate(extensioninches(), extension_target);


    // set arm slide to make it line up with vision target
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
    SmartDashboard.putNumber("Arm elevation", elevationangle());
    SmartDashboard.putNumber("Arm extension", extensioninches());
    SmartDashboard.putNumber("Arm slide", slideinches());
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
    if (c_enable()) {
        SmartDashboard.putBoolean("arm enable", true);

        if(c_extend()){
            extend(0.5);
        }
        if(c_retract()){
            extend(-0.5);
        }
        if(c_left()){
            slide(-0.5);
        }
        if(c_right()){
            slide(0.5);
        }
        elevate(c_elevate());
    }
    
    else{
        SmartDashboard.putBoolean("arm enable", false);
        elevate(0);
        slide(0);
        extend(0);
    }
}

// end of Arm class
}