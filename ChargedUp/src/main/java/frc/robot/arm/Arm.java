
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

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
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
     slidemotor = new Spark(config.kmp_slide);
     extensionmotor = new CANSparkMax(config.kmc_extend, MotorType.kBrushless);

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
    if (c_enable()) {

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
        elevate(0);
        slide(0);
        extend(0);
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

// end of Arm class
}