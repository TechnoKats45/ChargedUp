
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.config.*;


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

    private Config config = new Config();

    // driving
    public double odoOrigin = 0;
    public double dirOrigin = 0;
  

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

  // read accumulated left side motor position
  private double odoleft() {
    return leftmotor1.getEncoder().getPosition();
  }

  // read accumulated right side motor position
  private double odoright() {
    return -rightmotor1.getEncoder().getPosition();
  }

  // call with (false) to read the number of inches traveled since the last reset
  // call with (true) to reset the odometer to zero
  private double odometer(boolean reset) {
    // inches per count is computed from robot wheel size, gearbox ratio, and encoder counts per motor revolution
    final double inchesPerCount = (
      config.kk_wheeldiameter /* inch */ / 1.0 /* diameter */
    * 3.14159 /* diameter */ / 1.0 /* circumference */
    * 1.0 /* circumference */ / config.kk_gearreduction /* motor revolutions */
    * 1.0 /* motor revoluions */ / 1.0 /* count */
    );
    // raw robot odometer is average of left and right side raw values
    double odoRaw = (odoleft()+odoright())/2.0;
    if (reset) {
      odoOrigin = odoRaw;
    }
    return (odoRaw - odoOrigin) * inchesPerCount;
  }

  // call with (false) to read the number of degrees turned since the last reset
  // call with (true) to reset the direction to zero
  private double direction(boolean reset) {
    // degrees per count is computed from robot wheel size, wheelbase, gearbox ratio, and encoder counts per motor revolution
    final double degreesPerCount = (
      config.kk_wheeldiameter /* inch */ / 1.0 /* diameter */
    * 3.14159 /* diameter */ / 1.0 /* circumference */
    * 1.0 /* circumference */ / config.kk_gearreduction /* motor revolutions */
    * 1.0 /* motor revolutions */ / 1.0 /* count */
    * 1.0 /* wheelbase */ / config.kk_wheelbase /* inch */
    * 1.0 /* circle */ / 6.28308 /* wheelbase */
    * 360.0 /* degree */ / 1.0 /* circle */
    * config.kk_slipfactor /* slip factor */
    );
    // raw robot direction is difference between left and right side raw values
    double dirRaw = (odoleft()-odoright());
    if (reset) {
      dirOrigin = dirRaw;
    }
    return (dirRaw - dirOrigin) * degreesPerCount;
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
    leftmotor3.set(left);
    rightmotor1.set(right);
    rightmotor2.set(right);
    rightmotor1.set(right);
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
    SmartDashboard.putNumber("odo left raw", odoleft());
    SmartDashboard.putNumber("odo right raw", odoright());
    SmartDashboard.putNumber("db distance", odometer(false));
    SmartDashboard.putNumber("db direction", direction(false));
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
