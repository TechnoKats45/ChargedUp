// TechnoKats Robotics Team - 2023 FRC season: CHARGED UP
//
//   ###    ###   #   #  #####   ###    ####
//  #   #  #   #  ##  #  #        #    #
//  #      #   #  # # #  ####     #    # ###
//  #   #  #   #  #  ##  #        #    #   #
//   ###    ###   #   #  #       ###    ###  
//  
//  configuration constants
//

package frc.robot.config;

public class Config {

// drivebase parameters
  public final double kk_wheeldiameter = 6;
  public final double kk_gearreduction = 7.56;
  public final double kk_wheelbase = 18.0;
  public final double kk_slipfactor = 0.8;

// CAN motor control IDs
// -- drivebase --
  public final int km_left1 = 1;
  public final int km_left2 = 3;
  public final int km_left3 = 5;
  public final int km_right1 = 2;
  public final int km_right2 = 4;
  public final int km_right3 = 6;
// -- arm --
  public final int km_elevate = 7;
  public final int km_extend = 8;

// PWM motor control IDs
// -- arm --
public final int km_slide = 0;
// -- gripper --
public final int km_rotate = 1;

// Pneumatic control IDs
public final int kp_grab_in = 0;
public final int kp_grab_out = 1;

// Digital inputs
public int kd_armforward = 0;
public int kd_armbackward = 1;
public int kd_extendout = 2;
public int kd_extendin = 3;
public int kd_slideleft = 4;
public int kd_slideright = 5;

// Analog inputs
public int ka_elevate = 0;
public int ka_slide = 1;


  public Config() {
  }


// end of Config class
}
