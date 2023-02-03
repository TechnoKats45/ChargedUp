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

  public final double kP_drive = 0;
  public final double kI_drive = 0;
  public final double kD_drive = 0;

  public final double kP_turn = 0;
  public final double kI_turn = 0;
  public final double kD_turn = 0;

  public final double kk_inchesPerCount = (
    kk_wheeldiameter /* inch */ / 1.0 /* diameter */
  * 3.14159 /* diameter */ / 1.0 /* circumference */
  * 1.0 /* circumference */ / kk_gearreduction /* motor revolutions */
  * 1.0 /* motor revoluions */ / 1.0 /* count */
  );

  public final double kk_degreesPerCount = (
    kk_wheeldiameter /* inch */ / 1.0 /* diameter */
  * 3.14159 /* diameter */ / 1.0 /* circumference */
  * 1.0 /* circumference */ / kk_gearreduction /* motor revolutions */
  * 1.0 /* motor revolutions */ / 1.0 /* count */
  * 1.0 /* wheelbase */ / kk_wheelbase /* inch */
  * 1.0 /* circle */ / 6.28308 /* wheelbase */
  * 360.0 /* degree */ / 1.0 /* circle */
  * kk_slipfactor /* slip factor */
  );


// CAN motor control IDs
// -- drivebase --
  public final int kmc_left1 = 2;
  public final int kmc_left2 = 4;
  public final int kmc_left3 = 6;
  public final int kmc_right1 = 1;
  public final int kmc_right2 = 3;
  public final int kmc_right3 = 5;
  public final boolean kk_leftinvert = false;
  public final boolean kk_rightinvert = true;
// -- arm --
  public final int kmc_elevate = 7;
  public final int kmc_extend = 8;

// PWM motor control IDs
// -- arm --
public final int kmp_slide = 0;
// -- gripper --
public final int kmp_rotate = 1;

// Pneumatic control IDs
public final int kpd_grab_in = 0;
public final int kpd_grab_out = 1;

// Digital inputs
public int kdi_armforward = 0;
public int kdi_armbackward = 1;
public int kdi_extendout = 2;
public int kdi_extendin = 3;
public int kdi_slideleft = 4;
public int kdi_slideright = 5;

// Analog inputs
public int kai_elevate = 0;
public int kai_slide = 1;


  public Config() {
  }


// end of Config class
}