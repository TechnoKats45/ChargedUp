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

  public final double kP_drive = 0.005;
  public final double kI_drive = 0.001;
  public final double kD_drive = 0;

  public final double kP_turn = 0.002;
  public final double kI_turn = 0.001;
  public final double kD_turn = 0;

  public final double kk_WheelInchesPerCount = (
    kk_wheeldiameter /* inch */ / 1.0 /* diameter */
  * 3.14159 /* diameter */ / 1.0 /* circumference */
  * 1.0 /* circumference */ / kk_gearreduction /* motor revolutions */
  * 1.0 /* motor revoluions */ / 1.0 /* count */
  );

  public final double kk_TurnDegreesPerCount = (
    kk_wheeldiameter /* inch */ / 1.0 /* diameter */
  * 3.14159 /* diameter */ / 1.0 /* circumference */
  * 1.0 /* circumference */ / kk_gearreduction /* motor revolutions */
  * 1.0 /* motor revolutions */ / 1.0 /* count */
  * 1.0 /* wheelbase */ / kk_wheelbase /* inch */
  * 1.0 /* circle */ / 6.28308 /* wheelbase */
  * 360.0 /* degree */ / 1.0 /* circle */
  * kk_slipfactor /* slip factor */
  );

  // arm parameters
   public final double kk_extensionmin = 0;
   public final double kk_extensionmax = 19.5;
 
   public final double kk_elevationmin = -90;
   public final double kk_elevationmax = 90;

  public final double kP_elevation = 0.005;
  public final double kI_elevation = 0.001;
  public final double kD_elevation = 0;

  public final double kP_slide = 0.005;
  public final double kI_slide = 0.001;
  public final double kD_slide = 0;

  public final double kP_extension = 0.005;
  public final double kI_extension = 0.001;
  public final double kD_extension = 0;

  public final double kk_ExtensionInchesPerCount = (
      0.375 /* inches */ / 1.0 /* holes */
    * 1.0 /* holes */ / 1.0 /* teeth */
    * 18.0 /* teeth */ / 1.0 /* sprocket revolution */
    * 1.0 /* sprocket revolution */ / 12.0 /* motor revolutions */
    * 1.0 /* motor revolutions */ / 1.0 /* encoder counts */
  );

  public final double kk_ArmDegreesPerCount = (
      360.0 /* degrees */ / 1.0 /* arm revolution */
    * 1.0 /* arm revolutions */ / 34.0 /* upper sprocket teeth */
    * 1.0 /* sprocket teeth */ / 1.0 /* chain links */
    * 1.0 /* chain links */ / 1.0 /* lower sprocket teeth */
    * 12.0 /* lower sprocket teeth */ / 1.0 /* lower sprocket revolution */
    * 1.0 /* lower sprocket revolutions */ / 64.0 /* motor revolutions */
    * 1.0 /* motor revolutions */ / 1.0 /* encoder counts */
  );

  public final double kk_ArmSlideMin = 0;
  public final double kk_ArmSlideMax = 5;

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