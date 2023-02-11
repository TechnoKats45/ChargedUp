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

// Logitech ATTACK 3 joystick button numbers
public final int kj_1 = 0; 
public final int kj_2 = 1; 
public final int kj_3 = 2; 
public final int kj_4 = 3; 
public final int kj_5 = 4; 
public final int kj_6 = 5; 
public final int kj_7 = 6; 
public final int kj_8 = 7; 
public final int kj_9 = 8; 
public final int kj_10 = 9; 
public final int kj_11 = 10; 
// Logitech ATTACK 3 joystick button labels
public final int kj_trig = kj_1; // trigger
public final int kj_down = kj_2; // thumb down
public final int kj_up = kj_3; // thumb up
public final int kj_left = kj_4; // thumb left
public final int kj_right = kj_5; // thumb right
public final int kj_leftfar = kj_6; // base left far
public final int kj_leftnear = kj_7; // base left near
public final int kj_centerleft = kj_8; // base center left
public final int kj_centerright = kj_9; // base center right
public final int kj_rightnear = kj_10; // base right near
public final int kj_rightfar = kj_11; // base right far

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
  * 1.0 /* motor revolutions */ / 1.0 /* count */
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

  public final double kk_slidemin = -6;
  public final double kk_slidemax = 6;

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

  public final double kk_SlideInchesPerCount = (
      1.0 /* inches */ / 25.4 /* mm */
    * 3.0 /* mm */ / 1.0 /* teeth */
    * 36.0 /* teeth */ / 1.0 /* pulley revolutions */
    * 1.0 /* pulley revolutions */ / 5.0 /* sensor counts */
  );

  public final double kk_ArmSlideMin = -5;
  public final double kk_ArmSlideMax = 5;

// CAN motor control IDs
// -- drivebase --
  public final int kmc_left1 = 2;
  public final int kmc_left2 = 4;
  public final int kmc_left3 = 6;
  public final boolean kk_leftinvert = false;
  public final int kmc_right1 = 1;
  public final int kmc_right2 = 3;
  public final int kmc_right3 = 5;
  public final boolean kk_rightinvert = true;
// -- arm --
  public final int kmc_elevate = 7;
  public final boolean kk_elevateinvert = true;
  public final int kmc_extend = 8;
  public final boolean kk_extendinvert = false;

// PWM motor control IDs
// -- arm --
public final int kmp_slide = 0;
public final boolean kk_slideinvert = false;

// Pneumatic control IDs
public final int kpd_grab_in = 0;
public final int kpd_grab_out = 1;

// Digital inputs
public int kdi_slideleft = 0;
public int kdi_slideright = 1;
public int kdi_gamepiecesense = 2;

// Analog inputs
public int kai_slide = 0;


  public Config() {
  }


// end of Config class
}