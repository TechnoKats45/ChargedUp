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

// Logitech ATTACK 3 joystick button labels
public final int kj_trig = 1; // trigger
public final int kj_down = 2; // thumb down
public final int kj_up = 3; // thumb up
public final int kj_left = 4; // thumb left
public final int kj_right = 5; // thumb right
public final int kj_leftfar = 6; // base left far
public final int kj_leftnear = 7; // base left near
public final int kj_centerleft = 8; // base center left
public final int kj_centerright = 9; // base center right
public final int kj_rightnear = 10; // base right near
public final int kj_rightfar = 11; // base right far

// drivebase parameters
  public final double kk_accel = 1.0; // acceleration limit

  public final double kk_wheeldiameter = 6;
  public final double kk_gearreduction = 7.56;
  public final double kk_wheelbase = 18.0;
  public final double kk_slipfactor = 0.8;

  public final double kP_drive = 0.01;
  public final double kI_drive = 0.0;
  public final double kD_drive = 0.0;

  public final double kP_turn = 0.02;
  public final double kI_turn = 0.0;
  public final double kD_turn = 0.0;

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

  public final double kk_LEDtime = 20; /* seconds */
  public final double kk_armGREENmin = 42; /* degrees */
  public final double kk_armGREENmax = 48; /* degrees */

  // arm parameters
  public final double kk_extensionmin = 0;
  public final double kk_extensionmax = 19.5;
 
  public final double kk_extend_rate = 4.0; /* 1/4 second to full power */

  public final double kk_elevationmin = -127;
  public final double kk_elevationmax = 107;

  //constants to limit arm movement
  public final double kpc_armlength = 27.0;
  public final double kpc_maxheight = 40.0;
  public final double kpc_maxlength = 36.0;


  // @@ presets: elevation degrees, extension inches
  public final double kk_elevation_preset0 = 63;
  public final double kk_extension_preset0 = 8;
  public final double kk_elevation_preset1 = -123;
  public final double kk_extension_preset1 = 0;
  public final double kk_elevation_preset2 = -65;
  public final double kk_extension_preset2 = 3.5;
  public final double kk_elevation_preset3 = -57;
  public final double kk_extension_preset3 = 17;


  public final double kk_elevate_rate = 2.0; /* 1/2 second to full power */

  public final double kk_slidemin = -6;
  public final double kk_slidemax = 6;

  public final double kP_elevation = 0.02;
  public final double kI_elevation = 0.00;
  public final double kD_elevation = 0.00;

  public final double kP_slide = 0.005;
  public final double kI_slide = 0.001;
  public final double kD_slide = 0;

  public final double kP_extension = 0.4;
  public final double kI_extension = 0.0;
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
    * 5.0 /* mm */ / 1.0 /* teeth */
    * 42.0 /* teeth */ / 1.0 /* pulley revolutions */
    * 1.0 /* pulley revolutions */ / 5.0 /* sensor counts */
  );

  public final double kk_ArmSlideMin = -4.75;
  public final double kk_ArmSlideMax =  3.50;


// gripper parameters
  public final double kk_holdrange = 2.2; /* inches */ 

  public final double kP_theta = 0.5;
  public final double kI_theta = 0.0;
  public final double kD_theta = 0.0;


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
  public final int kmc_rightintake = 12;
  public final int kmc_leftintake = 10;
  public final int kmc_theta = 11;
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
public final int kpd_rotate_fwd = 2;
public final int kpd_rotate_rev = 3;

// Digital inputs
public final int kdi_slideleft = 0;
public final int kdi_slideright = 1;
public final int kdi_gamepiecesense = 2;
public final int kdi_forcearmcoast = 3;

// Servos
public final int ksp_grip = 1;

// Digital outputs
public final int kdo_color1 = 4;
public final int kdo_color2 = 5;
public final int kdo_color3 = 6;

// Analog inputs
public final int kai_slide = 0;


  public Config() {
  }


// end of Config class
}