
package Team4450.Robot22;

import java.util.Properties;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "RAC22-03.08.22-1";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false;
	    	
	public static DriverStation.Alliance	alliance;
	public static int                       location, matchNumber;
	public static String					eventName, gameMessage;
	    
	// Drive motor controller port assignments.
	public static final int		LF_TALON = 1, LR_TALON = 2, RF_TALON = 3, RR_TALON = 4;

	// Other motor controller port assignments
	public static final int		UPPER_PICKUP_VICTOR = 6, LOWER_PICKUP_VICTOR = 5, INDEXER_VICTOR = 7;
	public static final int		LEFT_CLIMBER_VICTOR = 8, RIGHT_CLIMBER_TALON = 9, SHOOTER_TALON = 10;
	
	// Joystick port assignments.
	public static final int		LEFT_STICK = 0, RIGHT_STICK = 1, UTILITY_STICK = 2, LAUNCH_PAD = 3, GAME_PAD = 4;

	// Pneumatic valve controller port assignments.
	public static final int		COMPRESSOR = 0;
	public static final int		PICKUP_VALVE = 0;			// 0-1
	public static final int		MAIN_CLIMBER_VALVE = 4;		// 4-5
	public static final int		AUX_CLIMBER_VALVE = 2;		// 2-3

	// Digital Input port assignments. Encoder takes 2 ports.
	public static final int		CLIMBER_SWITCH = 0;
	//public static final int		CLIMBER_ENCODER = 1;		// 1-2

	// Simulation dummy encoders use DIO port numbers above the actual ports on RoboRio.
	public static final int		DUMMY_LEFT_ENCODER = 10, DUMMY_RIGHT_ENCODER = 12;
	  
	// Analog Input port assignments.
	// Simulated Gyro needs an actual analog port and has to be 0 or 1.
	public static final int		SIM_GYRO = 0;
    public static final int		PRESSURE_SENSOR = 1;

    public static final double	TALON_RAMP_RATE = 1.0;			// Takes 1 sec for full power to be applied.
                                                                // Tried going above 1.0 but behavior became
                                                                // sketchy...
	public static final double  DRIVE_WHEEL_DIAMETER = 6.20;	// Inches.
	public static final double	STEERING_ASSIST_GAIN = .05;
    public static final double	TRACK_WIDTH = 30;				// Inches. 
	public static final double	GEAR_RATIO = 12.6;				// Overall gear ratio, motor rotations to one
																// wheel rotation.
	public static final double	ROBOT_WEIGHT = 154;				// Pounds.
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // TankDrive, ArcadeDrive command.
	public static final int		LCD_3 = 3;	    // TankDrive, ArcadeDrive command.
	public static final int		LCD_4 = 4;	    // TankDrive command, AutoDrive commnd.
	public static final int		LCD_5 = 5;	    // TankDrive command
	public static final int		LCD_7 = 7;	    // TankDrive, ArcadeDrive command
	public static final int		LCD_8 = 8;	    // TankDrive, ArcadeDrive command
	public static final int		LCD_9 = 9;	    // TankDrive command.
	public static final int		LCD_10 = 10;	// Not used.

	// Default starting field position in meters for pose tracking. For full field lower left corner.
	public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(1.2, 0.5, new Rotation2d(Math.toRadians(0)));

	// 2022 starting positions. Location in meters, angle in degrees.

	public static final Pose2d	BLUE_1 = new Pose2d(6.687, 5.406, new Rotation2d(Math.toRadians(243)));
	public static final Pose2d	BLUE_2 = new Pose2d(6.148, 4.022, new Rotation2d(Math.toRadians(160)));
	public static final Pose2d	BLUE_3 = new Pose2d(6.679, 2.818, new Rotation2d(Math.toRadians(153)));
	public static final Pose2d	BLUE_4 = new Pose2d(8.133, 2.251, new Rotation2d(Math.toRadians(70)));

	public static final Pose2d	RED_1 = new Pose2d(9.465, 2.812, new Rotation2d(Math.toRadians(60)));
	public static final Pose2d	RED_2 = new Pose2d(9.866, 4.230, new Rotation2d(Math.toRadians(336)));
	public static final Pose2d	RED_3 = new Pose2d(9.311, 5.373, new Rotation2d(Math.toRadians(332)));
	public static final Pose2d	RED_4 = new Pose2d(7.902, 6.003, new Rotation2d(Math.toRadians(246)));

    // Use these values in PathWeaver for speed and acceleration.
    // Robot will go faster than this, more like 3 mps but this value tones down autonomous speed.

    public static final double  MAX_WHEEL_SPEED = 2.0;     // Meters per second.
    public static final double  MAX_WHEEL_ACCEL = 1.0;     // Meters per second per second.
    
    // Estimated by eyeball observation. Needs to be estimated each new robot.

    public static final double  MAX_ROTATIONAL_VEL = 70;    // Degrees per second.
    public static final double  MAX_ROTATIONAL_ACCEL = 70;  // Degrees per second per second.

    // Drive base characterization results. These values from 2021 as placeholders until 2022
	// characterization is done.

    public static final double  TRACK_WIDTH_C = Util.inchesToMeters(TRACK_WIDTH);	// Meters.

    public static final double  DB_KS = 1.74;
    public static final double  DB_KV = 1.8;
    public static final double  DB_KA = .422;

    public static final double  DB_POSITIONAL_KP = .0688; 
    public static final double  DB_POSITIONAL_KD = 36.5; 
    public static final double  DB_VELOCITY_KP = .12;  
    public static final double  DB_VELOCITY_KD = 0.0;
}
