
package Team4450.Robot22;

import static Team4450.Robot22.Constants.*;

import java.io.IOException;
import java.nio.file.Path;

import Team4450.Lib.CameraFeed;
import Team4450.Lib.GamePad;
import Team4450.Lib.JoyStick;
import Team4450.Lib.LaunchPad;
import Team4450.Lib.MonitorCompressor;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.GamePad.GamePadButtonIDs;
import Team4450.Lib.JoyStick.JoyStickButtonIDs;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import Team4450.Robot22.commands.ArcadeDrive;
import Team4450.Robot22.commands.Climb;
import Team4450.Robot22.commands.TankDrive;
import Team4450.Robot22.commands.Utility.NotifierCommand;
import Team4450.Robot22.commands.autonomous.DriveOut;
import Team4450.Robot22.commands.autonomous.ShootFirst;
import Team4450.Robot22.subsystems.Channel;
import Team4450.Robot22.subsystems.Climber;
import Team4450.Robot22.subsystems.DriveBase;
import Team4450.Robot22.subsystems.Pickup;
import Team4450.Robot22.subsystems.Shooter;
import Team4450.Robot22.subsystems.ShuffleBoard;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// Subsystems.

	public static ShuffleBoard	shuffleBoard;
	public static DriveBase 	driveBase;
	public static Channel		channel;
	public static Pickup		pickup;
	public static Shooter		shooter;
	public static Climber		climber;

	// Subsystem Default Commands.

	private final TankDrive		driveCommand;
	//private final ArcadeDrive	driveCommand;

    // Persistent Commands.
	

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinihsed. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes). Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// Joy sticks. 3 Joy sticks use RobotLib JoyStick class for some of its extra features. The
	// wpilib Joystick is passed into our JoyStick. This means we can use features of both.
	// Specify trigger for monitoring to cause our JoyStick event monitoring to not start. We will 
	// use WpiLib button handling instead of RobotLib event monitoring. Note that button responsiveness
	// may be slowed as the schedulers command list gets longer or commands get longer as buttons are
	// processed once per scheduler run. RobotLib buttons are monitored in a separate thread and execute
	// functions in that separate thread and so are not generally affected by other functions in terms of
	// responsiveness. Which way is best? Thats a debatable topic. We use wpilib buttons here to conform
	// to FIRSTs "approved" way of doing things. Launch pad monitoring uses regular wpilib Joytick class.
	
	private JoyStick	leftStick = new JoyStick(new Joystick(LEFT_STICK), "Left Stick", JoyStickButtonIDs.TRIGGER);
	private JoyStick	rightStick = new JoyStick(new Joystick(RIGHT_STICK), "Right  Stick", JoyStickButtonIDs.TRIGGER);
	public static JoyStick utilityStick = new JoyStick(new Joystick(UTILITY_STICK), "Utility Stick", JoyStickButtonIDs.TRIGGER);

	private Joystick	launchPad = new Joystick(LAUNCH_PAD);

    private GamePad     gamePad;	// Used during simulation.

	private AnalogInput	pressureSensor = new AnalogInput(PRESSURE_SENSOR);
	  
	private PowerDistribution		pdp = new PowerDistribution(0, PowerDistribution.ModuleType.kCTRE);

	// PneumaticsControlModule class controls the PCM. New for 2022.
	private PneumaticsControlModule	pcm = new PneumaticsControlModule(COMPRESSOR);

	// Navigation board.
	public static NavX			navx;

	private Thread      		monitorPDPThread;
	private MonitorCompressor	monitorCompressorThread;
    private CameraFeed			cameraFeed;
    
	// Trajecotries.
    //public static Trajectory    ;

    // List of autonomous programs. Any change here must be reflected in getAutonomousCommand()
    // and setAutoChoices() which appear later in this class.
	private enum AutoProgram
	{
		NoProgram,
		DriveOut,
		ShootFirst
	}

	private static SendableChooser<AutoProgram>	autoChooser;
	private static SendableChooser<Pose2d>		startingPoseChooser;

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception
	{
		Util.consoleLog();

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk".
      
		robotProperties = Util.readProperties();

		// Is this the competition or clone robot?
   		
		if (robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Set compressor enabled switch on dashboard from properties file.
		// Later code will read that setting from the dashboard and turn 
		// compressor on or off in response to dashboard setting.
 		
		SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

		// Reset PDB & PCM sticky faults.
    
		resetFaults();

		// Create NavX object here since must done before CameraFeed is created (don't remember why).
        // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
        // generation Navx ~15 seconds for classic Navx. We assume there will be enough time between
        // power on and our first movement because normally things don't happen that fast.

		navx = NavX.getInstance(NavX.PortType.SPI);

		// Add navx as a Sendable. Updates the dashboard heading indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy stick Y axis so + values mean forward.
	  
		leftStick.invertY(true);
        rightStick.invertY(true);
		
		// Invert utility stick so pulling back is + which typically means go up.
		
		utilityStick.invertX(true);
		utilityStick.deadZoneY(.50);
		utilityStick.deadZoneX(.25);

		// We use Xbox game pad under sim instead of joysticks.
		
		if (RobotBase.isSimulation())
		{
			gamePad = new GamePad(new Joystick(GAME_PAD), "Game Pad", GamePadButtonIDs.START);

			// Invert driving joy stick Y axis so + values mean forward.
	  
			gamePad.invertY(true);  
		}

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();
		driveBase = new DriveBase();
        channel = new Channel();
        shooter = new Shooter(channel);
        pickup = new Pickup();
		climber = new Climber();

		// Create any persistent commands.

		// Set subsystem Default commands.
		
		// Set the default climb command. This command will be scheduled automatically to run
		// every teleop period and so use the utility joy stick to control the climber winch.
		// We pass in function lambda so the command can read the stick generically as a
		// DoubleProvider when it runs later (see below).
		
		climber.setDefaultCommand(new Climb(climber, () -> utilityStick.GetY()));

		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the joy sticks to drive the robot. We pass in function
		// lambda, which is like creating a DoulbleSupplier conformant class, so the command can 
		// read the sticks generically as DoubleSuppliers. This would be the same as implementing
		// the DoubleSupplier interface on the Joystick class, returning the GetY() value. The point
		// of all this is removing the direct connection between the Drive and JoyStick classes. The
		// other aspect of all this is that we are passing functions into the Drive command so it can
		// read the values later when the Drive command is executing under the Scheduler. Drive command
		// code does not have to know anything about the JoySticks (or any other source) but can still
		// read them.

		// Note that on real robot we use actual joysticks and on sim we use an XBox controller.
      
        if (RobotBase.isReal())
            driveBase.setDefaultCommand(driveCommand = new TankDrive(driveBase, () -> leftStick.GetY(), 
                                                                                () -> rightStick.GetY()));
        else
            driveBase.setDefaultCommand(driveCommand = new TankDrive(driveBase, () -> gamePad.GetLeftY(), 
                                                                                () -> gamePad.GetRightY()));
       
		// driveBase.setDefaultCommand(driveCommand = new ArcadeDrive(driveBase, 
		// 															() -> rightStick.GetY(), 
        //                                                             () -> rightStick.GetX(),
        //                                                             () -> rightStick.getJoyStick().getTrigger()));
		   
		// Start the compressor, PDP and camera feed monitoring Tasks.

   		//monitorBatteryThread = MonitorBattery.getInstance();
   		//monitorBatteryThread.start();

   		monitorCompressorThread = MonitorCompressor.getInstance(pressureSensor);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
		
   		monitorPDPThread = MonitorPDP.getInstance(pdp);
   		monitorPDPThread.start();
		
		// Start camera server thread using our class for usb cameras.
    
		cameraFeed = CameraFeed.getInstance(); 
		cameraFeed.start();
 		
		// Log info about NavX.
	  
		navx.dumpValuesToNetworkTables();
 		
		if (navx.isConnected())
			Util.consoleLog("NavX version=%s", navx.getAHRS().getFirmwareVersion());
		else
		{
			Exception e = new Exception("NavX is NOT connected!");
			Util.logException(e);
		}
        
        // Configure autonomous routines and send to dashboard.

		setAutoChoices();

		setStartingPoses();

		// Configure the button bindings for real and simulated robot.
		
        if (RobotBase.isReal()) 
            configureButtonBindings();
        else
            configureButtonBindingsSim();
        
        // Load any trajectory files in a separate thread on first scheduler run.
        // We do this because trajectory loads can take up to 10 seconds to load so we want this
        // being done while we are getting started up. Hopefully will complete before we are ready to
        // use the trajectory. See Robot22B2 for example of how to do this.

		// CANCoder test code.
		if (RobotBase.isSimulation())
		{
			//canCoder.initializeSim();
			//canCoder.reset();
		}
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     * 
     * These buttons are for real robot with 3 sticks and launchpad.
	 */
	private void configureButtonBindings() 
	{
		Util.consoleLog();
	  
		// ------- Left stick buttons --------------
		
		// ------- Right stick buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// Toggle alternate driving mode.
		new JoystickButton(rightStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TRIGGER.value)
    		.whenPressed(new InstantCommand(driveCommand::toggleAlternateDrivingMode));
		
		new JoystickButton(rightStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_RIGHT.value)
			.whenPressed(new InstantCommand(shuffleBoard::switchTab));
 
		// -------- Utility stick buttons ----------
		// Toggle extend Pickup.
		// So we show 3 ways to control the pickup. A regular command that toggles pickup state,
		// an instant command that calls a method on Pickup class that toggles state and finally
		// our special notifier variant that runs the Pickup class toggle method in a separate
		// thread. So we show all 3 methods as illustration but the reason we tried 3 methods is
		// that the pickup retraction action takes almost 1 second (due apparently to some big
		// overhead in disabling the electric eye interrupt) and triggers the global and drivebase
		// watchdogs (20ms). Threading does not as the toggle method is not run on the scheduler thread.
		// Also, any action that operates air valves, there is a 50ms delay in the ValveDA and SA
		// classes to apply power long enough so that the valve slides move far enough to open/close.
		// So any function that operates valves will trigger the watchdogs. Again, the watchdog 
		// notifications are only a warning (though too much delay on main thread can effect robot
		// operation) they can fill the Riolog to the point it is not useful.
		// Note: the threaded command can only execute a runnable (function on a class) not a Command.
		
		new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_BACK.value)
        	//.whenPressed(new PickupDeploy(pickup));		
			//.whenPressed(new InstantCommand(pickup::toggleDeploy, pickup));
			.whenPressed(new NotifierCommand(pickup::toggleDeploy, 0.0, pickup));

			new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_MIDDLE.value)
			.whenPressed(new InstantCommand(shooter::togglePID, shooter));		
		
        new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_LEFT.value)
			//.whenPressed(new InstantCommand(channel::toggleIndexerUp, channel));
			.whenPressed(new InstantCommand(channel.toggleTheIndexer(true), channel));
		
        new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TOP_RIGHT.value)
			.whenPressed(new InstantCommand(channel::toggleIndexerDown, channel));

        new JoystickButton(utilityStick.getJoyStick(), JoyStick.JoyStickButtonIDs.TRIGGER.value)
            .whenPressed(new NotifierCommand(channel::feedBall, 0.0));

		// -------- Launch pad buttons -------------
		
		// Because the launch pad buttons are wired backwards, we use whenReleased to 
		// react on button press instead of whenPressed.
        
        // Reset odometer.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_GREEN.value)
    		.whenReleased(new InstantCommand(driveBase::zeroOdometer));
        
        // Toggle shooter wheel high/low RPM.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_RED_RIGHT.value)
    		.whenReleased(new InstantCommand(shooter::toggleHighLowRPM));

    	// Reset encoders.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_RED.value)
    		.whenReleased(new InstantCommand(driveBase::resetEncoders));
	
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_YELLOW.value)
    		.whenReleased(new NotifierCommand(climber::toggleDeployMain, 0.0));
			
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.BUTTON_BLUE_RIGHT.value)
    		.whenReleased(new NotifierCommand(climber::toggleDeployAux, 0.0));
			
		// Toggle drive CAN Talon brake mode. We need to capture both sides of the rocker switch
		// to get a toggle on either position of the rocker.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_BACK.value)
			.whenPressed(new InstantCommand(driveBase::toggleCANTalonBrakeMode));
	
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_BACK.value)
    		.whenReleased(new InstantCommand(driveBase::toggleCANTalonBrakeMode));
		
		// Toggle camera feeds. We need to capture both sides of the rocker switch to get a toggle
		// on either position of the rocker.
		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_FRONT.value)
    		.whenPressed(new InstantCommand(cameraFeed::ChangeCamera));

		new JoystickButton(launchPad, LaunchPad.LaunchPadControlIDs.ROCKER_LEFT_FRONT.value)
	    	.whenReleased(new InstantCommand(cameraFeed::ChangeCamera));
	}

	/**
	 * Use this method to define your button->command mappings.  Buttons can be created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
	 * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     * 
     * These buttons are for simulated robot with Xbox controller on port 4.
	 */
	private void configureButtonBindingsSim() 
	{
        Util.consoleLog();
        
		new JoystickButton(gamePad.getJoyStick(), GamePad.GamePadButtonIDs.X.value)
    		.whenReleased(new InstantCommand(navx::resetYaw));
	
		new JoystickButton(gamePad.getJoyStick(),  GamePad.GamePadButtonIDs.Y.value)
    		.whenReleased(new InstantCommand(driveBase::resetEncoders));
	
		new JoystickButton(gamePad.getJoyStick(),  GamePad.GamePadButtonIDs.A.value)
    		//.whenReleased(new InstantCommand(climber::toggleDeployMain));
    		.whenReleased(new NotifierCommand(climber::toggleDeployMain, 0));
    }

	/**
	 * Use this to pass the autonomous command(s) to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The command to run in autonomous
	 */
	public Command getAutonomousCommand() 
	{
		AutoProgram		program = AutoProgram.NoProgram;
		Pose2d			startingPose = BLUE_1;
		Command			autoCommand = null;
		
		Util.consoleLog();

		try
		{
			program = autoChooser.getSelected();

			startingPose = startingPoseChooser.getSelected();
		}
		catch (Exception e)	{ Util.logException(e); }
		
		switch (program)
		{
			case NoProgram:
				autoCommand = null;
				break;
 				
			case DriveOut:
				autoCommand = new DriveOut(driveBase, startingPose);
				break;
 				
			case ShootFirst:
				autoCommand = new ShootFirst(driveBase, shooter, channel, startingPose);
				break;
		}
        
        // Reset motor deadband for auto.
        driveBase.setPowerDeadBand(.02);

		return autoCommand;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private static void setAutoChoices()
	{
		Util.consoleLog();
		
		autoChooser = new SendableChooser<AutoProgram>();
		
		SendableRegistry.add(autoChooser, "Auto Program");
		autoChooser.setDefaultOption("No Program", AutoProgram.NoProgram);
		autoChooser.addOption("Drive Out", AutoProgram.DriveOut);		
		autoChooser.addOption("Shoot First", AutoProgram.ShootFirst);		
				
		SmartDashboard.putData(autoChooser);
	}
  
    // Configure SendableChooser (drop down list on dashboard) with starting pose choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private static void setStartingPoses()
	{
		Util.consoleLog();
		
		startingPoseChooser = new SendableChooser<Pose2d>();
		
		SendableRegistry.add(startingPoseChooser, "Start Position");
		startingPoseChooser.setDefaultOption("Default", DEFAULT_STARTING_POSE);
		startingPoseChooser.addOption("Blue 1", BLUE_1);		
		startingPoseChooser.addOption("Blue 2", BLUE_2);		
		startingPoseChooser.addOption("Blue 3", BLUE_3);		
		startingPoseChooser.addOption("Blue 4", BLUE_4);		

		startingPoseChooser.addOption("Red 1", RED_1);		
		startingPoseChooser.addOption("Red 2", RED_2);		
		startingPoseChooser.addOption("Red 3", RED_3);		
		startingPoseChooser.addOption("Red 4", RED_4);		
				
		SmartDashboard.putData(startingPoseChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance();
  	  	location = DriverStation.getLocation();
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	/**
	 * Reset sticky faults in PDP and PCM and turn compressor on/off as
	 * set by switch on DS.
	 */
	public void resetFaults()
	{
		// This code turns on/off the automatic compressor management if requested by DS. Putting this
		// here is a convenience since this function is called at each mode change.
		if (SmartDashboard.getBoolean("CompressorEnabled", true)) 
			pcm.enableCompressorDigital();
		else
			pcm.disableCompressor();
		
		pdp.clearStickyFaults();
		pcm.clearAllStickyFaults();
    }
         
    /**
     * Loads a Pathweaver path file into a trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory.
     * @return The path's trajectory.
     */
    public static Trajectory loadTrajectoryFile(String fileName)
    {
        Trajectory  trajectory;
        Path        trajectoryFilePath;

        try 
        {
          trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("paths/" + fileName);

          Util.consoleLog("loading trajectory: %s", trajectoryFilePath);
          
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryFilePath);
        } catch (IOException ex) {
          throw new RuntimeException("Unable to open trajectory: " + ex.toString());
        }

        Util.consoleLog("trajectory loaded: %s", fileName);

        return trajectory;
    }

}
