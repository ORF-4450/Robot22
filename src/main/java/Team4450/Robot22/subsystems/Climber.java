package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase
{
	  
	private WPI_VictorSPX			climberFrontVictor, climberBackVictor;

	private MotorControllerGroup	climberDrive;

	//private ValveDA			    climberBrake = new ValveDA(CLIMBER_BRAKE_VALVE);
	private DigitalInput	        climberSwitch = new DigitalInput(CLIMBER_SWITCH);

	// Encoder (regular type) is plugged into dio port n:
	// orange=+5v blue=signal, dio port n+1: black=gnd yellow=signal. 
	private Encoder			climberEncoder = new Encoder(CLIMBER_ENCODER, CLIMBER_ENCODER + 1, true, EncodingType.k4X);

	private boolean			brakeEngaged;
	
	public Climber()
	{
		Util.consoleLog();
		
		climberFrontVictor = new WPI_VictorSPX(LEFT_CLIMBER_VICTOR);
		climberBackVictor = new WPI_VictorSPX(RIGHT_CLIMBER_VICTOR);
	      
		climberFrontVictor.setInverted(true);
	      
	    climberFrontVictor.setNeutralMode(NeutralMode.Brake);
	    climberBackVictor.setNeutralMode(NeutralMode.Brake);
	    //hookVictor.setNeutralMode(NeutralMode.Brake);

	    climberDrive = new MotorControllerGroup(climberFrontVictor, climberBackVictor);

		climberEncoder.reset();
		
		releaseBrake();
		
		Util.consoleLog("Climber created!");
	}
	
	@Override
	public void periodic() 
	{
		// This method will be called once per scheduler run
	}
	
	/**
	 * Set power level for climber motors.
	 * @param power -1 to +1, + is up because we pull the stick back to climb.
	 */
	public void setClimberPower(double power)
	{
		// If trying to go down (-) and switch returns true, we are at bottom so kill the power.
		
		if (power < 0 && climberSwitch.get()) 
		{
			climberEncoder.reset();
			power = 0;
		}
		
		// If trying to go up (+) and encoder is at upper limit count, we are the top kill the power.
		if (power > 0 && climberEncoder.get() >= 5300) power = 0;

		climberDrive.set(power);
	}
	
	/**
	 * Stops climber motors.
	 */
	public void stop()
	{
		Util.consoleLog();
		
		climberDrive.stopMotor();
	}
	
	/**
	 * Engage the climber brake.
	 */
	public void engageBrake()
	{
		Util.consoleLog();
		
		//climberBrake.SetA();
		
		brakeEngaged = true;
		
		updateDS();
	}
	
	/**
	 * Release the climber brake.
	 */
	public void releaseBrake()
	{
		Util.consoleLog();
		
		//climberBrake.SetB();
		
		brakeEngaged = false;
		
		updateDS();
	}
	
	/**
	 * Returns state of climber brake.
	 * @return True if  brake engaged.
	 */
	public boolean isBrakeEngaged()
	{
		return brakeEngaged;
	}
	
	/**
	 * Toggle state of climber brake.
	 */
	public void toggleBrake()
	{
		Util.consoleLog();
		
		if (brakeEngaged)
			releaseBrake();
		else
			engageBrake();
	}

	private void updateDS()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Brake", brakeEngaged);
	}

}
