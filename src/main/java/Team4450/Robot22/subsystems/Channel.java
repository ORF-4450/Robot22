package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import Team4450.Lib.Util;
import Team4450.Robot22.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Channel subsystem.
 */
public class Channel extends SubsystemBase
{
	private boolean			indexerRunning;
  	
    private WPI_VictorSPX   indexerMotor = new WPI_VictorSPX(INDEXER_VICTOR);
      
    private double          defaultPower = .30;

	public Channel()
	{
        indexerMotor.setInverted(true);

        stopIndexer();

		Util.consoleLog("Channel created!");
	}
	
	// This method will be called once per scheduler run
	@Override
	public void periodic() 
	{
	}

	private void updateDS()
	{
		SmartDashboard.putBoolean("Indexer", indexerRunning);
	}

	/**
	 * Stop belt.
	 */
	public void stopIndexer()
	{
		Util.consoleLog();
        
        indexerMotor.stopMotor();
		
		indexerRunning = false;
		
		updateDS();
	}

    /**
	 * Start belt.
	 * @param power Power level -1.0 to 1.0. + is ball IN.
	 */
	public void startIndexer(double power)
	{
		Util.consoleLog("%.2f", power);
		
		indexerMotor.set(power);
		
		indexerRunning = true;
		
		updateDS();
    }
    
    /**
     * Start index wheel up with default power.
     */
    public void startIndexer()
    {
        startIndexer(defaultPower);
    }
    
    /**
     * Toggles indexer on/off.
     * @param power Power level to use when starting wheel -1.0 to 1.0.
     * + is ball UP.
     * @return True if result is wheel on, false if off.
    */
    public boolean toggleIndexer(double power)
    {
        if (isRunning())
            stopIndexer();
        else
            startIndexer(power);

        return isRunning();
    }
    
    /**
     * Toggles indexer wheel on/off. Uses default + power level when turning on. Forward
     * is ball UP.
     * @return True if result is belt on, false if off.
    */
    public boolean toggleIndexerUp()
    {
        return toggleIndexer(defaultPower);
    }
   
    /**
     * Toggles indexer wheelt on/off. Uses default - power level when turning on. Backward
     * is ball DOWN.
     * @return True if result is belt on, false if off.
     */
    public boolean toggleIndexerDown()
    {
        return toggleIndexer(-defaultPower);
    }
    
    /**
     * This is an example of how to pass parameters to a runnable. The two funtions above
     * could be done on one, with a parameter for direction or perhaps power. Both ways
     * are legitimate but this shows how to pass a parameter to a runnable if a case
     * surfaces that needs a parameter. See RobotContainer button config method.
     * @param up Direction of wheel rotation. True is up, false is down.
     * @return A runnable object suitable for passing to an InstantCommand.
     */
    public Runnable toggleTheIndexer(boolean up)
    {
        Runnable aRunnable = new Runnable() {
            public void run()
            {
                if (up)
                    toggleIndexerUp();
                else
                    toggleIndexerDown();
            }
        };
    
        return aRunnable;
    }
	/**
	 * Returns running state of wheel.
	 * @return True is running.
	 */
	public boolean isRunning()
	{
		return indexerRunning;
    }

    /**
     * Run the indexer wheel just long enough to pop the top ball
     * into the shooter wheel. This function should be run from
     * a NotifierCommand so it is run in a separate thread.
     */
    public void feedBall()
    {
        Util.consoleLog();

        // Can't feed a ball if shooter wheel is not running.
        if  (!RobotContainer.shooter.isRunning()) return;

        startIndexer();

        Timer.delay(.50);   // set time so one ball is fed.

        stopIndexer();
    }
}
