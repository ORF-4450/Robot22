package Team4450.Robot22.subsystems;

import static Team4450.Robot22.Constants.*;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Robot22.RobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class hosts functions relating to communicating with the ShuffleBoard driver
 * station application. Primarily, it's periodic function handles the regular update
 * of the "LCL" panel's display of robot status information when the robot is active.
 */
public class ShuffleBoard extends SubsystemBase
{
    public int      currentTab, numberOfTabs = 2;

	public ShuffleBoard()
	{
		Util.consoleLog("ShuffleBoard created!");
	}
	
	// This method will be called once per scheduler run.
	@Override
	public void periodic() 
	{    
        LCD.printLine(LCD_3, "leftenc=%d  rightenc=%d", RobotContainer.driveBase.getLeftEncoder(), 
                      RobotContainer.driveBase.getRightEncoder());			
                
        LCD.printLine(LCD_4, "utilY=%.3f  utilX=%.3f  climberEnc=%d  climbswitch=%b", RobotContainer.utilityStick.GetY(), 
                      RobotContainer.utilityStick.GetX(), RobotContainer.climber.encoderGet(), 
                      RobotContainer.climber.getSwitch());
    
        LCD.printLine(LCD_7, "Lrpm=%d - Rrpm=%d  Lmax vel=%.3f - Rmax vel=%.3f", 
                      RobotContainer.driveBase.leftEncoder.getRPM(),
                      RobotContainer.driveBase.rightEncoder.getRPM(), 
                      RobotContainer.driveBase.leftEncoder.getMaxVelocity(PIDRateType.velocityMPS),
                      RobotContainer.driveBase.rightEncoder.getMaxVelocity(PIDRateType.velocityMPS));
      
        Pose2d pose = RobotContainer.driveBase.getOdometerPose();
      
        LCD.printLine(LCD_8, "pose x=%.1fm (lrot=%.2f)  y=%.1fm  deg=%.1f  yaw=%.1f", pose.getX(), 
                      RobotContainer.driveBase.leftEncoder.getRotations(), pose.getY(), pose.getRotation().getDegrees(),
                      RobotContainer.navx.getYaw());
                      
        LCD.printLine(LCD_9, "shooter rpm=%.0f  max=%.0f  ball switch=%b  ball eye=%d", RobotContainer.shooter.getRPM(), 
                      RobotContainer.shooter.getMaxRPM(), RobotContainer.channel.getBallStopSwitch(), 
                      RobotContainer.channel.getBallStartSensor());  
    }

    /**
     * Reset the shuffleboard indicators to disabled states.
     */
    public void resetLEDs()
    {
        SmartDashboard.putBoolean("Disabled", true);
        SmartDashboard.putBoolean("Auto Mode", false);
        SmartDashboard.putBoolean("Teleop Mode", false);
        SmartDashboard.putBoolean("FMS", DriverStation.isFMSAttached());
        SmartDashboard.putBoolean("Overload", false);
        SmartDashboard.putNumber("AirPressure", 0);
        SmartDashboard.putBoolean("AltDriveMode", false);
        SmartDashboard.putBoolean("SteeringAssist", false);
        SmartDashboard.putBoolean("Brake", false);
        //SmartDashboard.putBoolean("TargetLocked", false);
    }

    /**
     * Switch tab on shuffleboard display by rotating through the tabs.
     * @return The new tab index (0-based).
     */
    public int switchTab()
    {
        currentTab++;

        if (currentTab > (numberOfTabs - 1)) currentTab = 0;

        Util.consoleLog("%d", currentTab);

        Shuffleboard.selectTab(currentTab);

        return currentTab;
    }

    /**
     * Switch tab on shuffleboard display by tab name. Will create the tab if
     * it does not already exist.
     * @param tabName The name of the tab to select.
     * @return The selected tab object.
     */
    public ShuffleboardTab switchTab(String tabName)
    {
        Util.consoleLog("%s", tabName);

        return Shuffleboard.getTab(tabName);
    }
}
