package Team4450.Robot26.subsystems;

import Team4450.Lib.Util;
import Team4450.Robot26.RobotContainer;
import Team4450.Robot26.commands.Utility.NotifierCommand;

import static Team4450.Robot26.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class hosts functions relating to communicating with the ShuffleBoard driver
 * station application. Primarily, it's periodic function handles the regular update
 * of the robot status information when the robot is active.
 */
public class ShuffleBoard extends SubsystemBase {
    public int                  currentTab, numberOfTabs = 3;

    private NotifierCommand     updateCommand;
    private Notifier            notifier;

	public ShuffleBoard() {
        // We use a NotifierCommand to run the DS update process in a separate thread
        // from the main thread. We set that command as the default command for this
        // subsystem so the scheduler starts the command. After start, the notifier
        // runs all the time updating the DS every 25ms which is slightly slower than
        // the main thread update period.
        updateCommand = new NotifierCommand(this::updateDS, .025, "SB", this);

        this.setDefaultCommand(updateCommand);

		Util.consoleLog("ShuffleBoard created!");
	}

	// This method will be called once per scheduler run on the scheduler (main) thread. Only
    // used if not running the updateDS with the notifier.
	@Override
	public void periodic() {
        //updateDS();
    }

    /**
     * Shuffleboard telemetry. Do not call if this class is running in it's
     * own thread.
     */
    public void updateDS() {    
        // Pose2d pose = RobotContainer.drivebase.getPose(); 
    }

    /**
     * Reset the shuffleboard indicators to disabled states. Runs in
     * a separate thread.
     */
    public void resetLEDs() {
        // Notifier runs the reset function in a separate thread.
        notifier = new Notifier(this::resetLEDIndicators);
        notifier.startSingle(0);
    }

    /**
     * Reset the Shuffleboard indicators to diabled states. Runs on
     * main thread.
     */
    private void resetLEDIndicators() {
        Util.consoleLog();
        
        SmartDashboard.putBoolean("Disabled", true);
        SmartDashboard.putBoolean("Auto Mode", false);
        SmartDashboard.putBoolean("Teleop Mode", false);
        SmartDashboard.putBoolean("FMS", DriverStation.isFMSAttached());
        SmartDashboard.putBoolean("Autonomous Active", false);
        SmartDashboard.putBoolean("Tracking", false);
    }

    /**
     * Switch tab on shuffleboard display by rotating through the tabs.
     * @return The new tab index (0-based).
     */
    public int switchTab() {
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
    public ShuffleboardTab switchTab(String tabName) {
        Util.consoleLog("%s", tabName);

        return Shuffleboard.getTab(tabName);
    }
}
