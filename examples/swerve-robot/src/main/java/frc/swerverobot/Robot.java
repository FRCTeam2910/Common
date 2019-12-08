package frc.swerverobot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.swerverobot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
    private static final OI oi = new OI();

    private SubsystemManager subsystemManager = new SubsystemManager(
            DrivetrainSubsystem.getInstance()
    );

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();

        oi.bindButtons();

        subsystemManager.enableKinematicLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        subsystemManager.outputToSmartDashboard();

        Scheduler.getInstance().run();
    }
}
