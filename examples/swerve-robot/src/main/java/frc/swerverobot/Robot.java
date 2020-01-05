package frc.swerverobot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.swerverobot.subsystems.DrivetrainSubsystem;
import org.frcteam2910.common.robot.UpdateManager;

public class Robot extends TimedRobot {
    private static final OI oi = new OI();

    private UpdateManager updateManager = new UpdateManager(
            DrivetrainSubsystem.getInstance()
    );

    public static OI getOi() {
        return oi;
    }

    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();

        oi.bindButtons();

        updateManager.startLoop(5.0e-3);
    }

    @Override
    public void robotPeriodic() {
        Scheduler.getInstance().run();
    }
}
