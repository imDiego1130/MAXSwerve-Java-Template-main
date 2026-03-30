package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveControl;

public class LimelightAutoTargetCommand extends Command {
    private final Limelight limelight;
    private final DriveControl driverControl;

    public  LimelightAutoTargetCommand(Limelight limelight, DriveControl driverControl) {
        this.limelight = limelight;
        this.driverControl = driverControl;

        addRequirements(limelight);
    }

    @Override
    public void execute() {
        limelight.setTargetPose(driverControl.getCurrentObjectivePose(limelight.getCurrentPose()));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
