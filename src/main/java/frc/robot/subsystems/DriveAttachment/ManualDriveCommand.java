package frc.robot.subsystems.DriveAttachment;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveControl;

public class ManualDriveCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final DriveControl driverControl;

    public  ManualDriveCommand(DriveSubsystem driveSubsystem, DriveControl driverControl) {
        this.driveSubsystem = driveSubsystem;
        this.driverControl = driverControl;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        driveSubsystem.drive(
                  driverControl.getForward()
                , driverControl.getStrafe()
                , driverControl.getRotate()
                , false
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
