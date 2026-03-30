package frc.robot.subsystems.ShooterAttachment;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveControl;

public class AutoSetShooterCommand extends Command {
    private final Shooter shooter;
    private final DriveControl driverControl;

    public  AutoSetShooterCommand(Shooter shooter, DriveControl driverControl) {
        this.shooter = shooter;
        this.driverControl = driverControl;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.targetVelocity = 17;
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
