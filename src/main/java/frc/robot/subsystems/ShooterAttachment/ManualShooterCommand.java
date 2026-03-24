package frc.robot.subsystems.ShooterAttachment;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveControl;

public class ManualShooterCommand extends Command {
    private final Shooter shooter;
    private final DriveControl driverControl;

    public  ManualShooterCommand(Shooter shooter, DriveControl driverControl) {
        this.shooter = shooter;
        this.driverControl = driverControl;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        if (shooter.isturnedOff) {
            shooter.stop();
            return;
        }

        if (shooter.isAutoAdjusting) {
            shooter.setTargetVelocity(driverControl.getCorrelatedVelocity());
            return;
        }

        shooter.setTargetVelocity(shooter.getVelocity() + driverControl.getOperatorRightY());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
