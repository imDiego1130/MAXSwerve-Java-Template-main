package frc.robot.subsystems.Outake.TurretAttachment;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveControl;

public class ManualTurretCommand extends Command {
    private final Turret turret;
    private final DriveControl driverControl;

    public  ManualTurretCommand(Turret turret, DriveControl driverControl) {
        this.turret = turret;
        this.driverControl = driverControl;

        addRequirements(turret);
    }

    @Override
    public void execute() {

        if (turret.isTrackingPosition) {

        }

        Rotation2d joystickAngle = driverControl.getJoyStickAngle();

        if (joystickAngle == null) {
            turret.holdCurrentPosition();
            return;
        }

        turret.setTargetPosition(joystickAngle.getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
