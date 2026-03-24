package frc.robot.subsystems.Outake.TurretAttachment;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DriveControl;

public class AutoTurretCommand extends Command {
    private final Turret turret;
    private final DriveControl driverControl;

    public  AutoTurretCommand(Turret turret, DriveControl driverControl) {
        this.turret = turret;
        this.driverControl = driverControl;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setTargetPosition(driverControl.getVisionAngle());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
