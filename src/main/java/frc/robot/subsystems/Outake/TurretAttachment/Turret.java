package frc.robot.subsystems.Outake.TurretAttachment;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class Turret extends SubsystemBase {

    // Motors
    private final SparkMax turret;
    private final RelativeEncoder turretEncoder;
    private final SparkClosedLoopController turretPID;
    // IN DEGREES
    // + in position is cw rotation
    private final double MIN_ANGLE = -90;
    private final double MAX_ANGLE = 90;
    public boolean isTrackingPosition = false;
    private double target = -90;
    private DoubleSupplier visionAngleSupplier;
    @SuppressWarnings("removal")
    public Turret() {

        turret = new SparkMax(Constants.TurretConstants.turretCanId, MotorType.kBrushless);

        turret.configure(
                Configs.Outake.turretConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        turretPID = turret.getClosedLoopController();
        turretEncoder = turret.getEncoder();
        turretEncoder.setPosition(-90);
    }

    public void setTargetPosition(double degrees) {
        target = degrees;
        target = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, target));
    }

    public void holdCurrentPosition() {
        target = getPosition();
    }

    public double getPosition(){
        return turretEncoder.getPosition();
    }

    @Override
    public void periodic() {
        turretPID.setSetpoint(target,  SparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Turret Position (Degrees) ", turretEncoder.getPosition());
        SmartDashboard.putBoolean("Turret Tracking Target", isTrackingPosition);
    }

}
