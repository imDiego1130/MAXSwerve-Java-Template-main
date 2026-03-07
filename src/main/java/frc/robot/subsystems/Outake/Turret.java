package frc.robot.subsystems.Outake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Turret extends SubsystemBase {

    // Motors
    private final SparkMax turret;
    private final RelativeEncoder turretEncoder;
    private final SparkClosedLoopController turretPID;
    // IN DEGREES
    // + in position is cw rotation
    private final double MIN_ANGLE = -180;
    private final double MAX_ANGLE = 0;

    @SuppressWarnings("removal")
    public Turret() {

        turret = new SparkMax(20, MotorType.kBrushless);

        turret.configure(
                Configs.Outake.turretConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        turretPID = turret.getClosedLoopController();
        turretEncoder = turret.getEncoder();
        turretEncoder.setPosition(0);

        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );

    }

    public void turnWithPower(double power) {
        double pos =  turretEncoder.getPosition();
        if (pos > MAX_ANGLE ||  pos < MIN_ANGLE) {
            power = 0;
        }

        turret.set(power);
    }

    public void turnWithPosition(double degrees) {
        double clampedDegrees = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, degrees));
        turretPID.setSetpoint(clampedDegrees, SparkMax.ControlType.kPosition);
    }

    public void stop() {
        turret.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position (Degrees): ", turretEncoder.getPosition());
    }

}
