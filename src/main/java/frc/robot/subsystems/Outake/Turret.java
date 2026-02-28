package frc.robot.subsystems.Outake;

import com.revrobotics.RelativeEncoder;
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
        turret.set(power);
    }

    public void turnWithPosition(double rotations) {
        turretPID.setSetpoint(rotations, SparkMax.ControlType.kVelocity);
    }

    public void stop() {
        turret.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position (Rotations): ", turretEncoder.getPosition());
    }

}
