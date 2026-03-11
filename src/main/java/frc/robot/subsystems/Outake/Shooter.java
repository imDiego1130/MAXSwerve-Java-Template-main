package frc.robot.subsystems.Outake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Shooter extends SubsystemBase {

    // Motors
    private final SparkMax shooter;
    private final RelativeEncoder shooterEncoder;
    private final SparkClosedLoopController shooterPID;
    private double targetVelocity = 0;
    public boolean isturnedOff = false;
    // METERS (velocity in METERS/SEC)
    @SuppressWarnings("removal")
    public Shooter() {

        shooter = new SparkMax(19, MotorType.kBrushless);

        shooter.configure(
                Configs.Outake.shooterConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        shooterPID = shooter.getClosedLoopController();
        shooterEncoder = shooter.getEncoder();
        shooterEncoder.setPosition(0);

        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );

    }

    public void spinWithPower(double power) {
        shooter.set(power);
    }

    public void spinWithVelocity(double velocity) {
        targetVelocity = velocity;
        shooterPID.setSetpoint(targetVelocity, SparkMax.ControlType.kVelocity);
    }

    public void stop() {
        shooter.set(0);
    }

    public double getPower(){
        return shooter.get();
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Vel (M/s) ", shooterEncoder.getVelocity());
    }

}
