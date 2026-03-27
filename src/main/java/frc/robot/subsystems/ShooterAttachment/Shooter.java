package frc.robot.subsystems.ShooterAttachment;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

    // Motors
    private final SparkMax shooterLeader;
    private final SparkMax shooterFollower;
    private final RelativeEncoder shooterEncoder;
    private final SparkClosedLoopController shooterPID;
    public double targetVelocity = 0;
    public boolean isturnedOff = true;
    public boolean isAutoAdjusting = false;


    // METERS (velocity in METERS/SEC)
    @SuppressWarnings("removal")
    public Shooter() {

        shooterLeader = new SparkMax(Constants.ShooterConstants.leaderShooterCanId, MotorType.kBrushless);
        shooterFollower = new SparkMax(Constants.ShooterConstants.followerShooterCanId, MotorType.kBrushless);

        shooterLeader.configure(
                Configs.Outake.leaderShooterConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        shooterFollower.configure(
                Configs.Outake.followerShooterConfig,
                SparkBase.ResetMode.kResetSafeParameters,
                SparkBase.PersistMode.kPersistParameters
        );

        shooterPID = shooterLeader.getClosedLoopController();
        shooterEncoder = shooterLeader.getEncoder();
        shooterEncoder.setPosition(0);
    }

    public void setTargetVelocity(double velocity) {
        velocity = Math.max(velocity, 0);
    }

    public void stop() {
        shooterLeader.setVoltage(0);
    }

    public double getVelocity(){
        return shooterEncoder.getVelocity();
    }


    @Override
    public void periodic() {
        shooterPID.setSetpoint(targetVelocity, SparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Shooter Vel metersPsec ", getVelocity());
        SmartDashboard.putNumber("Target Velocity", targetVelocity);
    }

}
