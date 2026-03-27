package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeRollers extends SubsystemBase {

    // Motors
    private final SparkMax leaderForwardRoller;
    private final SparkMax followerBackwardRoller;
    @SuppressWarnings("removal")
    public IntakeRollers() {

        leaderForwardRoller = new SparkMax(Constants.IntakeConstants.leaderRollerCanId, MotorType.kBrushless);
        followerBackwardRoller = new SparkMax(Constants.IntakeConstants.followRollerCanId, MotorType.kBrushless);

        leaderForwardRoller.configure(
            Configs.Intake.leaderRollerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        followerBackwardRoller.configure(
            Configs.Intake.followerRollerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );
    }

    // =========================
    // Roller Control
    // =========================

    public void intakeIn() {
        leaderForwardRoller.set(1);
        followerBackwardRoller.set(1);
    }

    public void intakeOut() {
        leaderForwardRoller.set(-1);
        followerBackwardRoller.set(0);
    }

    public void stop() {
        leaderForwardRoller.set(0);
        followerBackwardRoller.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Velocity", leaderForwardRoller.getEncoder().getVelocity());
    }

}
