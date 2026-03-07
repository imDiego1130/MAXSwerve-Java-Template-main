package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkParameters;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Climber extends SubsystemBase {

    // Motors
    private final SparkMax climberMotor;

    // Encoder + PID (ONLY for pivot)
    private final RelativeEncoder climberEncoder;
    private final SparkClosedLoopController climberPID;

    // Preset Positions
    // in METERS
    private static final double UP_POSITION = 0.4;
    private static final double MID_POSITION = 0.2;
    private static final double DOWN_POSITION = 0;

    @SuppressWarnings("removal")
    public Climber() {
        climberMotor   = new SparkMax(19, MotorType.kBrushless);

        climberMotor.configure(
                Configs.Climber.climberConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        climberPID = climberMotor.getClosedLoopController();
        climberEncoder = climberMotor.getEncoder();
        climberEncoder.setPosition(0);

        setDefaultCommand(
                new RunCommand(() -> {}, this)
        );
    }

    // =========================
    // Pivot Control (Position)
    // =========================

    public void goToMaximumPosition() {
        climberPID.setSetpoint(UP_POSITION, SparkMax.ControlType.kPosition);
    }

    public void goToMediumPosition() {
        climberPID.setSetpoint(MID_POSITION, SparkMax.ControlType.kPosition);
    }

    public void goToMinimumPosition() {
        climberPID.setSetpoint(DOWN_POSITION, SparkMax.ControlType.kPosition);
    }

    public double getPivotPosition() {
        return climberEncoder.getPosition();
    }

    public void stop() {
        climberMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Deg: ", getPivotPosition());
        SmartDashboard.putNumber("Pivot Target: ", climberPID.getSetpoint());
    }

}
