package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Intake extends SubsystemBase {

    // Motors
    private final SparkMax groundRoller;
    private final SparkMax pivotMotor;
    private final SparkMax topRoller;

    // Encoder + PID (ONLY for pivot)
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPID;

    // Preset Positions (example values — you must tune)
    private static final double UP_POSITION = 0.0;
    private static final double DOWN_POSITION = 25.0;

    @SuppressWarnings("removal")
    public Intake() {

        groundRoller = new SparkMax(13, MotorType.kBrushless);
        pivotMotor   = new SparkMax(14, MotorType.kBrushless);
        topRoller    = new SparkMax(15, MotorType.kBrushless);

        groundRoller.configure(
            Configs.Intake.rollerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        topRoller.configure(
            Configs.Intake.rollerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        pivotMotor.configure(
            Configs.Intake.pivotConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        pivotPID = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();
        raise();
    }

    // =========================
    // Roller Control
    // =========================

    public void intakeIn() {
        groundRoller.set(0.8);
        topRoller.set(0.8);
    }

    public void intakeOut() {
        groundRoller.set(-0.8);
        topRoller.set(-0.8);
    }

    public void stopRollers() {
        groundRoller.set(0);
        topRoller.set(0);
    }

    // =========================
    // Pivot Control (Position)
    // =========================

    @SuppressWarnings("removal")
    public void raise() {
        pivotPID.setReference(UP_POSITION, SparkMax.ControlType.kPosition);
    }

    @SuppressWarnings("removal")
    public void lower() {
        pivotPID.setReference(DOWN_POSITION, SparkMax.ControlType.kPosition);
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public void stopAll() {
        stopRollers();
        pivotMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Position", pivotEncoder.getPosition());
    }

}
