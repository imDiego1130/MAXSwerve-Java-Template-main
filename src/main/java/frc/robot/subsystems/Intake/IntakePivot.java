package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakePivot extends SubsystemBase {

    // Motors
    private final SparkMax pivotMotor;

    // Encoder + PID (ONLY for pivot)
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPID;

    // Preset Positions
    // in deg
    private static final double UP_POSITION = 20;
    private static final double DOWN_POSITION = 50;

    @SuppressWarnings("removal")
    public IntakePivot() {
        pivotMotor   = new SparkMax(14, MotorType.kBrushless);

        pivotMotor.configure(
                Configs.Intake.pivotConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        pivotPID = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0);

        setDefaultCommand(
                new RunCommand(() -> {}, this)
        );
    }

    // =========================
    // Pivot Control (Position)
    // =========================

    public void raise() {
        pivotPID.setSetpoint(UP_POSITION, SparkMax.ControlType.kPosition);
    }

    public void lower() {
        pivotPID.setSetpoint(DOWN_POSITION, SparkMax.ControlType.kPosition);
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public void stop() {
        pivotMotor.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Pivot Deg: ", getPivotPosition());
        SmartDashboard.putNumber("Pivot Target: ", pivotPID.getSetpoint());
    }

}
