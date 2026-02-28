package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.geometry.Rotation2d;
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

    // Preset Positions (example values — you must tune)
    // in deg (we divide by 360 because the motor still talks in rotations)
    private static final double UP_POSITION = (27.428544) /360;
    private static final double DOWN_POSITION = (UP_POSITION + 90) /360;

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
                new RunCommand(() -> stop(), this)
        );
    }

    // =========================
    // Pivot Control (Position)
    // =========================

    //@SuppressWarnings("removal")
    public void raise() {
        pivotPID.setSetpoint(UP_POSITION, SparkMax.ControlType.kPosition);
    }

    //@SuppressWarnings("removal")
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
        SmartDashboard.putNumber("Pivot Deg: ", getPivotPosition()*360);
    }

}
