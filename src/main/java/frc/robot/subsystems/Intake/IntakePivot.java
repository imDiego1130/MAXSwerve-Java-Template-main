package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {

    // Motors
    private final SparkMax pivotMotor;

    // Encoder + PID (ONLY for pivot)
    private final RelativeEncoder pivotEncoder;
    private final SparkClosedLoopController pivotPID;

    // Preset Positions
    // in deg
    private static final double UP_POSITION = 25;
    private static final double DOWN_POSITION = 78;
    private double targetPosition = 0;

    @SuppressWarnings("removal")
    public IntakePivot() {
        pivotMotor   = new SparkMax(Constants.PivotConstants.pivotCanId, MotorType.kBrushless);

        pivotMotor.configure(
                Configs.Intake.pivotConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        pivotPID = pivotMotor.getClosedLoopController();
        pivotEncoder = pivotMotor.getEncoder();
        pivotEncoder.setPosition(0);
    }

    // =========================
    // Pivot Control (Position)
    // =========================

    public void raise() {
        targetPosition = UP_POSITION;
    }

    public void lower() {
        targetPosition = DOWN_POSITION;
    }

    public double getPivotPosition() {
        return pivotEncoder.getPosition();
    }

    public void stop() {
        pivotMotor.setVoltage(0);
    }

    @Override
    public void periodic() {
        pivotPID.setSetpoint(targetPosition, SparkMax.ControlType.kPosition);
        SmartDashboard.putNumber("Pivot Deg: ", getPivotPosition());
    }

}
