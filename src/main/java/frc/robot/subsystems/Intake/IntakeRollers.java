package frc.robot.subsystems.Intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class IntakeRollers extends SubsystemBase {

    // Motors
    private final SparkMax groundRoller;
   // private final SparkMax topRoller;
    @SuppressWarnings("removal")
    public IntakeRollers() {

        groundRoller = new SparkMax(13, MotorType.kBrushless);
        //topRoller    = new SparkMax(15, MotorType.kBrushless);

        groundRoller.configure(
            Configs.Intake.rollerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        /*topRoller.configure(
            Configs.Intake.rollerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );*/

        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );
    }

    // =========================
    // Roller Control
    // =========================

    public void intakeIn() {
        groundRoller.set(0.8);
        //topRoller.set(0.8);
    }

    public void intakeOut() {
        groundRoller.set(-0.8);
        //topRoller.set(-0.8);
    }

    public void stop() {
        groundRoller.set(0);
        //topRoller.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Roller Velocity", groundRoller.getAbsoluteEncoder().getVelocity());
    }

}
