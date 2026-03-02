package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Spindexer extends SubsystemBase {

    // Motors
    private final SparkMax spindexer;
    @SuppressWarnings("removal")
    public Spindexer() {

        spindexer = new SparkMax(20, MotorType.kBrushless);

        spindexer.configure(
            Configs.Spindexer.spindexerConfig,
            SparkMax.ResetMode.kResetSafeParameters,
            SparkMax.PersistMode.kPersistParameters
        );

        
        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );
    
    }

    public void spinClockwise(double power) {
        power = Math.abs(power);
        power = Math.min(power, 0.8);
        spindexer.set(-power);
    }

    public void spinCounterClockwise(double power) {
        power = Math.abs(power);
        power = Math.min(power, 0.8);
        spindexer.set(power);
    }

    public void stop() {
        spindexer.set(0);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Spindexer RPMs: ", spindexer.getEncoder().getVelocity());
    }

}
