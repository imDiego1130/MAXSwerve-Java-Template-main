package frc.robot.subsystems.Outake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Feeder extends SubsystemBase {

    // Motors
    private final SparkMax feeder;
    @SuppressWarnings("removal")
    public Feeder() {

        feeder = new SparkMax(21, MotorType.kBrushless);

        feeder.configure(
                Configs.Outake.feedgerConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        /* 
        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );
        */
    }

    public void feedIn() {
        feeder.set(-1);
    }

    public void feedOut() {
        feeder.set(1);
    }

    public void stop() {
        feeder.set(0);
    }

    public double getRPM(){
        return feeder.getEncoder().getVelocity();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Feeder RPM: ", feeder.getEncoder().getVelocity());
    }

}
