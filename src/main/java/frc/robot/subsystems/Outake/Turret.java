package frc.robot.subsystems.Outake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class Turret extends SubsystemBase {

    // Motors
    private final SparkMax turret;
    private final RelativeEncoder turretEncoder;
    private final SparkClosedLoopController turretPID;
    // IN DEGREES
    // + in position is cw rotation
    private final double MIN_ANGLE = -180;
    private final double MAX_ANGLE = 0;
    private boolean isMaintainingHeading = false;
    private double trackAroundHeading = 0;
    private double target = 0;
    private AHRS m_gyro;

    @SuppressWarnings("removal")
    public Turret(AHRS gyro) {

        turret = new SparkMax(18, MotorType.kBrushless);

        turret.configure(
                Configs.Outake.turretConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        turretPID = turret.getClosedLoopController();
        turretEncoder = turret.getEncoder();
        turretEncoder.setPosition(0);

        m_gyro = gyro;

        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );

    }

    public void turnToPosition(double degrees) {
        target = degrees;
        runTurret(degrees);
    }

    public void turnToPosition(){
        runTurret(target);
    }

    public void maintainHeading(boolean maintain){
        isMaintainingHeading = maintain;
        trackAroundHeading = -m_gyro.getYaw();
    }

    public void maintainHeading(boolean maintain, double headingToTrack){
        isMaintainingHeading = maintain;
        trackAroundHeading = headingToTrack;
    }

    private void runTurret(double target){
        if (isMaintainingHeading && !Double.isNaN(trackAroundHeading)) {
            target += (trackAroundHeading - (-m_gyro.getYaw()));
        }

        target = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, target));

        turretPID.setSetpoint(target, SparkMax.ControlType.kPosition);
    }

    public void stop() {
        turret.set(0);
    }

    public boolean isMaintainingHeading(){
        return isMaintainingHeading;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position (Degrees): ", turretEncoder.getPosition());
    }

}
