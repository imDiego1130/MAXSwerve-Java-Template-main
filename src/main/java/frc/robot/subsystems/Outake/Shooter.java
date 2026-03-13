package frc.robot.subsystems.Outake;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

import java.util.ArrayList;
import java.util.Arrays;

public class Shooter extends SubsystemBase {

    // Motors
    private final SparkMax shooter;
    private final RelativeEncoder shooterEncoder;
    private final SparkClosedLoopController shooterPID;
    private double targetVelocity = 0;
    public boolean isturnedOff = false;
    public boolean isAutoAdjusting = false;
    // plotted points
    private final ArrayList<double[]> dataPoints = new ArrayList<>(Arrays.asList(
            // 0 : distance (meters), 1 : velocity (m/s), 2 : angle (deg)
             new double[]{3.2 , 20.7, 0.0 }
            ,new double[]{4.36, 23.9, 0.0 }
            //,new double[]{5.5 , 25.7, 0.0 }
    ));

    private final double[] velocitySlopeList = initializeSlopeList(1);


    // METERS (velocity in METERS/SEC)
    @SuppressWarnings("removal")
    public Shooter() {

        shooter = new SparkMax(19, MotorType.kBrushless);

        shooter.configure(
                Configs.Outake.shooterConfig,
                SparkMax.ResetMode.kResetSafeParameters,
                SparkMax.PersistMode.kPersistParameters
        );

        shooterPID = shooter.getClosedLoopController();
        shooterEncoder = shooter.getEncoder();
        shooterEncoder.setPosition(0);

        setDefaultCommand(
                new RunCommand(() -> stop(), this)
        );

    }

    private double[] initializeSlopeList(int valueIndex){
        int maxIndex = dataPoints.size()-1;
        double[] list = new double[maxIndex];
        for (int i=0; i<maxIndex; i++){
            list[i] = (
                    (dataPoints.get(i+1)[valueIndex] - dataPoints.get(i)[valueIndex]) /
                            (dataPoints.get(i+1)[0] - dataPoints.get(i)[0])
            );
        }

        return list;
    }

    private double getRegressionValue(double distance, int valueIndex){
        double[] slopeList;
        slopeList = velocitySlopeList;
        
        double x0 = dataPoints.get(0)[0];
        if (distance <= x0) {
            double y0 = dataPoints.get(0)[valueIndex];
            double m0 = slopeList[0];
            return y0 + (distance - x0)*m0;
        }

        for (int i=0; i < dataPoints.size()-1; i++){
            double xi = dataPoints.get(i+1)[0];
            if (distance <= xi) {
                double yi = dataPoints.get(i)[valueIndex];
                double mi = slopeList[i];
                return yi + (distance - dataPoints.get(i)[0])*mi;
            }
        }

        double xf = dataPoints.get(dataPoints.size() -1)[0];
        double yf = dataPoints.get(dataPoints.size() -1)[valueIndex];
        double mf = slopeList[dataPoints.size() -2];
        return yf + (distance - xf)*mf;
    }

    public void spinWithPower(double power) {
        shooter.set(power);
    }

    public void spinWithVelocity(double velocity) {
        targetVelocity = velocity;
        shooterPID.setSetpoint(targetVelocity, SparkMax.ControlType.kVelocity);
    }

    public void stop() {
        shooter.set(0);
    }

    public double getPower(){
        return shooter.get();
    }

    public double getTargetVelocity(){
        return targetVelocity;
    }

    public double getVelocity(){
        return shooterEncoder.getVelocity();
    }

    public void autoAdjustSpeed(double distance){
        spinWithVelocity(getRegressionValue(distance, 1));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Vel metersPsec ", getVelocity());
        SmartDashboard.putBoolean("Shooter Auto Adjusting", isAutoAdjusting);
        SmartDashboard.putBoolean("Shooter Toggled On", !isturnedOff);
    }

}
