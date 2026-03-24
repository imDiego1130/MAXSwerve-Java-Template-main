package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;

public class DriveControl {
    private XboxController driver;
    private XboxController operator;
    private DoubleSupplier visionAngleSupplier;
    private DoubleSupplier visionDistanceSupplier;
    private final ArrayList<double[]> dataPoints = new ArrayList<>(Arrays.asList(
            // 0 : distance (meters), 1 : velocity (m/s), 2 : angle (deg)
            new double[]{3.2 , 8.0, 0.0 }
            ,new double[]{4.36, 9.0, 0.0 }
            //,new double[]{5.5 , 25.7, 0.0 }
    ));

    private final double[] velocitySlopeList = initializeSlopeList(1);
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

    public  DriveControl(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    public void setVisionAngleSupplier(DoubleSupplier visionAngleSupplier) {
        this.visionAngleSupplier = visionAngleSupplier;
    }
    public void setVisionDistanceSupplier(DoubleSupplier visionDistanceSupplier) {
        this.visionDistanceSupplier = visionDistanceSupplier;
    }

    public double getVisionAngle() {
        return  visionAngleSupplier.getAsDouble();
    }
    public double getVisionDistance() {
        return  visionDistanceSupplier.getAsDouble();
    }

    public Rotation2d getJoyStickAngle(){
        double x = MathUtil.applyDeadband(operator.getRightX(), 0.1);
        double y = -MathUtil.applyDeadband(operator.getRightY(), 0.1);

        if (Math.hypot(x, y) < 0.15){
            return null;
        }

        return Rotation2d.fromRadians((Math.atan2(x, y)));
    }

    public double getForward(){
        return -0.9*MathUtil.applyDeadband(driver.getLeftY(), 0.15);
    }

    public double getStrafe(){
        return -0.9*MathUtil.applyDeadband(driver.getLeftX(), 0.15);
    }

    public double getRotate(){
        return -0.7*MathUtil.applyDeadband(driver.getRightX(), 0.15);
    }
    public double getOperatorRightY(){
        return -0.5*MathUtil.applyDeadband(operator.getRightY(), 0.15);
    }

    public double getCorrelatedVelocity(){
        return getRegressionValue(getVisionDistance(), 1);
    }
}
