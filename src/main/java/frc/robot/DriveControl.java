package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
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
    private double middleY = 4.021328;
    private double blueGoalX = 4.611624;
    private double goalToGoal = 7.2898;
    private Pose2d redGoal = (new Pose2d((blueGoalX+goalToGoal),(middleY), new Rotation2d()));
    private Pose2d redTopZone = (new Pose2d((blueGoalX*1.5+goalToGoal), (middleY*1.5), new Rotation2d()));
    private Pose2d redBottomZone = (new Pose2d((blueGoalX*1.5+goalToGoal), (middleY*0.5), new Rotation2d()));

    private Pose2d blueGoal = (new Pose2d((blueGoalX),(middleY), new Rotation2d()));
    private Pose2d blueTopZone = (new Pose2d((blueGoalX*0.5), (middleY*1.5), new Rotation2d()));
    private Pose2d blueBottomZone = (new Pose2d((blueGoalX*0.5), (middleY*0.5), new Rotation2d()));
    private boolean isTeamBlue = true;
    public boolean isTargetingGoal = true;

        
    private final ArrayList<double[]> dataPoints = new ArrayList<>(Arrays.asList(
            // 0 : distance (meters), 1 : velocity (m/s), 2 : angle (deg)
            new double[]{3.05 , 19, 0.0 }
            ,new double[]{4.36, 22, 0.0 }
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

    public void setTeamColor(String team){
        if (team == "red") {
            isTeamBlue = false;
        } else if (team == "blue") {
            isTeamBlue = true;
        }
    }

    public Pose2d getCurrentObjectivePose(Pose2d currentPose){
        double poseY = currentPose.getY();
        boolean isAboveMidfield = (poseY >= middleY) ? true : false;
        if (isTeamBlue) {
            if (isTargetingGoal) {
                return blueGoal;
            } else if (isAboveMidfield) {
                return blueTopZone;
            } else {
                return blueBottomZone;
            }
        } else {
            if (isTargetingGoal) {
                return redGoal;
            } else if (isAboveMidfield) {
                return redTopZone;
            } else {
                return redBottomZone;
            }
        }
    }

    public Rotation2d getJoyStickAngle(){
        double x = -MathUtil.applyDeadband(operator.getRightX(), 0.1);
        double y = -MathUtil.applyDeadband(operator.getRightY(), 0.1);

        if (Math.hypot(x, y) < 0.15){
            return null;
        }

        return Rotation2d.fromRadians((Math.atan2(x, y)));
    }

    public double getForward(){
        return -1*MathUtil.applyDeadband(driver.getLeftY(), 0.15);
    }

    public double getStrafe(){
        return -1*MathUtil.applyDeadband(driver.getLeftX(), 0.15);
    }

    public double getRotate(){
        return -1*MathUtil.applyDeadband(driver.getRightX(), 0.15);
    }
    public double getOperatorRightY(){
        return -0.5*MathUtil.applyDeadband(operator.getRightY(), 0.15);
    }

    public double getCorrelatedVelocity(){
        return getRegressionValue(getVisionDistance(), 1);
    }
}
