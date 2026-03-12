package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.Outake.Turret;

public class Limelight extends SubsystemBase {
    // Offsets at initialization

    private final double turretAxisForwardOffset = -0.16; // meters, forward from robot center
    private final double turretAxisLeftwardOffset = 0.15; // meters, left from robot center
    private final double cameraHeightOffset = 0.59; // meters, up from robot center
    private final double cameraPitchOffset = 10; // degrees, following right hand rule
    private final double cameraYawOffset = -90; // degrees, following right hand rule
    private double cameraX; // computed, + is forward
    private double cameraY; // computed, + is left
    private double cameraYaw; // computed, + is ccw

    private final double cameraAxisDiameter = 0.225; // meters, how wide the circle the camera's path of motion is
    /**
     * should be the angle that faces the CAMERA, NOT the angle that faces the forward side of the turret (typical angle measurement)
     * ALSO, the 0 of this value should coincide with the 0 of the robot, + is ccw
     */
    private double turretAngle = 0; // degs, dynamic, use a turret object to fetch
    private String limeLightName = "limelight-rogue";
    public double calculatedTargetAngleDegrees = 0;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Turret turret;
    private AHRS m_gyro;
    private Field2d m_field = new Field2d();
    // automatically set to Blue Goal
    private Pose2d targetPose = new Pose2d(4.625594,4.034536, new Rotation2d());
    private final boolean setToScanWithMT2;

    private Pose2d latestPose = new Pose2d();
    
    public Limelight(SwerveDrivePoseEstimator poseEstimator, Turret turretModule, AHRS gyroModule, boolean scanWithMT2) {
        m_poseEstimator = poseEstimator;
        turret = turretModule;
        m_gyro = gyroModule;
        setToScanWithMT2 = scanWithMT2;

        cameraX = turretAxisForwardOffset;
        cameraY = turretAxisLeftwardOffset;
        cameraYaw = cameraYawOffset;

        LimelightHelpers.setPipelineIndex(limeLightName, 1);

        SmartDashboard.putData("Field", m_field);
    }

    private void calculateCameraTurretPosition() {
        turretAngle = turret.getPosition();
        double turretAngleRads = Math.toRadians(turretAngle);
        double turretRelativeCameraX =  Math.cos(turretAngleRads) * (cameraAxisDiameter/2); // + is forward
        double turretRelativeCameraY = -Math.sin(turretAngleRads) * (cameraAxisDiameter/2); // + is leftward
        double turretRelativeCameraYaw = turretAngle; // + is ccw, name is redundant

        cameraX = turretRelativeCameraX + turretAxisForwardOffset;
        cameraY = turretRelativeCameraY + turretAxisLeftwardOffset;
        cameraYaw = turretRelativeCameraYaw;
    }

    private double turretAngleToTargetDegrees(Pose2d target, Pose2d currentPose){

        currentPose = currentPose
                // turret axis-bot center offset
                .plus(new Transform2d(turretAxisForwardOffset, turretAxisLeftwardOffset, new Rotation2d()));


        Pose2d targetDistanceComponents = new Pose2d(
                target.getX() - currentPose.getX(),
                target.getY() - currentPose.getY(),
                new Rotation2d()
        );

        Rotation2d targetHeading = new Rotation2d(
                Math.atan2(targetDistanceComponents.getY(), targetDistanceComponents.getX())
        );

        return targetHeading.minus(currentPose.getRotation()).getDegrees();
    }

    public double distanceToTarget(){
        double x = targetPose.getX() - latestPose.getX();
        double y = targetPose.getY() - latestPose.getY();
        return (Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2)));
    }

    private void scanWithMT1(){
        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
        SmartDashboard.putNumber("Targets Detected", mt1.tagCount);

        boolean doRejectUpdate = false;
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > .7) {
                doRejectUpdate = true;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                doRejectUpdate = true;
            }
        }

        if (mt1.tagCount == 0) {
            doRejectUpdate = true;
        }

        if (!doRejectUpdate && mt1.tagCount >= 2) {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, Math.toRadians(8)));
            m_poseEstimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
        }
    }

    private void scanWithMT2(){
        LimelightHelpers.SetRobotOrientation(limeLightName, m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limeLightName);
        SmartDashboard.putNumber("Targets Detected", mt2.tagCount);

        boolean doRejectUpdate = false;
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        if(Math.abs(m_gyro.getRate()) > 360)
        {
            doRejectUpdate = true;
        }
        if(mt2.tagCount == 0)
        {
            doRejectUpdate = true;
        }
        if(!doRejectUpdate)
        {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
            m_poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
        }
    }

    public void setTargetPose(Pose2d pose){
        targetPose = pose;
    }


    @Override
    public void periodic() {
        try {
            if (m_poseEstimator == null || turret == null) {
                SmartDashboard.putString("Limelight Status", "Waiting for Subsystems");
                return;
            }

            calculateCameraTurretPosition();
            LimelightHelpers.setCameraPose_RobotSpace(limeLightName,
                    cameraX,
                    cameraY,
                    cameraHeightOffset,
                    0,
                    cameraPitchOffset,
                    cameraYaw);


            if (setToScanWithMT2) {
                scanWithMT2();
            } else {
                scanWithMT1();
            }

            latestPose = m_poseEstimator.getEstimatedPosition();
            if (latestPose == null) {
                SmartDashboard.putString("Limelight Status", "Estimator No Pose");
                return;
            }

            calculatedTargetAngleDegrees = turretAngleToTargetDegrees(targetPose, latestPose);
            SmartDashboard.putNumber("Calculated Angle: ", calculatedTargetAngleDegrees);
            m_field.setRobotPose(latestPose);
            SmartDashboard.putString("Limelight Status", "OK");
            SmartDashboard.putNumber("Distance To Target ", distanceToTarget());
        } catch (Exception exception){
            SmartDashboard.putString("Limelight Exception", exception.getMessage());
        }
    }

}
