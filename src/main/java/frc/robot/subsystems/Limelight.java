package frc.robot.subsystems;

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
    private final double cameraPitchOffset = 20; // degrees, following right hand rule
    private final double cameraYawOffset = -90; // degrees, following right hand rule
    private double cameraX; // computed, + is forward
    private double cameraY; // computed, + is left
    private double cameraYaw; // computed, + is ccw

    private double cameraAxisDiameter = 0.225; // meters, how wide the circle the camera's path of motion is
    /**
     * should be the angle that faces the CAMERA, NOT the angle that faces the forward side of the turret (typical angle measurement)
     * ALSO, the 0 of this value should coincide with the 0 of the robot, + is ccw
     */
    private double turretAngle = 0; // rads, dynamic, use a turret object to fetch
    private String limeLightName = "limelight-rogue";
    public double calculatedErrorAngle = 0;
    private SwerveDrivePoseEstimator m_poseEstimator;
    private Turret turret;
    private Field2d m_field = new Field2d();

    private Pose2d latestPose;
    @SuppressWarnings("removal")
    public Limelight(SwerveDrivePoseEstimator poseEstimator, Turret turretModule) {
        m_poseEstimator = poseEstimator;
        turret = turretModule;

        cameraX = turretAxisForwardOffset;
        cameraY = turretAxisLeftwardOffset;
        cameraYaw = cameraYawOffset;


        LimelightHelpers.setCameraPose_RobotSpace(limeLightName,
                turretAxisForwardOffset,
                turretAxisLeftwardOffset,
                cameraHeightOffset,
                0,
                cameraPitchOffset,
                cameraYawOffset);

        SmartDashboard.putData("Field", m_field);
    }

    private void calculateCameraTurretPosition() {
        turretAngle = turret.getPosition();
        double turretRelativeCameraX =  Math.sin(turretAngle) * cameraAxisDiameter/2; // + is forward
        double turretRelativeCameraY = -Math.cos(turretAngle) * cameraAxisDiameter/2; // + is leftward
        double turretRelativeCameraYaw = turretAngle; // + is ccw, name is redundant

        cameraX = turretRelativeCameraX + turretAxisForwardOffset;
        cameraY = turretRelativeCameraY + turretAxisLeftwardOffset;
        cameraYaw = turretRelativeCameraYaw;
    }

    private double turretAngleToTarget(Pose2d target, Pose2d currentPose){

        currentPose = currentPose
                // turret axis-bot center offset
                .plus(new Transform2d(turretAxisForwardOffset, turretAxisLeftwardOffset, new Rotation2d()));


        Pose2d targetTanComponent = new Pose2d(
                target.getX() - currentPose.getX(),
                target.getY() - currentPose.getY(),
                new Rotation2d()
        );

        Rotation2d targetHeading = new Rotation2d(
                Math.atan2(targetTanComponent.getY(), targetTanComponent.getX())
        );

        return targetHeading.minus(currentPose.getRotation()).getDegrees();
    }


    @Override
    public void periodic() {
        try {
            calculateCameraTurretPosition();
            LimelightHelpers.setCameraPose_RobotSpace(limeLightName,
                    cameraX,
                    cameraY,
                    cameraHeightOffset,
                    0,
                    cameraPitchOffset,
                    cameraYaw);


            LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limeLightName);
            if (measurement.tagCount >= 2) {
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.5, 0.5, 9999999));
                m_poseEstimator.addVisionMeasurement(
                    measurement.pose,
                    measurement.timestampSeconds);
            }

            latestPose = m_poseEstimator.getEstimatedPosition();

            calculatedErrorAngle = turretAngleToTarget(new Pose2d(5,10, new Rotation2d()), latestPose);
            SmartDashboard.putNumber("Calculated Angle: ", calculatedErrorAngle);
            m_field.setRobotPose(latestPose);
        } catch (Exception exception){
            SmartDashboard.putString("Limelight Exception", exception.getMessage());
        }
    }

}
