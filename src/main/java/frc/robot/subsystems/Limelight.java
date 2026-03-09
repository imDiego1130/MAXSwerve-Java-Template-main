package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
    // Offsets at initialization

    private final double turretAxisForwardOffset = -0.5; // meters, forward from robot center
    private final double turretAxisLeftwardOffset = 0.5; // meters, left from robot center
    private final double cameraHeightOffset = 0.1; // meters, up from robot center
    private final double cameraPitchOffset = 10; // degrees, following right hand rule
    private final double cameraYawOffset = -90; // degrees, following right hand rule
    private double cameraX; // computed, + is forward
    private double cameraY; // computed, + is left
    private double cameraYaw; // computed, + is ccw

    private double cameraAxisDiameter = 0.05; // meters, how wide the circle the camera's path of motion is
    /**
     * should be the angle that faces the CAMERA, NOT the angle that faces the forward side of the turret (typical angle measurement)
     * ALSO, the 0 of this value should coincide with the 0 of the robot, + is ccw
     */
    private double turretAngle = 0; // rads, dynamic, use a turret object to fetch
    @SuppressWarnings("removal")
    public Limelight() {
        cameraX = turretAxisForwardOffset;
        cameraY = turretAxisLeftwardOffset;
        cameraYaw = cameraYawOffset;


        LimelightHelpers.setCameraPose_RobotSpace("frc-8573",
                turretAxisForwardOffset,
                turretAxisLeftwardOffset,
                cameraHeightOffset,
                0,
                cameraPitchOffset,
                cameraYawOffset);
    }

    private void calculateCameraTurretPosition() {
        double turretRelativeCameraX = Math.sin(turretAngle) * cameraYawOffset/2; // + is forward
        double turretRelativeCameraY = -Math.cos(turretAngle) * cameraPitchOffset/2; // + is leftward
        double turretRelativeCameraYaw = Math.toDegrees(turretAngle); // + is ccw, name is redundant

        cameraX = turretRelativeCameraX + turretAxisForwardOffset;
        cameraY = turretRelativeCameraY + turretAxisLeftwardOffset;
        cameraYaw = turretRelativeCameraYaw;
    }


    @Override
    public void periodic() {
        calculateCameraTurretPosition();
        LimelightHelpers.setCameraPose_RobotSpace("frc-8573",
                cameraX,
                cameraY,
                cameraHeightOffset,
                0,
                cameraPitchOffset,
                cameraYaw);
        // set limelight scan action here
        Pose2d pose = new Pose2d();
        SmartDashboard.putNumber("Limelight Pose: ", pose.getX());
    }

}
