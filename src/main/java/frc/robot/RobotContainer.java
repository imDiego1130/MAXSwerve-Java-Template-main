// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
/* 
import frc.robot.subsystems.Outake.Feeder;
import frc.robot.subsystems.Outake.Shooter;
import frc.robot.subsystems.Outake.Turret;
*/
import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;

import frc.robot.subsystems.Intake.IntakePivot;
import frc.robot.subsystems.Intake.IntakeRollers;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final IntakeRollers m_intakeRollers = new IntakeRollers();
    private final IntakePivot m_intakePivot = new IntakePivot();
    private final Spindexer m_spindexer = new Spindexer();
    //private final Shooter m_shooter = new Shooter();
    //private final Turret m_turret = new Turret();
    //private final Feeder m_feeder = new Feeder();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    // The Mechanism Controller
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                true),
                        m_robotDrive));
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
     * subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
     * passing it to a
     * {@link JoystickButton}.
     */
    private void configureButtonBindings() {
        
        new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(new RunCommand(
                        () -> m_robotDrive.setX(),
                        m_robotDrive));

        new JoystickButton(m_driverController, XboxController.Button.kStart.value)
                .onTrue(new InstantCommand(
                        () -> m_robotDrive.zeroHeading(),
                        m_robotDrive));

        // Triggers
        Trigger rightTrigger = new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.2);
        rightTrigger.whileTrue(
                new RunCommand(() -> m_intakeRollers.intakeIn(), m_intakeRollers)
        );

        Trigger leftTrigger = new Trigger(() -> m_driverController.getLeftTriggerAxis() > 0.2);
        leftTrigger.whileTrue(
                new RunCommand(() -> m_intakeRollers.intakeOut(), m_intakeRollers)
        );

        // Bumpers
        Trigger leftBumper = new Trigger(() -> m_driverController.getLeftBumperButton());
        leftBumper.toggleOnTrue(
                new RunCommand(() -> m_intakePivot.lower(), m_intakePivot)
        );
        Trigger rightBumper = new Trigger(() -> m_driverController.getRightBumperButton());
        rightBumper.toggleOnTrue(
                new RunCommand(() -> m_intakePivot.raise(), m_intakePivot)
        );

        // Spindexer
        Trigger buttonA = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.03);
        buttonA.whileTrue(
                new RunCommand(() -> m_spindexer.spinClockwise(m_operatorController.getLeftTriggerAxis() ), m_spindexer)
        );

        Trigger buttonY = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.03);
        buttonY.whileTrue(
                new RunCommand(() -> m_spindexer.spinCounterClockwise(m_operatorController.getRightTriggerAxis()), m_spindexer)
        );
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // Create config for trajectory
        TrajectoryConfig config = new TrajectoryConfig(
                AutoConstants.kMaxSpeedMetersPerSecond,
                AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(DriveConstants.kDriveKinematics);

        // An example trajectory to follow. All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                config);

        var thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                exampleTrajectory,
                m_robotDrive::getPose, // Functional interface to feed supplier
                DriveConstants.kDriveKinematics,

                // Position controllers
                new PIDController(AutoConstants.kPXController, 0, 0),
                new PIDController(AutoConstants.kPYController, 0, 0),
                thetaController,
                m_robotDrive::setModuleStates,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    }

    public IntakeRollers getIntake() {
        return m_intakeRollers;
    }

    public Spindexer getSpindexer() {
        return m_spindexer;
    }

}
