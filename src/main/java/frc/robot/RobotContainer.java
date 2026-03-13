// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Outake.Feeder;
import frc.robot.subsystems.Outake.Turret;
 
import frc.robot.subsystems.Outake.Shooter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/* 
import frc.robot.subsystems.Climber;
*/
import frc.robot.subsystems.Spindexer;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    public DriveSubsystem m_robotDrive;
    private IntakeRollers m_intakeRollers;
    private IntakePivot m_intakePivot;
    private Spindexer m_spindexer;
    private Shooter m_shooter;
    private Turret m_turret;
    private Feeder m_feeder;
    private Limelight m_limelight;
    private SendableChooser<Command> autoChooser;
    //private Climber m_climber = new Climber();

    // The driver's controller
    XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
    // The Mechanism Controller
    XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_robotDrive = new DriveSubsystem();
        m_intakeRollers = new IntakeRollers();
        m_intakePivot = new IntakePivot();
        m_spindexer = new Spindexer();
        m_feeder = new Feeder();
        m_turret = new Turret(m_robotDrive.getGyroObject());
        m_shooter = new Shooter();
        

        NamedCommands.registerCommand("turretToPosition", new RunCommand(() -> m_turret.turnToPosition(m_limelight.calculatedTargetAngleDegrees), m_turret));
        NamedCommands.registerCommand("shooterToVelocity", new RunCommand(() -> m_shooter.autoAdjustSpeed(m_limelight.distanceToTarget())));
        NamedCommands.registerCommand("bootIntake", new InstantCommand(() -> {
            m_intakePivot.lower();
            m_intakeRollers.intakeIn();
        }, m_intakePivot, m_intakeRollers));
        NamedCommands.registerCommand("feed", new RunCommand(() -> {
                        if (Math.abs(m_feeder.getRPM()) >= 1000) {
                                m_spindexer.spinClockwise(m_operatorController.getLeftTriggerAxis() );
                        }
                        if (!m_shooter.isturnedOff && m_shooter.getVelocity() > 5) {
                            m_feeder.feedIn();
                        }

                }, m_spindexer, m_feeder));

        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -0.8*MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                -0.8*MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                -0.8*MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                false),
                        m_robotDrive));


        m_turret.setDefaultCommand(
                new RunCommand(() -> {
                    if (m_turret.isTrackingPosition) {
                        m_turret.turnToPosition(m_limelight.calculatedTargetAngleDegrees);
                    } else {
                        double x = MathUtil.applyDeadband(-m_operatorController.getLeftX(), 0.1);
                        double y = MathUtil.applyDeadband(-m_operatorController.getLeftY(), 0.1);

                        if (Math.abs(x) < 0.1 && Math.abs(y) < 0.1) {
                                m_turret.turnToPosition();
                        } else {
                                double angle = Math.toDegrees(Math.atan2(x,y));
                                m_turret.turnToPosition(angle);
                        }
                    }
                }, m_turret)
        );

         
        m_shooter.setDefaultCommand(
                new RunCommand(() -> {
                    double y = 0.5*MathUtil.applyDeadband(-m_operatorController.getRightY(), 0.1);
                    if (!m_shooter.isturnedOff) {
                        if (!m_shooter.isAutoAdjusting) {
                            double setVelocity = m_shooter.getTargetVelocity() + y;

                            if (setVelocity > 0) {
                                m_shooter.spinWithVelocity(setVelocity);
                            } else {
                                m_shooter.spinWithVelocity(0);
                            }
                        } else {
                            m_shooter.autoAdjustSpeed(m_limelight.distanceToTarget());
                        }
                    } else {
                        m_shooter.spinWithVelocity(0);
                    }
                        
                }, m_shooter)
        );
    }

    public void setUpAutoChooser(){
        autoChooser = AutoBuilder.buildAutoChooser();
    }

    public void configureByColor(String team){
        if (team == "red"){
            // idk get the red goal pose
            m_limelight.setTargetPose(new Pose2d(4.625594,4.034536, new Rotation2d()));
        } else  if (team == "blue"){
            m_limelight.setTargetPose(new Pose2d(4.625594,4.034536, new Rotation2d()));
        }
    }

    public void initLimelight(){
        m_limelight = new Limelight(m_robotDrive.m_odometry, m_turret, m_robotDrive.m_gyro, false);
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
        leftBumper.onTrue(
                new InstantCommand(() -> m_intakePivot.raise(), m_intakePivot)
        );
        Trigger rightBumper = new Trigger(() -> m_driverController.getRightBumperButton());
        rightBumper.onTrue(
                new InstantCommand(() -> m_intakePivot.lower(), m_intakePivot)
        );

        // OPERATOR CONTROLLER ---

        // Spindexer
        Trigger leftTrigger2 = new Trigger(() -> m_operatorController.getLeftTriggerAxis() > 0.03);
        leftTrigger2.whileTrue(
                new RunCommand(() -> {
                        if (Math.abs(m_feeder.getRPM()) >= 1000) {
                                m_spindexer.spinClockwise(m_operatorController.getLeftTriggerAxis() );
                        }
                        if (!m_shooter.isturnedOff && m_shooter.getVelocity() > 5) {
                            m_feeder.feedIn();
                        }

                }, m_spindexer, m_feeder)
        );

        // Stop Button
        Trigger buttonB = new Trigger(() -> m_operatorController.getBButton());
        buttonB.onTrue(
                new RunCommand(() -> {
                        m_spindexer.stop();
                        m_shooter.stop();
                        m_feeder.stop();
                        m_intakePivot.stop();
                        m_intakeRollers.stop();
                        m_turret.stop();
                }, m_spindexer, m_shooter, m_feeder, m_intakePivot, m_intakeRollers, m_turret)
        );

        // Turret tracking toggle
        Trigger buttonX = new Trigger(() -> m_operatorController.getXButtonPressed());
        buttonX.onTrue(
                new InstantCommand(() -> m_turret.isTrackingPosition = !m_turret.isTrackingPosition, m_turret)
        );

        // Shooter power toggle
        Trigger buttonY = new Trigger(() -> m_operatorController.getYButtonPressed());
        buttonY.onTrue(
                new InstantCommand(() -> {
                    m_shooter.isturnedOff = !m_shooter.isturnedOff;
                    if (!m_shooter.isturnedOff) {
                        // set default spinning speed to 24 m/s
                        m_shooter.spinWithVelocity(24);
                    }
                }, m_shooter)
        );

        // Shooter-Speed Control Toggle
        Trigger buttonA = new Trigger(() -> m_operatorController.getAButtonPressed());
        buttonA.onTrue(
                new InstantCommand(() -> m_shooter.isturnedOff = !m_shooter.isturnedOff, m_shooter)
        );

        // Anti-Jam reverse trigger
        Trigger rightTrigger2 = new Trigger(()  -> m_operatorController.getRightTriggerAxis() > 0.2);
        rightTrigger2.whileTrue(
                new RunCommand(() -> {
                    if (Math.abs(m_feeder.getRPM()) >= 1000) {
                        m_spindexer.spinCounterClockwise(m_operatorController.getRightTriggerAxis() );
                    }
                }, m_spindexer, m_feeder)
        );
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return autoChooser.getSelected();
        /* 
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
        */
    }

    public IntakeRollers getIntake() {
        return m_intakeRollers;
    }

    public Spindexer getSpindexer() {
        return m_spindexer;
    }

}
