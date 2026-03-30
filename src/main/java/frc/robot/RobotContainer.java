// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveAttachment.DriveSubsystem;
import frc.robot.subsystems.DriveAttachment.ManualDriveCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightAutoTargetCommand;
import frc.robot.subsystems.Outake.Feeder;
import frc.robot.subsystems.Outake.TurretAttachment.AutoTurretCommand;
import frc.robot.subsystems.Outake.TurretAttachment.ManualTurretCommand;
import frc.robot.subsystems.Outake.TurretAttachment.Turret;
import frc.robot.subsystems.ShooterAttachment.AutoSetShooterCommand;
import frc.robot.subsystems.ShooterAttachment.ManualShooterCommand;
import frc.robot.subsystems.ShooterAttachment.Shooter;
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

    private DriveControl controlCenter = new DriveControl(m_driverController, m_operatorController);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_intakeRollers = new IntakeRollers();
        m_intakePivot = new IntakePivot();
        m_spindexer = new Spindexer();
        m_feeder = new Feeder();
        m_turret = new Turret();
        m_shooter = new Shooter();
        m_robotDrive = new DriveSubsystem();

        controlCenter.setVisionAngleSupplier(() -> m_limelight.calculatedTargetAngleDegrees);
        controlCenter.setVisionDistanceSupplier(() -> m_limelight.distanceToTarget());

        // command that never finishes, always tracks LL's target position
        NamedCommands.registerCommand("turretToPosition", new AutoTurretCommand(m_turret, controlCenter));
        NamedCommands.registerCommand("setShooterSpeed", new AutoSetShooterCommand(m_shooter, controlCenter));
        NamedCommands.registerCommand("feedIn", new RunCommand(() -> m_feeder.feedIn(), m_feeder));
        NamedCommands.registerCommand("spindexerIn", new RunCommand(() -> m_spindexer.spinClockwise(1), m_spindexer));
        NamedCommands.registerCommand("intakeDown", new InstantCommand(() -> m_intakePivot.lower(), m_intakePivot));


        // Configure the button bindings
        configureButtonBindings();

        // Configure default commands
        m_robotDrive.setDefaultCommand(
                new ManualDriveCommand(m_robotDrive, controlCenter)
        );


        m_turret.setDefaultCommand(
                new ManualTurretCommand(m_turret, controlCenter)
        );

         
        m_shooter.setDefaultCommand(
                new ManualShooterCommand(m_shooter, controlCenter)
        );

        SmartDashboard.putBoolean("isTargetingGoal:", controlCenter.isTargetingGoal);
    }

     public void setUpAutoChooser(){
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public void configureByColor(String team){
        controlCenter.setTeamColor(team);
    }

    public void initLimelight(){
        m_limelight = new Limelight(m_robotDrive.m_odometry, m_turret, m_robotDrive.m_gyro, true);

        m_limelight.setDefaultCommand(
                new LimelightAutoTargetCommand(m_limelight, controlCenter)
        );
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
        Trigger rightTrigger2 = new Trigger(() -> m_operatorController.getRightTriggerAxis() > 0.1);
        rightTrigger2.whileTrue(
                new RunCommand(() -> {
                        if (Math.abs(m_feeder.getRPM()) >= 1000) {
                                m_spindexer.spinClockwise(m_operatorController.getRightTriggerAxis() );
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
                        m_turret.holdCurrentPosition();
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
                new InstantCommand(() -> m_shooter.isturnedOff = !m_shooter.isturnedOff, m_shooter)
        );

        Trigger leftBumper2 = new Trigger(() -> m_operatorController.getLeftBumperButtonPressed());
        leftBumper2.onTrue(
            new RunCommand(
                () -> {
                    if (!m_shooter.isturnedOff) {
                        m_shooter.targetVelocity = 16.5;//19
                    }
                }, m_shooter)
        );

        Trigger rightBumper2 = new Trigger(() -> m_operatorController.getRightBumperButtonPressed());
        rightBumper2.onTrue(
            new RunCommand(
                () -> {
                    if (!m_shooter.isturnedOff) {
                        m_shooter.targetVelocity = 18.75;//
                    }
                }, m_shooter)
        );

        // Anti-Jam reverse trigger
        Trigger leftTrigger2 = new Trigger(()  -> m_operatorController.getLeftTriggerAxis() > 0.2);
        leftTrigger2.whileTrue(
                new RunCommand(() -> {
                    if (Math.abs(m_feeder.getRPM()) >= 1000) {
                        m_spindexer.spinCounterClockwise(m_operatorController.getLeftTriggerAxis() );
                    }
                    m_feeder.feedOut();
                }, m_spindexer, m_feeder)
        );

        Trigger buttonA = new Trigger(() -> m_operatorController.getAButtonPressed());
        buttonA.onTrue(
            new InstantCommand(() -> controlCenter.isTargetingGoal = !controlCenter.isTargetingGoal)
        );
        
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public String getAutonomousCommand() {

        return autoChooser.getSelected().getName();
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
}
