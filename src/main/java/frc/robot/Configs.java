package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.ClimberConstants;

public final class Configs {
  public static final class MAXSwerveModule {
    public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

    static {
      // Use module constants to calculate conversion factors and feed forward gain.
      double drivingFactor =
          ModuleConstants.kWheelDiameterMeters * Math.PI / ModuleConstants.kDrivingMotorReduction;

      // NOTE:
      // NEO integrated encoder reports motor rotations.
      // If you want "radians at the wheel/module", you should divide by the steering reduction here.
      // For now, this matches your existing intent (radians per motor rotation).
      double turningFactor = (2.0 * Math.PI) / 12.8;

      double nominalVoltage = 12.0;

      // Your constant name says Rps, but your math produces meters/sec.
      // This keeps your behavior unchanged.  
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeVelocityMps;

      // -------------------- DRIVING --------------------
      drivingConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(50);

      drivingConfig.encoder
          .positionConversionFactor(drivingFactor)         // meters
          .velocityConversionFactor(drivingFactor / 60.0); // meters per second

      drivingConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Example gains — tune for your robot
          .pid(0.04, 0.0, 0)
          .outputRange(-1.0, 1.0);

      drivingConfig.closedLoop.feedForward
          .kV(drivingVelocityFeedForward);

      // -------------------- TURNING --------------------
      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(30);

      // Use the motor’s integrated encoder (primary encoder) for turning closed-loop
      turningConfig.encoder
          .positionConversionFactor(turningFactor)         // radians (per motor rotation)
          .velocityConversionFactor(turningFactor / 60.0); // radians per second

      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          // Example gains — tune for your robot
          .pid(2.0, 0.0, 1.5)
          .outputRange(-1.0, 1.0)
          // Wrap 0..2pi so angle setpoints behave nicely
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0.0, 2.0 * Math.PI);
    }
  }
  public static final class Intake {

    public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    static {

      // NEO internal encoder reports rotations.
      // We want to control how many degrees the end axle moves
      // Since there is a gearbox on the motor, (10 in : 1 out), we multiply the encoder return by that ratio
      double pivotFactor =
              (1 / PivotConstants.kPivotMotorReduction) * 360;

      // Now, the PID and target position is in end axle rotations (the actual mechanism) instead of the raw motor rotations


        // Rollers (open loop)
        rollerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        // Pivot (closed loop position)
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        pivotConfig.encoder
            .positionConversionFactor(pivotFactor) // degrees
            .velocityConversionFactor(pivotFactor / 60.0); // degrees / second

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.1, 0.0, 0.0)
            .outputRange(-0.6, 0.6);
    }
  }
  public static final class Spindexer {
    public static final SparkMaxConfig spindexerConfig = new SparkMaxConfig();

    static {
        // spindexer (open loop : for now)
        spindexerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(50);

    }
  }

  public static final class Outake {
    public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
    public static final SparkMaxConfig feederConfig = new SparkMaxConfig();
    public static final SparkMaxConfig turretConfig = new SparkMaxConfig();

    static {
      double shooterFactor = ShooterConstants.kWheelCircumferenceMeters / ShooterConstants.kShootingMotorReduction;
      double turretFactor = (1 / TurretConstants.kTurretMotorReduction) * 360;

      double nominalVoltage = 12.0;

      double shootingVelocityFeedForward =  nominalVoltage / ShooterConstants.kWheelFreeVelocityMps;

      shooterConfig
              .idleMode(IdleMode.kCoast)
              .inverted(false) // shooter is geared, invert direction (+ direction shoots out)
              .smartCurrentLimit(50);

      turretConfig
              .idleMode(IdleMode.kBrake)
              .inverted(false)
              .smartCurrentLimit(30);

      
      feederConfig
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(30);

      shooterConfig.encoder
              .positionConversionFactor(shooterFactor)         // meters
              .velocityConversionFactor(shooterFactor / 60.0); // meters per second

      turretConfig.encoder
              .positionConversionFactor(turretFactor)         // degrees
              .velocityConversionFactor(turretFactor / 60.0); // degrees per second

      shooterConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(0.11, 0.0, 0)
              .outputRange(-1.0, 1.0);

      turretConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(0.005, 0.0, 0)
              .outputRange(-1.0, 1.0);

      shooterConfig.closedLoop.feedForward
              .kV(shootingVelocityFeedForward);

      turretConfig.softLimit
              .forwardSoftLimit(91)
              .reverseSoftLimit(-91);
    }
  }

  public static final class Climber {

    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {
      double climberFactor = (ClimberConstants.kClimberSpoolCircumference / ClimberConstants.kClimberMotorReduction);

      climberConfig
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(50);

      climberConfig.encoder
              .positionConversionFactor(climberFactor)         // meters
              .velocityConversionFactor(climberFactor / 60.0); // meters per second

      climberConfig.closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              .pid(1.0, 0.0, 0)
              .outputRange(-1.0, 1.0);
    }
  }

}
