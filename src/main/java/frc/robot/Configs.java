package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

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
      double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

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
          .pid(0.04, 0.0, 0.0)
          .outputRange(-1.0, 1.0);

      drivingConfig.closedLoop.feedForward
          .kV(drivingVelocityFeedForward);

      // -------------------- TURNING --------------------
      turningConfig
          .idleMode(IdleMode.kBrake)
          .smartCurrentLimit(20);

      // Use the motor’s integrated encoder (primary encoder) for turning closed-loop
      turningConfig.encoder
          .positionConversionFactor(turningFactor)         // radians (per motor rotation)
          .velocityConversionFactor(turningFactor / 60.0); // radians per second

      turningConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
          // Example gains — tune for your robot
          .pid(2.60, 0.0, 0.0)
          .outputRange(-1.0, 1.0)
          // Wrap 0..2pi so angle setpoints behave nicely
          .positionWrappingEnabled(true)
          .positionWrappingInputRange(0.0, turningFactor);
    }
  }
  public static final class Intake {

    public static final SparkMaxConfig rollerConfig = new SparkMaxConfig();
    public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

    static {
        // Rollers (open loop)
        rollerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        // Pivot (closed loop position)
        pivotConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(30);

        pivotConfig.encoder
            .positionConversionFactor(1.0);

        pivotConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.05, 0.0, 0.0)
            .outputRange(-1.0, 1.0);
    }
}

}
