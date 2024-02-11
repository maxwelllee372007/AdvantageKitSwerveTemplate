package frc.robot.subsystems.drive;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static DrivetrainConfig drivetrainConfig =
      switch (Constants.getRobot()) {
        default ->
            new DrivetrainConfig(
                TunerConstants.kWheelRadiusInches,
                // Units.inchesToMeters(2.0),
                TunerConstants.kFrontLeftXPosInches * 2,
                TunerConstants.kFrontLeftXPosInches * 2,
                // Units.inchesToMeters(26.0),
                // Units.inchesToMeters(26.0),
                Units.feetToMeters(12.16),
                Units.feetToMeters(21.32),
                7.93,
                29.89);
      };
  public static final double wheelRadius = Units.inchesToMeters(2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);
  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case COMPBOT -> 250.0;
      };
  public static final Matrix<N3, N1> stateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };
  public static final double xyStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };
  public static final double thetaStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };

  public static final int gyroID = TunerConstants.kPigeonId;

  // Turn to "" for no canbus name
  public static final String canbus = TunerConstants.kCANbusName;

  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConfig[] {
              new ModuleConfig(
                  TunerConstants.kFrontLeftDriveMotorId,
                  TunerConstants.kFrontLeftSteerMotorId,
                  TunerConstants.kFrontLeftEncoderId,
                  Rotation2d.fromRotations(TunerConstants.kFrontLeftEncoderOffset)
                      .plus(Rotation2d.fromDegrees(180)),
                  TunerConstants.kSteerMotorReversed,
                  TunerConstants.kInvertLeftSide),
              new ModuleConfig(
                  TunerConstants.kFrontRightDriveMotorId,
                  TunerConstants.kFrontRightSteerMotorId,
                  TunerConstants.kFrontRightEncoderId,
                  Rotation2d.fromRotations(TunerConstants.kFrontRightEncoderOffset)
                      .plus(Rotation2d.fromDegrees(180)),
                  TunerConstants.kSteerMotorReversed,
                  TunerConstants.kInvertRightSide),
              new ModuleConfig(
                  TunerConstants.kBackLeftDriveMotorId,
                  TunerConstants.kBackLeftSteerMotorId,
                  TunerConstants.kBackLeftEncoderId,
                  Rotation2d.fromRotations(TunerConstants.kBackLeftEncoderOffset)
                      .plus(Rotation2d.fromDegrees(180)),
                  TunerConstants.kSteerMotorReversed,
                  TunerConstants.kInvertLeftSide),
              new ModuleConfig(
                  TunerConstants.kBackRightDriveMotorId,
                  TunerConstants.kBackRightSteerMotorId,
                  TunerConstants.kBackRightEncoderId,
                  Rotation2d.fromRotations(TunerConstants.kBackRightEncoderOffset)
                      .plus(Rotation2d.fromDegrees(180)),
                  TunerConstants.kSteerMotorReversed,
                  TunerConstants.kInvertRightSide)
            };
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(), false, false);
          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConstants(
                TunerConstants.driveGains.kS,
                TunerConstants.driveGains.kV,
                TunerConstants.driveGains.kP,
                TunerConstants.driveGains.kD,
                TunerConstants.steerGains.kP,
                TunerConstants.steerGains.kD,
                Mk4iReductions.L3.reduction,
                Mk4iReductions.TURN.reduction);
        case SIMBOT ->
            new ModuleConstants(
                TunerConstants.driveGains.kS,
                TunerConstants.driveGains.kV,
                TunerConstants.driveGains.kP,
                TunerConstants.driveGains.kD,
                TunerConstants.steerGains.kP,
                TunerConstants.steerGains.kD,
                Mk4iReductions.L3.reduction,
                Mk4iReductions.TURN.reduction);
      };

  public static HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new HeadingControllerConstants(3.0, 0.0);
        case SIMBOT -> new HeadingControllerConstants(3.0, 0.0);
      };

  public static final PIDConstants PPtranslationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(10, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
      };

  public static final PIDConstants PProtationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(10, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
      };

  public record DrivetrainConfig(
      double wheelRadius,
      double trackwidthX,
      double trackwidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted,
      boolean driveMotorInverted) {}

  public record ModuleConstants(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  public record HeadingControllerConstants(double Kp, double Kd) {}

  private enum Mk4iReductions {
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0));

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
