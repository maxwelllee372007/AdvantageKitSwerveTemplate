// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int loopPeriodMs = 20;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final boolean tuningMode = true;
  public static final boolean characterizationMode = false;

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid Robot Selected, using COMPBOT as default", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType.toString());
      System.exit(1);
    }
  }

  public static class VisionConstants {

    public static boolean USE_VISION = false;

    public static final String FRONT_CAMERA_NAME = "BlueCamera"; // LEFT
    public static final String BACK_CAMERA_NAME = "BlackCamera"; // RIGHT
    public static final String NOTE_CAMERA_NAME = "NoteCamera";

    /** Physical location of the left camera on the robot, relative to the center of the robot. */
    public static final Transform3d ROBOT_TO_FRONT_CAMERA =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(15.5), Units.inchesToMeters(0), Units.inchesToMeters(6.5)),
            new Rotation3d(0, Math.toRadians(30), Math.toRadians(0)));

    /** Physical location of the back camera on the robot, relative to the center of the robot. */
    public static final Transform3d ROBOT_TO_BACK_CAMERA =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(15.5), Units.inchesToMeters(0), Units.inchesToMeters(6.5)),
            new Rotation3d(0, Math.toRadians(39), Math.toRadians(180)));

    /** Minimum target ambiguity. Targets with higher ambiguity will be discarded */
    public static final double APRILTAG_AMBIGUITY_THRESHOLD = 0.2;

    public static final double POSE_AMBIGUITY_SHIFTER = 0.2;
    public static final double POSE_AMBIGUITY_MULTIPLIER = 4;
    public static final double NOISY_DISTANCE_METERS = 2.5;
    public static final double DISTANCE_WEIGHT = 7;
    public static final int TAG_PRESENCE_WEIGHT = 10;

    // PLACEHOLDER
    public static final double NOTE_CAMERA_HEIGHT_METERS = Units.inchesToMeters(20);
    public static final double NOTE_HEIGHT_METERS = Units.inchesToMeters(2);
    public static final double NOTE_CAMERA_PITCH_RADIANS = Units.degreesToRadians(30);

    /**
     * Standard deviations of model states. Increase these numbers to trust your model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
     * then meters.
     */
    public static final Matrix<N3, N1> VISION_MEASUREMENT_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
                1, // x
                1, // y
                1 * Math.PI // theta
                );

    /**
     * Standard deviations of the vision measurements. Increase these numbers to trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians.
     */
    public static final Matrix<N3, N1> STATE_STANDARD_DEVIATIONS =
        Matrix.mat(Nat.N3(), Nat.N1())
            .fill(
                // if these numbers are less than one, multiplying will do bad things
                .1, // x
                .1, // y
                .1);

    public static final double FIELD_LENGTH_METERS = 16.54175;

    public static final double FIELD_WIDTH_METERS = 8.0137;

    // Pose on the opposite side of the field. Use with `relativeTo` to flip a pose to the opposite
    // alliance
    public static final Pose2d FLIPPING_POSE =
        new Pose2d(
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS), new Rotation2d(Math.PI));

    // Vision Drive Constants

    public static final double TRANSLATION_TOLERANCE_X = 0.1; // Changed from 0.05 3/26/23
    public static final double TRANSLATION_TOLERANCE_Y = 0.1; // Changed from 0.05 3/26/23
    public static final double ROTATION_TOLERANCE = 0.035;

    public static final double MAX_VELOCITY = 2; // 3
    public static final double MAX_ACCELARATION = 1; // 2
    public static final double MAX_VELOCITY_ROTATION = 8; // 8
    public static final double MAX_ACCELARATION_ROTATION = 8; // 8

    public static final double kPXController = 2.5d;
    public static final double kIXController = 0d;
    public static final double kDXController = 0d;
    public static final double kPYController = 2.5d;
    public static final double kIYController = 0d;
    public static final double kDYController = 0d;
    public static final double kPThetaController = 1.2d;
    public static final double kIThetaController = 0d;
    public static final double kDThetaController = 0d;
  }
}
