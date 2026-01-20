// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class VisionConstants {
    //Add pipeline names later when all cameras are set up; grab from 10.3.42.11:5800
    public static final String LEFT_TURRET_CAMERA = "";
    public static final String RIGHT_TURRET_CAMERA = "";
    public static final String LEFT_ROBOT_CAMERA = "";
    public static final String RIGHT_ROBOT_CAMERA = "";

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    //Change these values after testing
    public static final double AMBIGUITY_CUTOFF = 0.1;
    public static final double TAG_CUTOFF_DISTANCE = 0.0;

    //Get all these values after cameras are put on robot
    public static final Translation3d LEFT_TURRET_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);
    public static final Translation3d RIGHT_TURRET_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);
    public static final Translation3d LEFT_ROBOT_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);
    public static final Translation3d RIGHT_ROBOT_CAMERA_TRANSLATION_3D = new Translation3d(0, 0, 0);

    public static final Rotation3d LEFT_TURRET_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);
    public static final Rotation3d RIGHT_TURRET_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);
    public static final Rotation3d LEFT_ROBOT_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);
    public static final Rotation3d RIGHT_ROBOT_CAMERA_ROTATION_3D = new Rotation3d(0, 0, 0);

    public static final Transform3d LEFT_TURRET_CAMERA_TRANSFORM_3D = new Transform3d(LEFT_TURRET_CAMERA_TRANSLATION_3D, LEFT_TURRET_CAMERA_ROTATION_3D);
    public static final Transform3d RIGHT_TURRET_CAMERA_TRANSFORM_3D = new Transform3d(RIGHT_TURRET_CAMERA_TRANSLATION_3D, RIGHT_TURRET_CAMERA_ROTATION_3D);
    public static final Transform3d LEFT_ROBOT_CAMERA_TRANSFORM_3D = new Transform3d(LEFT_ROBOT_CAMERA_TRANSLATION_3D, LEFT_ROBOT_CAMERA_ROTATION_3D);
    public static final Transform3d RIGHT_ROBOT_CAMERA_TRANSFORM_3D = new Transform3d(RIGHT_ROBOT_CAMERA_TRANSLATION_3D, RIGHT_ROBOT_CAMERA_ROTATION_3D);
  }

}
