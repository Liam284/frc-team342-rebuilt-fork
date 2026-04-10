// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.TurretConstants.TURRET_OFFSET;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.PhotonVision;
import frc.robot.CustomXboxController;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AimAhead extends Command {
  private SwerveDrive swerve;
  private Shooter shooter;
  private Turret turret;
  private PhotonVision photonVision;
  private CustomXboxController controller;

  private Pose2d currentPose;
  private Pose2d nextRobotPose;
  private static Pose2d nextTurretPose;

  // private double averageShooterSpeed;
  // private double exitVelocity;
  // private double flightTime;
  // private double shooterHeight;
  // private double hubHeight;

  private double xDisplacement;
  private double yDisplacement;
  private double rotationDisplacement;
  private double displacedX;
  private double displacedY;
  private double displacedRotation;

  /** Creates a new ShootWhileMoving. */
  public AimAhead(SwerveDrive swerve, Shooter shooter, Turret turret, PhotonVision photonVision, CustomXboxController controller) {
    this.swerve = swerve;
    this.shooter = shooter;
    this.turret = turret;
    this.photonVision = photonVision;
    this.controller = controller;

    currentPose = swerve.getPose2d();

    // averageShooterSpeed = (shooter.getBottomTargetRPM(photonVision.getDistanceToHub(currentPose)) + shooter.getTopTargetRPM(photonVision.getDistanceToHub(currentPose))) / 2;
    // exitVelocity = ((SHOOTER_WHEEL_DIAMETERS * Math.PI * (averageShooterSpeed / 60)) * 0.0254) * 0.9;
    //TODO: get values
    // shooterHeight = 0.0; //meters
    // hubHeight = 0.0; //meters
    // flightTime = ((exitVelocity * Math.sin(Units.degreesToRadians(65))) + Math.sqrt((Math.pow(exitVelocity * Math.sin(Units.degreesToRadians(65)), 2)) + (19.612 *(shooterHeight - hubHeight)))) / 9.806;

    xDisplacement = swerve.getChassisSpeeds().vxMetersPerSecond * shooter.getFlightTime(photonVision.getDistanceToHub(currentPose));
    yDisplacement = swerve.getChassisSpeeds().vyMetersPerSecond * shooter.getFlightTime(photonVision.getDistanceToHub(currentPose));
    rotationDisplacement = swerve.getChassisSpeeds().omegaRadiansPerSecond * shooter.getFlightTime(photonVision.getDistanceToHub(currentPose));

    displacedX = currentPose.getX() + xDisplacement;
    displacedY = currentPose.getY() + yDisplacement;
    displacedRotation = currentPose.getRotation().getRadians() + rotationDisplacement;

    nextRobotPose = new Pose2d(displacedX, displacedY, new Rotation2d(displacedRotation));

    nextTurretPose = nextRobotPose.plus(TURRET_OFFSET);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turret.trackLookAheadPose(photonVision.getHubCenterPose2d(), controller);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static Pose2d getNextTurretPose() {
    return nextTurretPose;
  }
}
