// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.*;
import frc.robot.AprilTagIDs.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera leftTurretCamera;
  private PhotonCamera rightTurretCamera;
  private PhotonCamera leftRobotCamera;
  private PhotonCamera rightRobotCamera;

  private PhotonPoseEstimator poseEstimator;
  private Pose3d robotPose;

  private List<PhotonPipelineResult> allCameraResults;
  private PhotonTrackedTarget trackedHubTag;

  /** Creates a new PhotonVision. */
  public PhotonVision() {

    leftTurretCamera = new PhotonCamera(LEFT_TURRET_CAMERA);
    rightTurretCamera = new PhotonCamera(RIGHT_TURRET_CAMERA);

    leftRobotCamera = new PhotonCamera(LEFT_ROBOT_CAMERA);
    rightRobotCamera = new PhotonCamera(RIGHT_ROBOT_CAMERA);

    allCameraResults = getAllCameraPipelines();

    poseEstimator = new PhotonPoseEstimator(
      FIELD_LAYOUT,
      new Transform3d(
        new Translation3d(
          0,
          0,
          0
        ),
        new Rotation3d(
          0,
          0, 
          0
        )
      )
    );

  }

  public boolean tagIsUsable(PhotonTrackedTarget tag) {
    return
      FIELD_LAYOUT.getTagPose(tag.getFiducialId()).isPresent() && 
      tag.getPoseAmbiguity() < AMBIGUITY_CUTOFF &&
      tag.getBestCameraToTarget().getTranslation().toTranslation2d().getNorm() < TAG_CUTOFF_DISTANCE;
  }

  public Optional<EstimatedRobotPose> getRobotPoseWithMultiTags(PhotonPipelineResult result) {
    return poseEstimator.estimateCoprocMultiTagPose(result);
  }

  public Optional<EstimatedRobotPose> getRobotPoseWithSingleTag(PhotonPipelineResult result) {
    return poseEstimator.estimateLowestAmbiguityPose(result);
  }

  public List<PhotonTrackedTarget> getAllTagsSeen() {
    List<PhotonTrackedTarget> tags = new ArrayList<>();
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          tags.add(tag);
        }
      }
    }

    return tags;
  }

  public PhotonTrackedTarget getSpecificTag(double tagID) {
    List<PhotonTrackedTarget> wantedTags = new ArrayList<>();

    for(PhotonTrackedTarget tag : getAllTagsSeen()) {
      if(tag.getFiducialId() == tagID) {
        wantedTags.add(tag);
      }
    }

    PhotonTrackedTarget firstTag = wantedTags.get(0);
    PhotonTrackedTarget previousTag = wantedTags.get(0);
    PhotonTrackedTarget bestTag = wantedTags.get(0);
    for(PhotonTrackedTarget tag : wantedTags) {
      if(tag.getFiducialId() != firstTag.getFiducialId()) {
        if(tag.getPoseAmbiguity() < previousTag.getPoseAmbiguity()) {
          bestTag = tag;
        }
      }
      previousTag = tag;
    }

    return bestTag;
  }

  public void setTrackedHubTag(PhotonTrackedTarget tag) {
    trackedHubTag = tag;
  }

  public boolean allianceHubTagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          for(hubTagsIDs tagID : hubTagsIDs.values()) {
            for(int i = 0; i < hubTagsIDs.values().length; i++) {
              if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if(tag.getFiducialId() == tagID.getRedHubTagID()) {
                  setTrackedHubTag(tag);
                  return true;
                }
              }else{
                if(tag.getFiducialId() == tagID.getBlueHubTagID()) {
                  setTrackedHubTag(tag);
                  return true;
                }
              }
            }
          }
        }
      }
    }

    return false;
  }

  public boolean allianceOutpostTagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          for(outpostTagsIDs tagID : outpostTagsIDs.values()) {
            for(int i = 0; i < outpostTagsIDs.values().length; i++) {
              if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if(tag.getFiducialId() == tagID.getRedOutpostTagID()) {
                  return true;
                }
              }else{
                if(tag.getFiducialId() == tagID.getBlueOutpostTagID()) {
                  return true;
                }
              }
            }
          }
        }
      }
    }

    return false;
  }

  public boolean allianceTowerTagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          for(towerTagsIDs tagID : towerTagsIDs.values()) {
            for(int i = 0; i < towerTagsIDs.values().length; i++) {
              if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
                if(tag.getFiducialId() == tagID.getRedTowerTagID()) {
                  return true;
                }
              }else{
                if(tag.getFiducialId() == tagID.getBlueTowerTagID()) {
                  return true;
                }
              }
            }
          }
        }
      }
    }

    return false;
  }

  public boolean tagIsPresent() {
    for(PhotonPipelineResult result : allCameraResults) {
      for(PhotonTrackedTarget tag : result.getTargets()) {
        if(tagIsUsable(tag)) {
          return true;
        }
      }
    }
    return false;
  }

  public double getHorizontalAngleToTarget(PhotonTrackedTarget tag) {
    return tag.getYaw();
  }

  public double getVerticalAngleToTarget(PhotonTrackedTarget tag) {
    return tag.getPitch();
  }

  public List<PhotonPipelineResult> getAllCameraPipelines() {
    List<PhotonPipelineResult> allCamResults = new ArrayList<>();
    List<PhotonPipelineResult> leftTurretResult = new ArrayList<>();
    List<PhotonPipelineResult> rightTurretResult = new ArrayList<>();
    List<PhotonPipelineResult> leftRobotResult = new ArrayList<>();
    List<PhotonPipelineResult> rightRobotResult = new ArrayList<>();

    leftTurretResult = leftTurretCamera.getAllUnreadResults();
    rightTurretResult = rightTurretCamera.getAllUnreadResults();
    leftRobotResult = leftRobotCamera.getAllUnreadResults();
    rightRobotResult = rightRobotCamera.getAllUnreadResults();

    allCamResults.add(leftTurretResult.get(leftTurretResult.size() - 1));
    allCamResults.add(rightTurretResult.get(rightTurretResult.size() - 1));
    allCamResults.add(leftRobotResult.get(leftRobotResult.size() - 1));
    allCamResults.add(rightRobotResult.get(rightRobotResult.size() - 1));

    return allCamResults;
  }

  public Pose3d getRobotPose3d() {
    return robotPose;
  }

  public void setRobotPose3d(Pose3d newPose3d) {
    robotPose = newPose3d;
  }

  public void updateRobotPose3d() {
    List<PhotonTrackedTarget> uniqueTags = new ArrayList<>();

    for(PhotonPipelineResult result : allCameraResults) {
      PhotonTrackedTarget firstTarget = result.getTargets().get(0);
      PhotonTrackedTarget previousTarget = result.getTargets().get(0);
      for(PhotonTrackedTarget target : result.getTargets()) {
        if(tagIsUsable(target)) {
          if(target.getFiducialId() != firstTarget.getFiducialId()) {
            if(target.getFiducialId() != previousTarget.getFiducialId()) {
              uniqueTags.add(target);
            }
          }
        }
        previousTarget = target;
      }
    }

    PhotonTrackedTarget firstTag = uniqueTags.get(0);
    PhotonTrackedTarget previousTag = uniqueTags.get(0);
    for (PhotonTrackedTarget tag : uniqueTags) {
      if(tag.getFiducialId() != firstTag.getFiducialId()) {
        if(tag.getFiducialId() != previousTag.getFiducialId()) {}
      }
      previousTag = tag;
      uniqueTags.remove(tag);
    }

    if(uniqueTags.size() <= 1) {
      getRobotPoseWithSingleTag(allCameraResults.get(allCameraResults.size() - 1));
      robotPose = getRobotPoseWithSingleTag(allCameraResults.get(allCameraResults.size() - 1)).get().estimatedPose;
    }else{
      getRobotPoseWithMultiTags(allCameraResults.get(allCameraResults.size() - 1));
      robotPose = getRobotPoseWithMultiTags(allCameraResults.get(allCameraResults.size() - 1)).get().estimatedPose;
    }
  }

  public double getRobotX() {
    return getRobotPose3d().getX();
  }

  public double getRobotY() {
    return getRobotPose3d().getY();
  }

  public double getDistanceToTag(PhotonTrackedTarget tag) {
    double xDistance = Math.abs(FIELD_LAYOUT.getTagPose(tag.getFiducialId()).get().getX() - getRobotX());
    double yDistance = Math.abs(FIELD_LAYOUT.getTagPose(tag.getFiducialId()).get().getY() - getRobotY());

    return Math.abs(Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2)));
  }

  public double getDistanceToAllianceHub() {
    return getDistanceToTag(trackedHubTag);
  }

  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);

    builder.setSmartDashboardType("PhotonVision");

    builder.addBooleanProperty("Alliance Hub Tag Seen", () -> allianceHubTagIsPresent(), null);
    builder.addBooleanProperty("Alliance Tower Tag Seen", () -> allianceTowerTagIsPresent(), null);
    builder.addBooleanProperty("Alliance Outpost Tag Seen", () -> allianceOutpostTagIsPresent(), null);
    builder.addBooleanProperty("Tag Seen", () -> tagIsPresent(), null);
    builder.addDoubleProperty("Robot X", () -> getRobotX(), null);
    builder.addDoubleProperty("Robot Y", () -> getRobotY(), null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    allCameraResults = getAllCameraPipelines();
    updateRobotPose3d();
    if(allianceHubTagIsPresent()) {
      getHorizontalAngleToTarget(trackedHubTag);
      getDistanceToAllianceHub();
    }
  }
}
