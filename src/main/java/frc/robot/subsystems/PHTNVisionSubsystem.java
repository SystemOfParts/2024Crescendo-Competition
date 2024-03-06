// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Iterator;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.commands.XboxRumbleCommand;
import frc.robot.lib.VisionHelpers;

public class PHTNVisionSubsystem extends SubsystemBase implements VisionHelpers {

  // private AprilTagFieldLayout aprilTagFieldLayout =
  // AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // load 2024 field

  private String cameraName;
  private PhotonCamera camera;
  private PhotonPipelineResult aprilTagResult;
  private boolean aprilTagHasTargets;
  private List<PhotonTrackedTarget> aprilTagTargets;
  private PhotonTrackedTarget aprilTagBestTarget;
  private AprilTagFieldLayout aprilTagFieldLayout;
  // private PhotonPoseEstimator poseEstimator;
  private int fiducialID;
  private Transform3d robotToCam;
  private double aprilTagX, aprilTagY;
  private static double aprilTagZAngle;
  private static double aprilTagZ = -1;
  private Pose2d globalPoseEstimate = new Pose2d();
  private Transform3d fieldToCamera;
  // private Field2d apriltaField2d = new Field2d();

  /** Creates a new PhotonVisionSubsystem. */
  
  public PHTNVisionSubsystem(String cameraName) {

    // Check if we have PV installed
    if (! PhotonVisionConstants.PV_PRESENT) {
        System.out.println("----=======> PV Not Working");

      return;
    }

    this.cameraName = cameraName;
    camera = new PhotonCamera("AprilTagCamera");
    aprilTagResult = new PhotonPipelineResult();
    aprilTagHasTargets = false;
    System.out.println("----=======> PV Working");

    // aprilTagFieldLayout = new
    // AprilTagFieldLayout(AprilTagFields.k2024Crescendo.m_resourceFile);
    // this.robotToCam = robotToCam;
    // poseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
    // PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

  }

  public Pose2d getRobotFieldPosePV() {

    aprilTagResult = camera.getLatestResult();
    aprilTagHasTargets = aprilTagResult.hasTargets();

    if (aprilTagHasTargets) {
      aprilTagTargets = aprilTagResult.getTargets();
      aprilTagBestTarget = aprilTagResult.getBestTarget();

      fiducialID = aprilTagBestTarget.getFiducialId();
      aprilTagX = aprilTagBestTarget.getBestCameraToTarget().getX();
      aprilTagY = aprilTagBestTarget.getBestCameraToTarget().getY();
      aprilTagZ = aprilTagBestTarget.getBestCameraToTarget().getZ();
      aprilTagZAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
      fieldToCamera = aprilTagResult.getMultiTagResult().estimatedPose.best;

      if (aprilTagResult.getMultiTagResult().estimatedPose.isPresent) { // this may need to be commented out as it depends whether the single tag pose estimation is enabled

        globalPoseEstimate = new Pose2d(fieldToCamera.getX(), fieldToCamera.getY(),
          new Rotation2d(fieldToCamera.getRotation().getX(), fieldToCamera.getRotation().getY()));
          // apriltaField2d.setRobotPose(globalPoseEstimate);
        return globalPoseEstimate;
      } 

    } 
    return null; // if no apriltags visible or the pose cannot be determined
  }
   public void getPHTNData() {
    boolean hasGoodTarget;
    aprilTagResult = camera.getLatestResult();
    aprilTagHasTargets = aprilTagResult.hasTargets();
    hasGoodTarget = aprilTagHasTargets;
    if (aprilTagHasTargets) {

      aprilTagTargets = aprilTagResult.getTargets();
      aprilTagBestTarget = aprilTagResult.getBestTarget();

      fiducialID = aprilTagBestTarget.getFiducialId();
      // we have the Fiducial ID of the Best Target, but what if we don't want to use that target?
      // check to make sure one of the offset speaker tags was the best target...
      if ((getFiducialID()!=8)&&(getFiducialID()!=4)){
        // the best target wasn't a speaker tag
        // Let's get rid of the speaker tag for now:
        aprilTagBestTarget = null;
        hasGoodTarget = false;
        if ((getFiducialID()==3)||(getFiducialID()==7)){
          // The best target is one of the offset speaker tags so we might still find a speaker tag
          // let's look through the other targets to see if the main speaker tag is included too:
          for (PhotonTrackedTarget targ : aprilTagTargets) {
            // we have a target, is it one of the speaker tags they are tags numbers 8 and 4?
            if ((targ.getFiducialId()==8)||(targ.getFiducialId()==4)){
              // if so replace the best target with this new target
              fiducialID = targ.getFiducialId();
              aprilTagBestTarget = targ;
              hasGoodTarget = true;
              // since we found a good one, let's stop looking for them
              break;
            }
          }
        }
      }
      // make sure the best target isn't null 
      if (aprilTagBestTarget != null){
        // get the data from the tag
        aprilTagX = aprilTagBestTarget.getBestCameraToTarget().getX();
        aprilTagY = aprilTagBestTarget.getBestCameraToTarget().getY();
        aprilTagZ = aprilTagBestTarget.getBestCameraToTarget().getZ();
        aprilTagZAngle = aprilTagBestTarget.getBestCameraToTarget().getRotation().getAngle();
        //fieldToCamera = aprilTagResult.getMultiTagResult().estimatedPose.best;
      }
      // if we didn't find a good target
      if (!hasGoodTarget){
        // change the hasTargets value to false so we don't try to use it
        aprilTagHasTargets = false;
      }
    } 
  }

  public boolean isApriltagVisible() {
    return this.aprilTagHasTargets;
  }

  /**
   * Gets the Fiducial ID of the AprilTag.
   * 
   * @return The Fiducial ID.
   */
  public int getFiducialID() {
    return fiducialID;
  }

  /**
   * Gets the X coordinate of the AprilTag in meters.
   * 
   * @return The X coordinate.
   */
  public double getAprilTagX() {
    return aprilTagX;
  }

  /**
   * Gets the Y coordinate of the AprilTag in meters.
   * 
   * @return The Y coordinate.
   */
  public double getAprilTagY() {
    return aprilTagY;
  }

  /**
   * Gets the Z coordinate of the AprilTag in meters.
   * 
   * @return The Z coordinate.
   */
  public  double getAprilTagZ() {
    return aprilTagZ;
  }

  /**
   * Gets the Z angle of the AprilTag in degrees.
   * 
   * @return The Z angle.
   */
  public double getAprilTagZAngle() {

    return aprilTagZAngle*(180/Math.PI);

  }

  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getPHTNData();

    if (isApriltagVisible()){
      if (((getAprilTagZAngle()-180) > -5) && ((getAprilTagZAngle()-180) < 5)){
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 1);
      } else if (((getAprilTagZAngle()-180) > -10) && ((getAprilTagZAngle()-180) < 10)){
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, .66);
      } else if (((getAprilTagZAngle()-180) > -15) && ((getAprilTagZAngle()-180) < 15)){
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, .33);
      } else if (((getAprilTagZAngle()-180) > -20) && ((getAprilTagZAngle()-180) < 20)){
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, .1);
      } else {
        RobotContainer.xboxController.setRumble(RumbleType.kBothRumble, 0);

      }
    }
  }
}