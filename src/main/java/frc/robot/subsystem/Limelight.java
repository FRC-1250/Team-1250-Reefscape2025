// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

  private final String name;
  private double fid = -1;

  public enum LimeLightPipeline {
    MORNING(0),
    EVENING(1);

    public final int pipelineId;

    LimeLightPipeline(int pipelineId) {
      this.pipelineId = pipelineId;
    }
  }

  public Limelight(String name) {
    this.name = name;
  }

  public Limelight() {
    name = "limelight";
  }

  public double getFid() {
    return fid;
  }

  public boolean isBestTagSeen(double fid) {
    return Double.compare(this.fid, fid) == 0;
  }

  public void setRobotOrientation(double headingDeg) {
    LimelightHelpers.SetRobotOrientation(name, headingDeg, 0, 0, 0, 0, 0);
  }

  public PoseEstimate getBotPoseEstimate_wpiBlue_MegaTag2() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
  }

  public double getDistanceFromTarget() {
    return LimelightHelpers.getTargetPose3d_RobotSpace(name).getMeasureZ().magnitude();
  }

  public Command switchPipeline(LimeLightPipeline limeLightPipeline) {
    return Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(name, limeLightPipeline.pipelineId));
  }

  @Override
  public void periodic() {
    fid = LimelightHelpers.getFiducialID(name);
  }
}
