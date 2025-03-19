// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Limelight extends SubsystemBase {

  private final String name;

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
    return LimelightHelpers.getFiducialID(name);
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
    return Commands.runOnce(() -> LimelightHelpers.setPipelineIndex(name, limeLightPipeline.pipelineId))
        .withName(String.format("%s set pipeline %s", name, limeLightPipeline.toString()))
        .ignoringDisable(true);
  }

  @Override
  public void periodic() {
  }
}
