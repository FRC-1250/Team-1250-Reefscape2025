package frc.robot.util;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefScoringMap {
    public static final Map<Integer, Pose2d> poses = Map.ofEntries(
            Map.entry(6, new Pose2d(13.700, 2.930, Rotation2d.fromDegrees(120))),
            Map.entry(7, new Pose2d(14.320, 4.030, Rotation2d.fromDegrees(180))),
            Map.entry(8, new Pose2d(13.700, 5.120, Rotation2d.fromDegrees(-120))),
            Map.entry(9, new Pose2d(12.430, 5.120, Rotation2d.fromDegrees(-60))),
            Map.entry(10, new Pose2d(11.800, 4.030, Rotation2d.kZero)),
            Map.entry(11, new Pose2d(12.430, 2.930, Rotation2d.fromDegrees(60))),
            Map.entry(17, new Pose2d(3.860, 2.930, Rotation2d.fromDegrees(60))),
            Map.entry(18, new Pose2d(3.230, 4.030, Rotation2d.kZero)),
            Map.entry(19, new Pose2d(3.860, 5.120, Rotation2d.fromDegrees(-60))),
            Map.entry(20, new Pose2d(5.120, 5.120, Rotation2d.fromDegrees(-120))),
            Map.entry(21, new Pose2d(5.750, 4.030, Rotation2d.fromDegrees(180))),
            Map.entry(22, new Pose2d(5.120, 2.930, Rotation2d.fromDegrees(120))));

    public static final Pose2d getReefPoseFromLimelightID(int id) {
        return poses.get(id);
    }
}
