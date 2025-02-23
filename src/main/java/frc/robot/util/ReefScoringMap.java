package frc.robot.util;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ReefScoringMap {
    public static final Map<Integer, Pose2d> poses = Map.ofEntries(
            Map.entry(6, new Pose2d(13.690, 2.960, Rotation2d.fromDegrees(120))),
            Map.entry(7, new Pose2d(14.300, 4.030, Rotation2d.fromDegrees(180))),
            Map.entry(8, new Pose2d(13.690, 5.100, Rotation2d.fromDegrees(-120))),
            Map.entry(9, new Pose2d(12.440, 5.100, Rotation2d.fromDegrees(-60))),
            Map.entry(10, new Pose2d(11.825, 4.030, Rotation2d.kZero)),
            Map.entry(11, new Pose2d(12.440, 2.960, Rotation2d.fromDegrees(60))),
            Map.entry(17, new Pose2d(3.870, 2.960, Rotation2d.fromDegrees(60))),
            Map.entry(18, new Pose2d(3.260, 4.030, Rotation2d.kZero)),
            Map.entry(19, new Pose2d(3.870, 5.100, Rotation2d.fromDegrees(-60))),
            Map.entry(20, new Pose2d(5.100, 5.100, Rotation2d.fromDegrees(-120))),
            Map.entry(21, new Pose2d(5.720, 4.030, Rotation2d.fromDegrees(180))),
            Map.entry(22, new Pose2d(5.100, 2.960, Rotation2d.fromDegrees(120))));

    public static final Pose2d getReefPoseFromLimelightID(int id) {
        return poses.get(id);
    }
}
