package frc.robot.utils;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.FieldConstants;

public class PoseUtils {
    public static Pose2d flipX(Pose2d pose) {
        return new Pose2d(
                pose.getX(),
                FieldConstants.FIELD_WIDTH - pose.getY(),
                pose.getRotation().times(-1));
    }

    public static Pose2d flipAlliance(Pose2d pose) {
        return new Pose2d(
            FieldConstants.FIELD_LENGTH - pose.getX(),
            pose.getY(),
            new Rotation2d(Degrees.of(180)).minus(pose.getRotation())
        );
    }
}
