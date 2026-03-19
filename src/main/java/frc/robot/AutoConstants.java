package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utils.PoseUtils;

public class AutoConstants {
    public static final double DEFAULT_WAYPOINT_TOLERANCE = 0.1;

    public enum Position {
        BLUE_STARTING_LINE_RIGHT,
        BLUE_NEUTRAL_RIGHT_1,
        BLUE_NEUTRAL_RIGHT_2,
        BLUE_NEUTRAL_RIGHT_3,
        BLUE_ALLIANCE_RIGHT_1,
        RED_STARTING_LINE_RIGHT,
        RED_NEUTRAL_RIGHT_1,
        RED_NEUTRAL_RIGHT_2,
        RED_NEUTRAL_RIGHT_3,
        RED_ALLIANCE_RIGHT_1
    }

    // Create map position enum to Pose2D
    public static Map<Position, Pose2d> positionToPose = Map.ofEntries(
            Map.entry(Position.BLUE_STARTING_LINE_RIGHT, new Pose2d(4.0218614, 0.632, new Rotation2d(0))),
            Map.entry(Position.BLUE_NEUTRAL_RIGHT_1, new Pose2d(8.02169132232666, 0.632, new Rotation2d(-1.3203533077139966))),
            Map.entry(Position.BLUE_NEUTRAL_RIGHT_2, new Pose2d(7.77288293838501, 3.4187989234924316, new Rotation2d(-1.3203533077139966))),
            Map.entry(Position.BLUE_NEUTRAL_RIGHT_3, new Pose2d(5.869497776031494, 0.632, new Rotation2d(Degrees.of(90)))),
            Map.entry(Position.BLUE_ALLIANCE_RIGHT_1, new Pose2d(4.015, 0.632, new Rotation2d(Degrees.of(80.25)))),
            Map.entry(Position.RED_STARTING_LINE_RIGHT, PoseUtils.flipAlliance( new Pose2d(4.0218614, 0.632, new Rotation2d(0)))),
            Map.entry(Position.RED_NEUTRAL_RIGHT_1, PoseUtils.flipAlliance(new Pose2d(8.02169132232666, 0.632, new Rotation2d(-1.3203533077139966)))),
            Map.entry(Position.RED_NEUTRAL_RIGHT_2, PoseUtils.flipAlliance(new Pose2d(7.77288293838501, 3.4187989234924316, new Rotation2d(-1.3203533077139966)))),
            Map.entry(Position.RED_NEUTRAL_RIGHT_3, PoseUtils.flipAlliance(new Pose2d(5.869497776031494, 0.632, new Rotation2d(Degrees.of(90))))),
            Map.entry(Position.RED_ALLIANCE_RIGHT_1, PoseUtils.flipAlliance(new Pose2d(4.015, 0.632, new Rotation2d(Degrees.of(80.25))))));
}
