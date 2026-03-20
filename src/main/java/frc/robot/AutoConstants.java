package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoConstants {
    public static final double DEFAULT_WAYPOINT_TOLERANCE = 0.25;

    public enum Position {
        STARTING_LINE_RIGHT,
        NEUTRAL_RIGHT_1,
        NEUTRAL_RIGHT_2,
        NEUTRAL_RIGHT_3,
        ALLIANCE_RIGHT_1,
        RIGHT_OUTPOST_ALIGN,
        RIGHT_BACK_UP_OUTPOST,
        RIGHT_CLIMB_ALIGN
    }

    // Create map position enum to Pose2D
    public static Map<Position, Pose2d> positionToPose = Map.of(
            Position.STARTING_LINE_RIGHT, new Pose2d(4.0218614, 0.632, new Rotation2d(0)),
            Position.NEUTRAL_RIGHT_1,
            new Pose2d(8.02169132232666, 0.632, new Rotation2d(-1.3203533077139966)),
            Position.NEUTRAL_RIGHT_2,
            new Pose2d(7.77288293838501, 3.4187989234924316,
                    new Rotation2d(-1.3203533077139966)),
            Position.NEUTRAL_RIGHT_3,
            new Pose2d(5.869497776031494, 0.632, new Rotation2d(Degrees.of(90))),
            Position.ALLIANCE_RIGHT_1,
            new Pose2d(4.015, 0.632, new Rotation2d(Degrees.of(80.25))),
            Position.RIGHT_OUTPOST_ALIGN, new Pose2d(0.5992408394813538,0.6748262047767639, new Rotation2d(Degrees.of(0))),
            Position.RIGHT_BACK_UP_OUTPOST, new Pose2d(1.6838583946228027,0.6748262047767639, new Rotation2d(Degrees.of(90))),
            Position.RIGHT_CLIMB_ALIGN, new Pose2d(1.6838583946228027, 3.3163061141967773, new Rotation2d(Degrees.of(180))));
}
