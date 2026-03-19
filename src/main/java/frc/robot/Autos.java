package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants.Position;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import frc.robot.utils.PoseUtils;
import swervelib.SwerveInputStream;

public class Autos {

        private final RobotContainer m_robotContainer;

        private final IndexerSubsystem m_indexerSubsystem;
        private final IntakeRollerSubsystem m_intakeRollerSubsystem;
        private final LinearIntakeSubsystem m_linearIntakeSubsystem;
        private final ShooterSubsystem m_shooterSubsystem;
        private final SwerveSubsystem m_swerveSubsystem;

        private final SwerveInputStream m_driveAngularVelocity;
        private final Command m_driveFieldOrientedAngularVelocity;
        private final Command m_driveFieldOrientedAutoAim;

        public Autos(RobotContainer robotContainer) {
                m_robotContainer = robotContainer;

                m_indexerSubsystem = robotContainer.m_indexerSubsystem;
                m_intakeRollerSubsystem = robotContainer.m_intakeRollerSubsystem;
                m_linearIntakeSubsystem = robotContainer.m_linearIntakeSubsystem;
                m_shooterSubsystem = robotContainer.m_shooterSubsystem;
                m_swerveSubsystem = robotContainer.m_swerveSubsystem;

                m_driveAngularVelocity = m_robotContainer.driveAngularVelocity;

                m_driveFieldOrientedAngularVelocity = m_robotContainer.driveFieldOrientedAngularVelocity;
                m_driveFieldOrientedAutoAim = m_robotContainer.driveFieldOrientedAutoAim;

                if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
                        StructArrayPublisher<Pose2d> waypointPositions = NetworkTableInstance.getDefault()
                                        .getStructArrayTopic("Waypoint Positions", Pose2d.struct)
                                        .publish();

                        Position[] waypointSelection = {
                                        Position.BLUE_STARTING_LINE_RIGHT,
                                        Position.BLUE_NEUTRAL_RIGHT_1,
                                        Position.BLUE_NEUTRAL_RIGHT_2,
                                        Position.BLUE_NEUTRAL_RIGHT_3,
                                        Position.BLUE_ALLIANCE_RIGHT_1,
                                        Position.RED_STARTING_LINE_RIGHT,
                                        Position.RED_NEUTRAL_RIGHT_1,
                                        Position.RED_NEUTRAL_RIGHT_2,
                                        Position.RED_NEUTRAL_RIGHT_3,
                                        Position.RED_ALLIANCE_RIGHT_1
                        };

                        waypointPositions.accept(
                                        java.util.Arrays.stream(waypointSelection)
                                                        .map(AutoConstants.positionToPose::get)
                                                        // .map((position) ->
                                                        // PoseUtils.flipX(AutoConstants.positionToPose.get(position)))
                                                        .toArray(Pose2d[]::new));
                }
        }

        private Command driveToWaypoint(Pose2d waypoint) {
                return new InstantCommand(
                                () -> m_swerveSubsystem.setDriveToWaypoint(waypoint))

                                // Wait until the robot is within the specified tolerance of the waypoint
                                .andThen(Commands.waitUntil(
                                                m_swerveSubsystem::isAtWaypoint));
        }

        private Command driveToWaypoint(Position position) {
                return driveToWaypoint(AutoConstants.positionToPose.get(position));
        }

        private Command driveToWaypoint(Position position, double maxTranslationalVelocity) {
                return Commands.sequence(
                                new InstantCommand(() -> updateLimits(maxTranslationalVelocity)),
                                driveToWaypoint(position));
        }

        private Command driveToWaypoint(Position position, double maxTranslationalVelocity,
                        double maxTranslationalAcceleration) {
                return Commands.sequence(
                                new InstantCommand(() -> updateLimits(maxTranslationalVelocity,
                                                maxTranslationalAcceleration)),
                                driveToWaypoint(position));
        }

        private Command driveToWaypoint(Position position, AngularVelocity maxRotationalVelocity) {
                return Commands.sequence(
                                new InstantCommand(() -> updateLimits(maxRotationalVelocity)),
                                driveToWaypoint(position));
        }

        private Command driveToWaypoint(Position position, AngularVelocity maxRotationalVelocity,
                        AngularVelocity maxRotationalAcceleration) {
                return Commands.sequence(
                                new InstantCommand(
                                                () -> updateLimits(maxRotationalVelocity, maxRotationalAcceleration)),
                                driveToWaypoint(position));
        }

        // private Command driveToWaypoint(Position position, double
        // maxTranslationalVelocity,
        // AngularVelocity maxRotationalVelocity) {
        // return Commands.sequence(
        // new InstantCommand(() -> updateLimits(maxTranslationalVelocity,
        // maxRotationalVelocity)),
        // driveToWaypoint(position));
        // }

        private Command resetOdometry(Pose2d waypoint) {
                return Commands.runOnce(() -> m_swerveSubsystem.resetOdometry(waypoint));
        }

        private Command resetOdometry(Position position) {
                return resetOdometry(AutoConstants.positionToPose.get(position));
        }

        private void updateLimits(double maxTranslationalVelocity, double maxTranslationalAcceleration,
                        double maxRotationalVelocity, double maxRotationalAcceleration) {
                m_driveAngularVelocity.driveToPose(m_swerveSubsystem::getDriveToWaypoint,
                                new ProfiledPIDController(
                                                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kP,
                                                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kI,
                                                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kD,
                                                new TrapezoidProfile.Constraints(
                                                                maxTranslationalVelocity,
                                                                maxTranslationalAcceleration)),
                                new ProfiledPIDController(
                                                SwerveConstants.DRIVE_TO_POSE_ROTATION_kP,
                                                SwerveConstants.DRIVE_TO_POSE_ROTATION_kI,
                                                SwerveConstants.DRIVE_TO_POSE_ROTATION_kD,
                                                new TrapezoidProfile.Constraints(
                                                                maxRotationalVelocity,
                                                                maxRotationalAcceleration)));
        }

        private void updateLimits(double maxTranslationalVelocity, double maxTranslationalAcceleration) {
                updateLimits(maxTranslationalVelocity, maxTranslationalAcceleration,
                                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_VELOCITY_RAD,
                                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD);
        }

        private void updateLimits(double maxTranslationalVelocity) {
                updateLimits(maxTranslationalVelocity, SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION);
        }

        private void updateLimits(AngularVelocity maxRotationalVelocity, AngularVelocity maxRotationalAcceleration) {
                updateLimits(SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_VELOCITY,
                                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION,
                                maxRotationalVelocity.in(RadiansPerSecond),
                                maxRotationalAcceleration.in(RadiansPerSecond));
        }

        private void updateLimits(AngularVelocity maxRotationalVelocity) {
                updateLimits(maxRotationalVelocity,
                                RadiansPerSecond.of(SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD));
        }

        private void updateLimits(double maxTranslationalVelocity, AngularVelocity maxRotationalVelocity) {
                updateLimits(maxTranslationalVelocity, SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION,
                                maxRotationalVelocity.in(RadiansPerSecond),
                                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD);
        }

        public Command rightNeutralAuto() {
                Position START = DriverStation.getAlliance().get() == Alliance.Red ? Position.RED_STARTING_LINE_RIGHT
                                : Position.BLUE_STARTING_LINE_RIGHT;
                Position NEUTRAL_RIGHT_1 = DriverStation.getAlliance().get() == Alliance.Red
                                ? Position.RED_NEUTRAL_RIGHT_1
                                : Position.BLUE_NEUTRAL_RIGHT_1;
                Position NEUTRAL_RIGHT_2 = DriverStation.getAlliance().get() == Alliance.Red
                                ? Position.RED_NEUTRAL_RIGHT_2
                                : Position.BLUE_NEUTRAL_RIGHT_2;
                Position NEUTRAL_RIGHT_3 = DriverStation.getAlliance().get() == Alliance.Red
                                ? Position.RED_NEUTRAL_RIGHT_3
                                : Position.BLUE_NEUTRAL_RIGHT_3;
                Position ALLIANCE_RIGHT_1 = DriverStation.getAlliance().get() == Alliance.Red
                                ? Position.RED_ALLIANCE_RIGHT_1
                                : Position.BLUE_ALLIANCE_RIGHT_1;

                // TODO: Add alliance flipping util

                return new SequentialCommandGroup(
                                resetOdometry(START),
                                new InstantCommand(
                                                () -> m_robotContainer.driveAngularVelocity.driveToPoseEnabled(true)),

                                // Drive to center line and extend/run intake
                                Commands.deadline(
                                                driveToWaypoint(NEUTRAL_RIGHT_1),
                                                m_linearIntakeSubsystem.extend(),
                                                m_intakeRollerSubsystem.intake(),
                                                m_indexerSubsystem.run()),

                                // Drive to center of field
                                driveToWaypoint(NEUTRAL_RIGHT_2),

                                // Drive back to trench
                                Commands.deadline(
                                                driveToWaypoint(NEUTRAL_RIGHT_3),
                                                m_linearIntakeSubsystem.midpoint().andThen(
                                                                Commands.parallel(
                                                                                m_intakeRollerSubsystem.stop(),
                                                                                m_indexerSubsystem.stop()))),

                                // Get in shooting position
                                driveToWaypoint(ALLIANCE_RIGHT_1),

                                Commands.parallel(
                                                m_shooterSubsystem.aimAndShootIgnoreCheck(
                                                                m_swerveSubsystem::getDistanceToTarget),
                                                m_indexerSubsystem.run(),
                                                m_intakeRollerSubsystem.intake()));

        }

}
