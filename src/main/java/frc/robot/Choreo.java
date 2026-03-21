package frc.robot;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import swervelib.SwerveInputStream;

public class Choreo {

    private final AutoFactory m_autoFactory;

    private final RobotContainer m_robotContainer;

    private final IndexerSubsystem m_indexerSubsystem;
    private final IntakeRollerSubsystem m_intakeRollerSubsystem;
    // private final LinearIntakeSubsystem m_linearIntakeSubsystem;
    private final ShooterSubsystem m_shooterSubsystem;
    private final SwerveSubsystem m_swerveSubsystem;

    private final DoubleSupplier m_autoAimHeadingX;
    private final DoubleSupplier m_autoAimHeadingY;

    public final SwerveInputStream stationaryAutoAim;

    public Choreo(RobotContainer robotContainer) {
        m_robotContainer = robotContainer;

        m_autoFactory = robotContainer.m_autoFactory;

        m_indexerSubsystem = robotContainer.m_indexerSubsystem;
        m_intakeRollerSubsystem = robotContainer.m_intakeRollerSubsystem;
        // m_linearIntakeSubsystem = robotContainer.m_linearIntakeSubsystem;
        m_shooterSubsystem = robotContainer.m_shooterSubsystem;
        m_swerveSubsystem = robotContainer.m_swerveSubsystem;

        m_autoAimHeadingX = robotContainer.autoAimHeadingX();
        m_autoAimHeadingY = robotContainer.autoAimHeadingY();

        stationaryAutoAim = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
                () -> 0.0,
                () -> 0.0)
                .deadband(OperatorConstants.DEADBAND)
                .scaleTranslation(1.0)
                .allianceRelativeControl(true)
                .withControllerHeadingAxis(m_autoAimHeadingX, m_autoAimHeadingY)
                .headingWhile(true)
                .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
    }

    public Command shootPreloadAuto() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("ShootPreloadAuto"),
                // m_autoFactory.trajectoryCmd("ShootPreloadAuto"),
                new InstantCommand(
                        () -> m_swerveSubsystem.drive(new Translation2d(-0.25, 0.0), 0, false)),
                Commands.waitSeconds(3),
                m_swerveSubsystem.stop(),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake(),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true))));
    }

    public Command rightNeutralAuto() {
        return Commands.sequence(
                m_autoFactory.resetOdometry("RightAuto1"),
                m_autoFactory.trajectoryCmd("RightAuto1").deadlineFor(
                        m_intakeRollerSubsystem.intake(),
                        m_indexerSubsystem.run()),
                m_swerveSubsystem.stop(),
                Commands.waitSeconds(5).deadlineFor(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake()));
    }

    public Command rightNeutralAutoSweepTwice() {
        return Commands.sequence(
                rightNeutralAuto(),
                m_autoFactory.trajectoryCmd("RightAuto2a").deadlineFor(
                        m_shooterSubsystem.stopShooting(),
                        m_intakeRollerSubsystem.intake(),
                        m_indexerSubsystem.run()),
                Commands.parallel(
                        m_swerveSubsystem.driveFieldOriented(stationaryAutoAim),
                        m_shooterSubsystem.aimAndShootIgnoreCheck(
                                () -> m_swerveSubsystem.getDistanceToTarget(true)),
                        m_indexerSubsystem.run(),
                        m_intakeRollerSubsystem.intake()));
    }

    public Command rightNeutralAutoThenClimb() {
        return Commands.sequence(
                rightNeutralAuto(),
                m_autoFactory.trajectoryCmd("RightAuto2b1").deadlineFor(
                        m_intakeRollerSubsystem.stop(),
                        m_indexerSubsystem.stop()));
    }
}