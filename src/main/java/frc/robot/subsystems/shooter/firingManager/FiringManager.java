package frc.robot.subsystems.shooter.firingManager;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.FiringParameters;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class FiringManager {

  public record FiringSolution(
      double flywheelSpeedRPS,
      double hoodAngle,
      double turretAngleDeg,
      double tunnelSpeedRPS,
      double indexerSpeedRPS) {}

  public enum FiringTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  private static final LoggedTunableNumber flywheelSpeedRPM =
      new LoggedTunableNumber("FiringManager/flywheelSpeedRPM", 0);
  private static final LoggedTunableNumber hoodAngle =
      new LoggedTunableNumber("FiringManager/hoodAngle", 0);
  private static final LoggedTunableNumber turretAngleDeg =
      new LoggedTunableNumber("FiringManager/turretAngleDeg", 0);
  private static final LoggedTunableNumber tunnelSpeedRPS =
      new LoggedTunableNumber("FiringManager/tunnelSpeedRPS", 0);
  private static final LoggedTunableNumber indexerSpeedRPS =
      new LoggedTunableNumber("FiringManager/indexerSpeedRPS", 0);

  public static FiringSolution getFiringSolution(
      Translation2d robotPosition, Translation2d robotVelocity, boolean isScoring) {
    if (Constants.firingManager == Constants.SubsystemMode.TUNING) {
      Logger.recordOutput("FiringManager/requestedTuning/flywheelSpeedRPM", flywheelSpeedRPM.get());
      Logger.recordOutput("FiringManager/requestedTuning/hoodAngle", hoodAngle.get());
      Logger.recordOutput("FiringManager/requestedTuning/turretAngleDeg", turretAngleDeg.get());
      Logger.recordOutput("FiringManager/requestedTuning/tunnelSpeedRPS", tunnelSpeedRPS.get());
      Logger.recordOutput("FiringManager/requestedTuning/indexerSpeedRPS", indexerSpeedRPS.get());
      return new FiringSolution(
          flywheelSpeedRPM.get(),
          hoodAngle.get(),
          turretAngleDeg.get(),
          tunnelSpeedRPS.get(),
          indexerSpeedRPS.get());
    }

    Translation2d turretPosition = robotPosition.plus(Constants.Turret.originToTurret);

    // Project future position
    double latencyCompensation =
        isScoring
            ? Constants.FiringManager.latencyCompensationScoring
            : Constants.FiringManager.latencyCompensationPassing;
    Translation2d futurePos = turretPosition.plus(robotVelocity.times(latencyCompensation));

    Logger.recordOutput("FiringManager/futurePos", new Pose2d(futurePos, new Rotation2d()));
    // Get target vector
    Translation2d goalPosition = getShootingTarget(robotPosition);
    Logger.recordOutput("FiringManager/targetPosition", new Pose2d(goalPosition, new Rotation2d()));
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();
    Logger.recordOutput("FiringManager/distance", distance);
    Translation2d targetDirection = toGoal.div(distance);

    // Get velocity
    FiringParameters baseline =
        isScoring
            ? Constants.FiringManager.firingMapScoring.get(distance)
            : Constants.FiringManager.firingMapPassing.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();

    // Build target velocity vector
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // Compensate for robot velocity
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    Logger.recordOutput(
        "FiringManager/calculatedShootingTarget",
        new Pose2d(turretPosition.plus(shotVelocity), new Rotation2d()));

    // Get results
    Rotation2d turretAngle = new Rotation2d();
    try {
      turretAngle = shotVelocity.getAngle();
    } catch (Exception e) {

    }
    double requiredVelocity = shotVelocity.getNorm();
    Logger.recordOutput("FiringManager/requiredVelocity", requiredVelocity);

    // Use table in reverse: velocity -> effective distance → RPM
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity, isScoring);
    Logger.recordOutput("FiringManager/effectiveDistance", effectiveDistance);

    // TODO implement tunnel/indexer speed
    return getHybridFiringSolution(
        effectiveDistance,
        requiredVelocity,
        turretAngle,
        baseline.getTunnelRPS(),
        baseline.getIndexerRPS(),
        isScoring);
  }

  private static FiringSolution getHybridFiringSolution(
      double distance,
      double requiredVelocity,
      Rotation2d turretAngle,
      double tunnelSpeedRPS,
      double indexerSpeedRPS,
      boolean isScoring) {
    FiringParameters baseline =
        isScoring
            ? Constants.FiringManager.firingMapScoring.get(distance)
            : Constants.FiringManager.firingMapPassing.get(distance);
    double baselineVelocity = distance / baseline.getTimeOfFlightSec();
    double velocityRatio = requiredVelocity / baselineVelocity;

    double rpmFactor = Math.sqrt(velocityRatio);
    Logger.recordOutput("FiringManager/rpmFactor", rpmFactor);
    double hoodFactor = Math.sqrt(velocityRatio);
    Logger.recordOutput("FiringManager/hoodFactor", hoodFactor);

    double adjustedRPM = baseline.getFlywheelRPM() * rpmFactor;

    double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.getHoodAngleDeg()));

    double targetHorizFromHood = baselineVelocity * hoodFactor;
    double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
    double adjustedHood = Math.toDegrees(Math.acos(ratio));

    Logger.recordOutput("FiringManager/FiringSolution/adjustedRPM", adjustedRPM);
    Logger.recordOutput("FiringManager/FiringSolution/adjustedHood", adjustedHood);
    Logger.recordOutput("FiringManager/FiringSolution/turretAngle", turretAngle.getDegrees());
    return new FiringSolution(
        adjustedRPM, adjustedHood, turretAngle.getDegrees(), tunnelSpeedRPS, indexerSpeedRPS);
  }

  public static double velocityToEffectiveDistance(double velocity, boolean isScoring) {
    return isScoring
        ? Constants.FiringManager.velocityToDistanceMapScoring.get(velocity)
        : Constants.FiringManager.velocityToDistanceMapPassing.get(velocity);
  }

  private static Translation2d getShootingTarget(Translation2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);

    if (Robot.alliance == Alliance.Blue) {
      
      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Blue.hubTranslation;
        case RIGHT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
        case LEFT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Blue.neutralRightTranslation;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Blue.neutralLeftTranslation;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid");
          return new Translation2d();
      }
    } else {
      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Red.hubTranslation;
        case RIGHT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
        case LEFT_NEUTRAL:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Red.neutralRightTranslation;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Red.neutralLeftTranslation;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid");
          return new Translation2d();
      }
    }
  }
}
