package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection.ObjectDetectionType;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static final boolean buzz = false;
  public static final boolean wantDriveTestAutos = false;

  public static final String logPath = "/home/lvuser/logs";
  public static final long minFreeSpace = 1000000000; // 1 GB
  public static final int dioCoastButton = 8;
  public static final double coastButtonDelaySec = 10.0;
  public static final boolean tuningWithLoggableNumbers = true;
  public static final double brownoutVoltage = 5.75;

  public static final double loopPeriodSecs = 0.02;

  // only set this if loop cycles are significantly below 20 ms
  // to avoid starvation of critical processes
  public static final boolean realTimeCommandScheduler = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum SubsystemMode {
    DISABLED,
    NORMAL,
    TUNING
  }

  public static final SubsystemMode driveMode = SubsystemMode.NORMAL;
  public static final SubsystemMode flywheelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode hoodMode = SubsystemMode.NORMAL;
  public static final SubsystemMode spindexerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode tunnelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode turretMode = SubsystemMode.DISABLED;
  public static final SubsystemMode deployerMode = SubsystemMode.DISABLED;
  public static final SubsystemMode rollerMode = SubsystemMode.DISABLED;
  public static final SubsystemMode intakeMode = SubsystemMode.DISABLED;
  public static final SubsystemMode climberMode = SubsystemMode.DISABLED;
  public static final SubsystemMode ledMode = SubsystemMode.DISABLED;
  public static final SubsystemMode visionGlobalPose = SubsystemMode.NORMAL;
  public static final SubsystemMode visionObjectDetection = SubsystemMode.DISABLED;
  public static final SubsystemMode firingManager = SubsystemMode.TUNING;

  public static class Drive {
    public static final int gyroID = 0;
    public static final boolean zeroTurnEncoders = false; // requires drive to be in tuning mode
  }

  public static class Spindexer {
    public static final boolean dynamicVelocity = false;
    public static final double dynamicVelocityPercent = 0.9; // TODO tune

    public static final int spindexerMotorId = 4;
    public static final double supplyCurrentLimit = 40; // TODO
    public static final double statorCurrentLimit = 60;
    public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0.34;
    public static final double kV = 1.47;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double stoppedMechanismRotationsPerSec = 0.1; // TODO

    public static final double motorToMechanismRatio = 12.0; // 10 inch wheel
  }

  public static class Tunnel {
    public static final boolean dynamicVelocity = false;

    public static final int tunnelMotorId = 20;
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40; // TODO
    public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0.31;
    public static final double kV = 0.19;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double stoppedMechanismRotationsPerSec = 0.1; // TODO
    public static final double motorToMechanismRatio = 1.5; // 2 inch diameter
    public static final double minPercentVelocity = 0.95;
  }

  public static class Flywheel {
    public static final int motorId = 2;
    public static final int followerMotorId = 3;
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0.25;
    public static final double kV = 0.128;
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double allowedVelocityErrorRPS = 5.0; // TODO
    public static final boolean enableFlyWheelSensor = false;

    public static final double motorToMechanismRatio = 1; // Diameter 4

    public static final double idleMechanismRPS = 5;
    public static final int canandcolorId = 0;
    public static final double minFuelDetectionProximity = 0.2;
    public static final double allowedVelocityErrorMechanismRPS = 0.2;
    public static final int idleRPS = 30;
  }

  public static class Turret {
    public static final int motorId = 22;
    // TODO set PID
    public static final double kS = 0.28;
    public static final double kV = 0;
    public static final double kP = 150;
    public static final double mmkP = 15;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double motionMagicCruiseVelocity = 3.5;
    public static final double motionMagicAcceleration = 14.0;

    public static final double statorCurrentLimit = 60; // HACK set limits
    public static final double supplyCurrentLimit = 40; // set limits
    public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double goalToleranceDeg = 1.0;
    public static final double CANCoderOneRatio = 9.0;
    public static final double CANCoderTwoRatio = 5.0;
    public static final double CANCoderOneOffset = 0.0; // TODO find offsets
    public static final double CANCoderTwoOffset = 0.0; // find offsets
    public static final double turretGearRatio = 27;
    public static final double minPhysicalLimitDeg = -250.0; // HACK find limits
    public static final double maxPhysicalLimitDeg = 250.0; // HACK find limits
    public static final double midPointPhysicalDeg =
        (minPhysicalLimitDeg + maxPhysicalLimitDeg) / 2.0;
    public static final double maxMidPointPhysicalDeg = midPointPhysicalDeg + 180.0;
    public static final double minMidPointPhysicalDeg = midPointPhysicalDeg - 180.0;
    public static final double unwindToleranceDeg = 10.0; // HACK need to find proper tolerance
    public static final double minUnwindLimitDeg = minPhysicalLimitDeg + unwindToleranceDeg;
    public static final double maxUnwindLimitDeg = maxPhysicalLimitDeg - unwindToleranceDeg;
    public static final int CANCoderOneId = 1;
    public static final int CANCoderTwoId = 4;
    public static final Translation2d originToTurret =
        new Translation2d(Units.inchesToMeters(-4.6111), Units.inchesToMeters(5.8889));
  }

  public static class Hood {
    public static final int servoChannel = 3;
    public static final int encoderId = 3;
    public static final double gearRatio = 164 / 11.0;
    public static final double kSPulsewidthUp = 80;
    public static final double kSPulsewidthDown = 45;
    public static final double kV = 0;
    public static final double kP = 0.04;
    public static final double kI = 0.03;
    public static final double kIZone = 1.0;
    public static final double kD = 0.0;
    public static final int idleVelocity = 0;
    public static final double toleranceDeg = 0.1;
    public static final double homingVelocityThresholdRPS = 0.01;
    public static final double homingVelocity = -0.4;
    public static final double idleAngleDeg = 0;
  }

  public static class Control {
    public static final int toggle1ButtonNumber = 1; // TODO set these
    public static final int toggle4ButtonNumber = 4;
    public static final int button3ButtonNumber = 5;
    public static final int toggle3ButtonNumber = 3;
  }

  public class Rollers {
    public static final double voltageIntake = 2; // TODO
    public static final double voltageEject = -2; // TODO
    public static final int motorId = 1;
    public static final double statorCurrentLimit = 60; // TODO
    public static final double supplyCurrentLimit = 40; // TODO
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final double voltageIdle = 0;
  }

  public class Deployer {
    // 0 degrees is stowed postion
    // postive degrees when extending
    public static final double retractDeg = 0; // TODO
    public static final double extendDeg = 95; // TODO
    public static final double maxGravityDegrees = 65; // TODO
    public static final int motorId = 25;
    public static final double statorCurrentLimit = 60;
    public static final double supplyCurrentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final double kP = 1; // TODO
    public static final double kG = 2; // TODO
    public static final double kI = 0;
    public static final double kD = 0;
    public static final int CANCoderID = 2;
    public static final double sensorToMechanismRatio = 3.0; // TODO
    public static final double RotorToSensorRatio = 12.0; // TODO
    public static final double tolerance = 0.1;
    public static final double CANCoderStowed = 0.5; // TODO
  }

  public static class FiringParameters {
    private final double flywheelRPM;
    private final double hoodAngleDeg;
    private final double timeOfFlightSec;
    private final double tunnelRPS;
    private final double indexerRPS;

    public FiringParameters(
        double flywheelRPM,
        double hoodAngleDeg,
        double timeOfFlightSec,
        double tunnelRPS,
        double indexerRPS) {
      this.flywheelRPM = flywheelRPM;
      this.hoodAngleDeg = hoodAngleDeg;
      this.timeOfFlightSec = timeOfFlightSec;
      this.tunnelRPS = tunnelRPS;
      this.indexerRPS = indexerRPS;
    }

    public double getFlywheelRPM() {
      return flywheelRPM;
    }

    public double getTunnelRPS() {
      return tunnelRPS;
    }

    public double getIndexerRPS() {
      return indexerRPS;
    }

    public double getHoodAngleDeg() {
      return hoodAngleDeg;
    }

    public double getTimeOfFlightSec() {
      return timeOfFlightSec;
    }

    public static FiringParameters interpolate(
        FiringParameters start, FiringParameters end, double howFar) {
      return new FiringParameters(
          start.flywheelRPM + (end.flywheelRPM - start.flywheelRPM) * howFar,
          start.hoodAngleDeg + (end.hoodAngleDeg - start.hoodAngleDeg) * howFar,
          start.timeOfFlightSec + (end.timeOfFlightSec - start.timeOfFlightSec) * howFar,
          start.tunnelRPS + (end.tunnelRPS - start.tunnelRPS) * howFar,
          start.indexerRPS + (end.indexerRPS - start.indexerRPS) * howFar);
    }
  }

  public static class FiringManager {
    public static final double minTimeOfFlight = 0; // TODO
    public static final double maxTimeOfFlight = 4; // TODO

    public static final InterpolatingTreeMap<Double, FiringParameters> firingMapScoring =
        new InterpolatingTreeMap<Double, FiringParameters>(
            InverseInterpolator.forDouble(), FiringParameters::interpolate);
    public static final InterpolatingTreeMap<Double, FiringParameters> firingMapPassing =
        new InterpolatingTreeMap<Double, FiringParameters>(
            InverseInterpolator.forDouble(), FiringParameters::interpolate);

    // Reverse maps for velocity to distance lookup
    public static final InterpolatingDoubleTreeMap velocityToDistanceMapScoring =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap velocityToDistanceMapPassing =
        new InterpolatingDoubleTreeMap();

    public static final double latencyCompensationScoring = 0;
    public static final double latencyCompensationPassing = 0;

    // Add entry to both maps
    public static void putFiringMapEntryScoring(double meters, FiringParameters params) {
      firingMapScoring.put(meters, params);
      double velocity = meters / params.getTimeOfFlightSec();
      velocityToDistanceMapScoring.put(velocity, meters);
    }

    public static void putFiringMapEntryPassing(double meters, FiringParameters params) {
      firingMapPassing.put(meters, params);
      double velocity = meters / params.getTimeOfFlightSec();
      velocityToDistanceMapPassing.put(velocity, meters);
    }

    static {
      // Meters is center of turret to 3 inches behind center from hub

      // Shooting
      putFiringMapEntryScoring(Units.inchesToMeters(48), new FiringParameters(41, 3.5, 1, 30, 7));
      putFiringMapEntryScoring(
          Units.inchesToMeters(210), new FiringParameters(53, 24, 1, 30, 7)); // Low lob
      // High Lob, putFiringMapEntryScoring(Units.inchesToMeters(210), new FiringParameters(60, 18,
      // 1, 20, 7));
      putFiringMapEntryScoring(Units.inchesToMeters(129), new FiringParameters(45, 16, 1, 30, 7));
       putFiringMapEntryScoring(Units.inchesToMeters(164), new FiringParameters(49, 20, 1, 30, 7));

      // Passing
      putFiringMapEntryPassing(Units.inchesToMeters(144.5), new FiringParameters(39, 19, 1, 30, 7));
      putFiringMapEntryPassing(Units.inchesToMeters(302), new FiringParameters(70, 38, 1, 30, 7));
      putFiringMapEntryPassing(Units.inchesToMeters(539), new FiringParameters(90, 38, 1, 30, 7));
    }

    public static final boolean alwaysTargetAllianceZone = true;
  }

  public static class FiringTargetTranslations {
    // Right/left are determined as view from blue alliance driver station
    // TODO get exact values
    public static class Red {
      public static final Translation2d hubTranslation = FieldConstants.Red.hubTranslation;
      public static final Translation2d allianceRightTranslation = new Translation2d(14.5, 1.75);
      public static final Translation2d allianceLeftTranslation = new Translation2d(14.5, 6.25);
      public static final Translation2d neutralRightTranslation = new Translation2d(8.25, 1.75);
      public static final Translation2d neutralLeftTranslation = new Translation2d(8.25, 6.25);
    }

    public static class Blue {
      public static final Translation2d hubTranslation = FieldConstants.Blue.hubTranslation;
      public static final Translation2d allianceRightTranslation = new Translation2d(2, 1.75);
      public static final Translation2d allianceLeftTranslation = new Translation2d(2, 6.25);
      public static final Translation2d neutralRightTranslation = new Translation2d(8.25, 1.75);
      public static final Translation2d neutralLeftTranslation = new Translation2d(8.25, 6.25);
    }
  }

  public static class CANivore {
    public static final String canbusName = "Clockwork";
    public static final CANBus CANBus = new CANBus(Constants.CANivore.canbusName);
  }

  public static class HubTracker {

    public static final int preBuffer = 3; // TODO
    public static final int postBuffer = 2;
  }

  public static class Sim {

    public static final double tunnelRate = 0.2;
    public static final double spindexerRate = 0.2;
    public static final double flywheelRate = 0.1;
    public static final double servoRate = 0.2;
  }

  public static final class VisionGlobalPose {
    // TODO
    public static final boolean enableGlobalPoseTrigEstimation = false;
    // Camera names, must match names configured on coprocessor
    public static String frontRightName = "FrontRight";
    public static String frontLeftName = "FrontLeft";
    public static String backRightName = "BackRight";
    public static String backLeftName = "BackLeft";

    // Robot to camera transforms
    // TODO
    public static Transform3d frontRightTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14.199859),
                Units.inchesToMeters(-12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(-45)));
    public static Transform3d frontLeftTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14.199859),
                Units.inchesToMeters(12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(45)));
    public static Transform3d backRightTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-14.199859),
                Units.inchesToMeters(-12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(
                0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(-135)));
    public static Transform3d backLeftTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-14.199859),
                Units.inchesToMeters(12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(135)));

    // Basic filtering thresholds
    // TODO
    public static double maxAmbiguity = 0.15;
    public static double maxZError = 0.75;
    public static double maxAvgTagDistance = 3;

    public static double stdDevBaseline = 0.2;
    public static double thetaStdDevBaseline = 0.075;
  }

  public static final class VisionObjectDetection {
    public enum ObjectDetectionTarget {
      CENTROID,
      CLOSEST
    }

    public static final ObjectDetectionTarget mode = ObjectDetectionTarget.CLOSEST;

    public static final ObjectDetectionType detectionType = ObjectDetectionType.OBJECT;
    public static final Transform3d robotCenterToCamera =
        new Transform3d(
            -0.2208,
            -0.23495,
            0.98315,
            new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180)));
    public static final boolean enableObjectDetectionDebug = false;
    public static final String objectCamera = "objectCamera1";

    public static final double fuelIntakeOffset = Units.inchesToMeters(13); // TODO set this
    ; // TODO set this
  }

  public static class Autonomous {}

  public static class LED {
    public static final int CANdleID = 99;
    public static final StripTypeValue stripType = StripTypeValue.RGBW; // TODO set these
    public static final double brightnessScalar = 0.5;
    public static final int ledStart = 0;
    public static final int ledEnd = 0;
  }
}
