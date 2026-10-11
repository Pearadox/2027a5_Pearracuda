// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package first.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import first.robot.Constants;
import first.robot.Constants.Mode;
import first.robot.subsystems.vision.Vision;
import first.robot.util.LocalADStarAK;
import first.robot.util.SmarterDashboard;

// import org.wpilib.hardware.hal.FRCNetComm.tInstances;
// import org.wpilib.hardware.hal.FRCNetComm.tResourceType;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.Alert.Level;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
// import org.ironmaple.simulation.drivesims.COTS;
// import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
// import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase implements Vision.VisionConsumer {
    // Constants.TUNER_CONSTANTS doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY =
            new CANBus(Constants.TUNER_CONSTANTS.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(
                            Constants.TUNER_CONSTANTS.FrontLeft.LocationX,
                            Constants.TUNER_CONSTANTS.FrontLeft.LocationY),
                    Math.hypot(
                            Constants.TUNER_CONSTANTS.FrontRight.LocationX,
                            Constants.TUNER_CONSTANTS.FrontRight.LocationY)),
            Math.max(
                    Math.hypot(
                            Constants.TUNER_CONSTANTS.BackLeft.LocationX, Constants.TUNER_CONSTANTS.BackLeft.LocationY),
                    Math.hypot(
                            Constants.TUNER_CONSTANTS.BackRight.LocationX,
                            Constants.TUNER_CONSTANTS.BackRight.LocationY)));

    private double driveMultiplier = 1;
    private double turnMultiplier = 1;
    private boolean useMultiplier = false;

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = Pounds.of(114 + 18).in(Kilograms);
    private static final double ROBOT_MOI = 6.883;
    // private static final double WHEEL_COF = COTS.WHEELS.VEX_GRIP_V2.cof;
    private static final RobotConfig PP_CONFIG = new RobotConfig(
            ROBOT_MASS_KG,
            ROBOT_MOI,
            new ModuleConfig(
                    Constants.TUNER_CONSTANTS.FrontLeft.WheelRadius,
                    Constants.TUNER_CONSTANTS.kSpeedAt12Volts.in(MetersPerSecond),
                    2, // WHEEL_COF,
                    DCMotor.getKrakenX60Foc(1).withReduction(Constants.TUNER_CONSTANTS.FrontLeft.DriveMotorGearRatio),
                    Constants.TUNER_CONSTANTS.FrontLeft.SlipCurrent,
                    1),
            getModuleTranslations());

    // public static final DriveTrainSimulationConfig mapleSimConfig = DriveTrainSimulationConfig.Default()
    //         .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
    //         .withCustomModuleTranslations(getModuleTranslations())
    //         .withGyro(COTS.ofPigeon2())
    //         .withBumperSize(Inches.of(30 + 3.25 * 2), Inches.of(28 + 3.25 * 2))
    //         .withSwerveModule(new SwerveModuleSimulationConfig(
    //                 DCMotor.getKrakenX60(1),
    //                 DCMotor.getFalcon500(1),
    //                 Constants.TUNER_CONSTANTS.FrontLeft.DriveMotorGearRatio,
    //                 Constants.TUNER_CONSTANTS.FrontLeft.SteerMotorGearRatio,
    //                 Volts.of(Constants.TUNER_CONSTANTS.FrontLeft.DriveFrictionVoltage),
    //                 Volts.of(Constants.TUNER_CONSTANTS.FrontLeft.SteerFrictionVoltage),
    //                 Inches.of(2),
    //                 KilogramSquareMeters.of(Constants.TUNER_CONSTANTS.FrontLeft.SteerInertia),
    //                 WHEEL_COF));

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", Level.HIGH);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private final SwerveDrivePoseEstimator poseEstimator =
            new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;
        modules[0] = new Module(flModuleIO, 0, Constants.TUNER_CONSTANTS.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, Constants.TUNER_CONSTANTS.FrontRight);
        modules[2] = new Module(blModuleIO, 2, Constants.TUNER_CONSTANTS.BackLeft);
        modules[3] = new Module(brModuleIO, 3, Constants.TUNER_CONSTANTS.BackRight);

        // Usage reporting for swerve template
        // HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::getPose,
                this::resetOdometry,
                this::getChassisVelocities,
                this::runVelocity,
                new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> DriverStationBackend.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStationBackend.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStationBackend.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                        modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        SmarterDashboard.putBoolean("Drive/Slow Mode", useMultiplier);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisVelocities speeds) {
        // Calculate module setpoints
        speeds = speeds.discretize(0.02);
        SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(speeds);
        setpointStates = SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, Constants.TUNER_CONSTANTS.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisVelocities/Setpoints", speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisVelocities());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will return to their
     * normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleVelocity[] getModuleStates() {
        SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
        for (int i = 0; i < 4; i++) {
            states[i] = new SwerveModuleVelocity(modules[i].getVelocityMetersPerSec(), modules[i].getAngle());
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisVelocities/Measured")
    public ChassisVelocities getChassisVelocities() {
        return kinematics.toChassisVelocities(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void resetOdometry(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    @Override
    public void accept(Pose2d visionRobotPoseMeters, double timestampSeconds, Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return Constants.TUNER_CONSTANTS.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(
                    Constants.TUNER_CONSTANTS.FrontLeft.LocationX, Constants.TUNER_CONSTANTS.FrontLeft.LocationY),
            new Translation2d(
                    Constants.TUNER_CONSTANTS.FrontRight.LocationX, Constants.TUNER_CONSTANTS.FrontRight.LocationY),
            new Translation2d(
                    Constants.TUNER_CONSTANTS.BackLeft.LocationX, Constants.TUNER_CONSTANTS.BackLeft.LocationY),
            new Translation2d(
                    Constants.TUNER_CONSTANTS.BackRight.LocationX, Constants.TUNER_CONSTANTS.BackRight.LocationY)
        };
    }

    public void changeSpeedMultiplier() {
        useMultiplier = !useMultiplier;
        if (useMultiplier) {
            driveMultiplier = 0.4;
            turnMultiplier = 0.6;
        } else {
            driveMultiplier = 1;
            turnMultiplier = 1;
        }
    }

    public double getDriveMultiplier() {
        return driveMultiplier;
    }

    public double getTurnMultiplier() {
        return turnMultiplier;
    }
}
