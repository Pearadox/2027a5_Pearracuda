// package first.robot.subsystems.drive;

// import static org.wpilib.units.Units.RadiansPerSecond;

// import org.wpilib.math.util.Units;

// import first.robot.util.PhoenixUtil;

// import org.ironmaple.simulation.drivesims.GyroSimulation;

// public class GyroIOSim implements GyroIO {
//     private final GyroSimulation gyroSimulation;

//     public GyroIOSim(GyroSimulation gyroSimulation) {
//         this.gyroSimulation = gyroSimulation;
//     }

//     @Override
//     public void updateInputs(GyroIOInputs inputs) {
//         inputs.connected = true;
//         inputs.yawPosition = gyroSimulation.getGyroReading();
//         inputs.yawVelocityRadPerSec = Units.degreesToRadians(
//                 gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

//         inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
//         inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
//     }
// }
