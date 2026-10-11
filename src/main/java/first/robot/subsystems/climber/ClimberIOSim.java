// package first.robot.subsystems.climber;

// import com.ctre.phoenix6.sim.TalonFXSimState;

// import first.robot.Constants.ClimbConstants;

// import org.wpilib.math.system.plant.DCMotor;
// import org.wpilib.math.util.Units;
// import org.wpilib.simulation.SingleJointedArmSim;

// public class ClimberIOSim extends ClimberIOReal {
//     private SingleJointedArmSim climberSim = new SingleJointedArmSim(
//             DCMotor.getKrakenX60(1),
//             ClimbConstants.GEAR_RATIO,
//             ClimbConstants.MOI,
//             ClimbConstants.CLIMBER_LENGTH,
//             ClimbConstants.MIN_ANGLE,
//             ClimbConstants.MAX_ANGLE,
//             false,
//             ClimbConstants.STARTING_ANGLE);

//     private TalonFXSimState climberSimState = climbMotor.getSimState();

//     @Override
//     public void updateInputs(ClimberIOInputs inputs) {
//         super.updateInputs(inputs);

//         climberSimState.setSupplyVoltage(12);

//         climberSim.setInputVoltage(climberSimState.getMotorVoltage());
//         climberSim.update(0.02);

//         climberSimState.setRawRotorPosition(
//                 Units.radiansToRotations(climberSim.getAngleRads() - ClimbConstants.ZERO_ANGLE)
//                         * ClimbConstants.GEAR_RATIO);

//         climberSimState.setRotorVelocity(
//                 Units.radiansToRotations(climberSim.getVelocityRadPerSec() * ClimbConstants.GEAR_RATIO));
//     }
// }
