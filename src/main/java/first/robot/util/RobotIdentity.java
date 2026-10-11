package first.robot.util;

import org.wpilib.system.RobotController;

import first.robot.generated.EveTunerConstants;
import first.robot.generated.PearracudaTunerConstants;
import first.robot.generated.TunerConstants;

public enum RobotIdentity {
    PEARRACUDA(new PearracudaTunerConstants()), // Competition Bot (5414)
    EVE(new EveTunerConstants()); // Practice Bot (9994)

    public final TunerConstants tunerConstants;

    private RobotIdentity(TunerConstants driveConstants) {
        this.tunerConstants = driveConstants;
    }

    public static RobotIdentity getRobotIdentity() {
        String rioSerial = RobotController.getSerialNumber();

        if (rioSerial.equals("032B4B61")) {
            return EVE;
        }

        return PEARRACUDA;
    }

    public static String getRobotIdentityString() {
        String rioSerial = RobotController.getSerialNumber();

        if (rioSerial.equals("032B4B61")) {
            return "EVE";
        } else if (rioSerial.equals("032B4B64")) {
            return "PEARRACUDA";
        } else {
            return "UNKNOWN";
        }
    }

    public static String getRoboRioSerial() {
        return RobotController.getSerialNumber();
    }
}
