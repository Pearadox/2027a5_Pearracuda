package first.robot.commands;

import org.wpilib.command2.Command;

import first.robot.subsystems.arm.Arm;

public class ArmHold extends Command {
    private Arm arm;

    public ArmHold(Arm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        arm.armHold();
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
