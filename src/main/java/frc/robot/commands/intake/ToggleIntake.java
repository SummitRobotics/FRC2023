package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.State;

public class ToggleIntake extends CommandBase {

    // TODO - set these
    private final double RAISE_INTAKE_POWER = 0.0;
    private final double RAISE_ENCODER_THRESHOLD = 0.0;
    private final double LOWER_INTAKE_POWER = 0.0;
    private final double LOWER_ENCODER_THRESHOLD = 0.0;
    Intake intake;
    Type type;

    public enum Type {
        RAISE(true),
        LOWER(false);

        public final boolean value;
        Type(boolean value) {this.value = value;}

        public static Type fromBoolean(boolean value) {
            return value ? RAISE : LOWER;
        }
    }

    // No extra arguments; toggles from current position
    public ToggleIntake(Intake intake) {
        this.intake = intake;
        type = Type.fromBoolean(intake.getState() != State.RAISED);

        addRequirements(intake);
    }

    // From Type enum
    public ToggleIntake(Intake intake, Type type) {
        this.intake = intake;
        this.type = type;

        addRequirements(intake);
    }

    // From boolean; true is raise
    public ToggleIntake(Intake intake, boolean type) {
        this.intake = intake;
        this.type = Type.fromBoolean(type);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (type == Type.LOWER) intake.unlock();  
    }

    @Override
    public void execute() {
        // TODO - maybe swap < and > because we don't know which encoder threshold is max vs min
        if (type == Type.LOWER && intake.getPivotEncoderPos() <= LOWER_ENCODER_THRESHOLD) {
            intake.setState(State.LOWERED_MOVING);
        } else if (type == Type.RAISE && intake.getPivotEncoderPos() >= RAISE_ENCODER_THRESHOLD) {
            intake.setState(State.RAISED);
        } else {
            intake.setPivotMotor(type == Type.RAISE ? RAISE_INTAKE_POWER : LOWER_INTAKE_POWER);
        }
    }

    @Override
    public boolean isFinished() {
        return(
            // We've finished lowering
            (type == Type.LOWER
            && (intake.getState() == State.LOWERED_MOVING
            || intake.getState() == State.LOWERED_STATIONARY))
            // We've finished raising
            || (type == Type.RAISE
            && intake.getState() == State.RAISED)
        );
    }

    @Override
    public void end(final boolean interrupted) {
        intake.setPivotMotor(0);
        if (type == Type.RAISE) intake.lock();
    }
}
