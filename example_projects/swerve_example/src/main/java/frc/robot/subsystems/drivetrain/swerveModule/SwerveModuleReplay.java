package frc.robot.subsystems.drivetrain.swerveModule;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleReplay extends SwerveModuleIO{
    /**
     * Constructor for the SwerveModuleIO class.
     *
     * @param constants The constants for the swerve module
     */
    public SwerveModuleReplay(SwerveModuleConstants constants) {
        super(constants);
    }

    @Override
    public void setTarget(SwerveModuleState targetState) {

    }

    @Override
    protected void updateStateAndPosition(SwerveModuleState state, SwerveModulePosition position) {

    }

    @Override
    protected void updateLogs(SwerveModuleInputs inputs) {

    }
}
