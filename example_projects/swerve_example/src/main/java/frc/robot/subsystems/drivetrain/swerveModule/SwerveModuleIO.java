// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain.swerveModule;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Add your docs here. */
public abstract class SwerveModuleIO {
    @AutoLog
    public static class SwerveModuleInputs{
        public SwerveModuleState state;
        public SwerveModulePosition position;

        public String canCoderColor = "";
    }

    protected final SwerveModuleConstants constants;

    public SwerveModuleIO(SwerveModuleConstants constants){
        this.constants = constants;
    }

    public String getName(){
        return constants.NAME;
    }

    public void update(SwerveModuleInputs inputs){
        updateStateAndPosition(inputs.state, inputs.position);

        updateLogs(inputs);
    };

    /**
     * Sets a new target for the swerve module.
     * The target should be limited to the max speed and optimized for the shortest angle
     * @param targetState The target state of the module
     */
    public abstract void setTarget(SwerveModuleState targetState);

    /**
     * Sets a new target for the swerve module.
     * The target should be limited to the max speed and optimized for the shortest angle
     * @param targetState The target state of the module
     * @param acceleration The accelration the module should be in (Used by pathPlanner),
     * also should be already in the correct direction according to the optimaztion.
     */
    public abstract void setTarget(SwerveModuleState targetState, double acceleration);

    protected abstract void updateStateAndPosition(SwerveModuleState state, SwerveModulePosition position);

    protected abstract void updateLogs(SwerveModuleInputs inputs);
}
