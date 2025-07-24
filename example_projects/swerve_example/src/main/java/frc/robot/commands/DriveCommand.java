package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.DriveTrain;
import frc.robot.subsystems.drivetrain.swerveModule.SwerveModuleConstants;

import java.util.function.DoubleSupplier;


public class Drive extends Command {
    public static final double DEADBAND = 0.1; // Adjust deadband as needed

    private static final double maxSpeed = SwerveModuleConstants.MAX_WHEEL_SPEED;
    private final DriveTrain driveTrain;

    private final DoubleSupplier xVelocity;
    private final DoubleSupplier yVelocity;
    private final DoubleSupplier rotation;
    private final DoubleSupplier brake;

    public Drive(DriveTrain driveTrain, DoubleSupplier xVelocity, DoubleSupplier yVelocity, DoubleSupplier rotation, DoubleSupplier brake) {
        this.driveTrain = driveTrain;

        this.xVelocity = xVelocity;
        this.yVelocity = yVelocity;
        this.rotation = rotation;
        this.brake = brake;
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.driveTrain);
    }

    @Override
    public void initialize() {
        driveTrain.runVelocity(new ChassisSpeeds());
    }

    @Override
    public void execute() {
        double brake = this.brake.getAsDouble();

        double xVel = applyDeadband(xVelocity.getAsDouble()); // Adjust deadband as needed
        double yVel = applyDeadband(yVelocity.getAsDouble()); // Adjust deadband as needed
        double rot = applyDeadband(rotation.getAsDouble()); // Adjust deadband as needed
    }

    private static double applyDeadband(double value) {
        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }
        return value;
    }

    @Override
    public void end(boolean interrupted) {
        driveTrain.runVelocity(new ChassisSpeeds());
    }
}
