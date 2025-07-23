// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.Util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    public void update();

    public void setAngle(Rotation2d angle);

    public Rotation2d getAngle();
}