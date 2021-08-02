// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intake extends SubsystemBase {
  /** Creates a new intake. */
  Solenoid Sol1;
  public intake() {
    Sol1 = new Solenoid(Constants.OIConstants.kintakeSolenoid);
  }

  public void lower() {
    Sol1.set(true);
  }
  
  public void raise() {
    Sol1.set(false);
  }

  public boolean isLow() {
    return Sol1.get();
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
