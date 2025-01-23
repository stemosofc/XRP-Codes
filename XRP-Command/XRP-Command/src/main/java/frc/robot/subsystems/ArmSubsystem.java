// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.xrp.XRPServo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  
  private XRPServo arm;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem(int channel) {
    arm = new XRPServo(channel);
  }

  public Command setAngle(double angle) {
    return Commands.runOnce(() -> arm.setAngle(angle), this);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
