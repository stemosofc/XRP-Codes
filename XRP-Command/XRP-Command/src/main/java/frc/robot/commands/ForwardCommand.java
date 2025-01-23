// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardCommand extends Command {
  
  private final PIDController pid = new PIDController(2, 0, 0);
  XRPDrivetrain drivetrain;
  /** Creates a new ForwardCommand. */
  public ForwardCommand(XRPDrivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    pid.setSetpoint(distance);
    pid.setTolerance(0.05);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = pid.calculate(drivetrain.getLeftDistanceInch());

    drivetrain.arcadeDrive(MathUtil.clamp(error, -1, 1), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
