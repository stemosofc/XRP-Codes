// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.XRPDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ForwardCommand extends Command {
  
  private final PIDController pid = new PIDController(2, 0, 0);


  private final XRPDrivetrain drivetrain;
  /** Creates a new ForwardCommand. */
  public ForwardCommand(XRPDrivetrain drivetrain, double distance) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    pid.setSetpoint(distance);
    addRequirements(drivetrain);

    drivetrain.resetEncoders();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output = pid.calculate(drivetrain.getLeftDistanceInch());

    drivetrain.arcadeDrive(output, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
