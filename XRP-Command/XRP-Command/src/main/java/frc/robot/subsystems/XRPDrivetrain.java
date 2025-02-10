// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.xrp.XRPGyro;
import edu.wpi.first.wpilibj.xrp.XRPMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class XRPDrivetrain extends SubsystemBase {
  private final Field2d field = new Field2d();
  public final DifferentialDriveOdometry odometry;
  public double a;
  private final XRPGyro gyro = new XRPGyro();

  public double kGearRatio =
      (30.0 / 14.0) * (28.0 / 16.0) * (36.0 / 9.0) * (26.0 / 8.0); // 48.75:1
  private static final double kCountsPerMotorShaftRev = 12.0;
  private static final double kCountsPerRevolution = kCountsPerMotorShaftRev * kGearRatio; // 585.0
  private static final double kWheelDiameterInch = 2.3622; // 60 mm

  private static final double MAX_VELOCITY_INCH = 14.53;

  // The XRP has the left and right motors set to
  // channels 0 and 1 respectively
  private final XRPMotor m_leftMotor = new XRPMotor(0);
  private final XRPMotor m_rightMotor = new XRPMotor(1);

  // The XRP has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);

  private double previousVelocity;
  private double acceleration;

  /** Creates a new XRPDrivetrain. */
  public XRPDrivetrain() {
    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();

    odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), Units.inchesToMeters(m_leftEncoder.getDistance()), Units.inchesToMeters(m_rightEncoder.getDistance()));
    // Invert right side since motor is flipped
    m_rightMotor.setInverted(true);

    SmartDashboard.putData("Field", field);
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getVelocity() {
    double leftSpeed = m_leftEncoder.getRate();
    double rightSpeed = m_rightEncoder.getRate();

    return (leftSpeed + rightSpeed) / 2;
  }

  public double getAcceleration() {
    return acceleration;
  }

  public double getMaxVelocityInch() {
    return MAX_VELOCITY_INCH;
  }

  public void setVoltage(double voltage) {
    m_leftMotor.setVoltage(voltage);
    m_rightMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(gyro.getRotation2d(), Units.inchesToMeters(m_leftEncoder.getDistance()), Units.inchesToMeters(m_rightEncoder.getDistance()));
    field.setRobotPose(odometry.getPoseMeters());
    double actualVelocity = getVelocity();
    double deltaVelocity =  actualVelocity - previousVelocity;
    acceleration = deltaVelocity / (1.0/20.0);

    previousVelocity = actualVelocity;

    SmartDashboard.putNumber("Actual velocity", actualVelocity);
    SmartDashboard.putNumber("Actual Acceleration", acceleration);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
