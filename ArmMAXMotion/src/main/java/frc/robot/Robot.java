/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private SparkMax motor;
  private SparkMaxConfig motorConfig;
  private SparkClosedLoopController closedLoopController;
  private RelativeEncoder encoder;

  XboxController controle;

  public Robot() {
    controle = new XboxController(0);
    /*
     * Initialize the SPARK MAX and get its encoder and closed loop controller
     * objects for later use.
     */
    motor = new SparkMax(10, MotorType.kBrushless);
    closedLoopController = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    /*
     * Create a new SPARK MAX configuration object. This will store the
     * configuration parameters for the SPARK MAX that we will set below.
     */
    motorConfig = new SparkMaxConfig();

    motorConfig
      .idleMode(IdleMode.kBrake)
      .inverted(false);
    /*
     * Configure the encoder. For this specific example, we are using the
     * integrated encoder of the NEO, and we don't need to configure it. If
     * needed, we can adjust values like the position or velocity conversion
     * factors.
     */
    motorConfig.encoder
        .positionConversionFactor(360.0 / 200.0)
        .velocityConversionFactor(1.0 / 200.0);

    /*
     * Configure the closed loop controller. We want to make sure we set the
     * feedback sensor as the primary encoder.
     */
    motorConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        // Set PID values for position control. We don't need to pass a closed
        // loop slot, as it will default to slot 0.
        .p(0.1)
        .i(0)
        .d(0.02)
        .outputRange(-0.9,  0.9);

    motorConfig.closedLoop.maxMotion
        // Set MAXMotion parameters for position control. We don't need to pass
        // a closed loop slot, as it will default to slot 0.
        .maxVelocity(25000)
        .maxAcceleration(50000)
        .allowedClosedLoopError(0.6);


    /*
     * Apply the configuration to the SPARK MAX.
     *
     * kResetSafeParameters is used to get the SPARK MAX to a known state. This
     * is useful in case the SPARK MAX is replaced.
     *
     * kPersistParameters is used to ensure the configuration is not lost when
     * the SPARK MAX loses power. This is useful for power cycles that may occur
     * mid-operation.
     */
    motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    encoder.setPosition(0);
    // Initialize dashboard values
    SmartDashboard.putNumber("Target Position", 0);
  }

  double setPoint = 0;
  boolean state = false;
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Target Position", setPoint);
    SmartDashboard.putNumber("Output voltage", motor.getBusVoltage());
    SmartDashboard.putNumber("Percet output", motor.getAppliedOutput());
    if(!state) {
      if(controle.getAButtonPressed()) {
        setPoint = 90;
      }
      if(controle.getXButtonPressed()) {
        setPoint = 45;
      } 
      if(controle.getBButtonPressed()) {
        setPoint = 0;
      }
      closedLoopController.setReference(setPoint, ControlType.kMAXMotionPositionControl,
      ClosedLoopSlot.kSlot0);
  }
    if(controle.getYButton()) {
      motor.set(controle.getLeftY()*0.5);
      state = true;
    } else {
      state = false;
    }
    if(controle.getRightBumperButtonPressed()) {
      encoder.setPosition(0);
      setPoint = 0;
    }
  }

  @Override
  public void robotPeriodic() {
    // Display encoder position and velocity
    SmartDashboard.putNumber("Actual Position", encoder.getPosition());
    SmartDashboard.putNumber("Actual Velocity", encoder.getVelocity());
  }
}