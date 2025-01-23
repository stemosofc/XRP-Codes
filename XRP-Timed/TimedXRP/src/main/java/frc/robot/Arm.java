package frc.robot;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class Arm {
    
    XRPServo servo;

    public Arm(int channel) {
        servo = new XRPServo(channel);
    }

    public void setAngle(double angle) {
        servo.setAngle(angle);
    }
}
