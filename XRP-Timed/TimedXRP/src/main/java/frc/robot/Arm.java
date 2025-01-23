package frc.robot;

import edu.wpi.first.wpilibj.xrp.XRPServo;

public class Arm {
    
    XRPServo servo;

    private double angle;

    public Arm(int channel) {
        servo = new XRPServo(channel);
        angle = 0;
    }

    public void setAngle(double angle) {
        this.angle = angle;
        servo.setAngle(angle);
    }

    public double getAngle() {
        return angle;
    }
}
