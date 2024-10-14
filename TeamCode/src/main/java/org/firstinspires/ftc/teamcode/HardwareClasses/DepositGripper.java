package org.firstinspires.ftc.teamcode.HardwareClasses;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class DepositGripper {
    private final Servo clawServo;
    private final Servo rollServo;
    private final Servo pitchServo;

    public DepositGripper(HardwareMap hardwareMap) {
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        rollServo = hardwareMap.get(Servo.class, "rollServo");
        pitchServo = hardwareMap.get(Servo.class, "pitchServo");
    }

    public void CloseGripper() {
        clawServo.setPosition(0.5);
    }

    public void OpenGripper() {
        clawServo.setPosition(0.2);
    }

    public void rotateGripperToPos(double pos) {
        rollServo.setPosition(pos);
    }

    public void pitchGripperToPos(double pos) {
        pitchServo.setPosition(pos);
    }

    public void gripperGrabSpecimen() {
        rollServo.setPosition(0);
        pitchServo.setPosition(0.28);
    }

    public void gripperPlaceSpecimen() {
        rollServo.setPosition(0.7);
        pitchServo.setPosition(0.92);
    }

    public void gripperTransferSample() {
        rollServo.setPosition(0);
        pitchServo.setPosition(0);
    }
}