package org.firstinspires.ftc.teamcode.Legacy.HardwareClasses;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    private final DcMotorEx armMotor;

    double lastSetArmPos = 0;
    boolean armMotorBusy = false;

    public Arm(HardwareMap hardwareMap) {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveArmToPosition(double targetPos) {
        lastSetArmPos = targetPos;

        double error = targetPos - armMotor.getCurrentPosition(); // Change error calculation

        if (Math.abs(error) > 20) { // Ensure error threshold is handled correctly
            armMotor.setPower(Range.clip((error * 0.005), -1, 1)); // Adjust power direction
            armMotorBusy = true;
        } else {
            armMotor.setPower(0);
            armMotorBusy = false;
        }
    }

    public void correctArmPosition() {
        if (!armMotorBusy) {
            double error = lastSetArmPos - armMotor.getCurrentPosition(); // Change error calculation
            armMotor.setPower(Range.clip((error * 0.005), -0.5, 0.5)); // Adjust power direction
        }
    }

    public void update() {
        if (isArmMotorBusy()) {
            moveArmToPosition(lastSetArmPos);
        } else {
            correctArmPosition();
        }
    }

    public boolean isArmMotorBusy() {
        return armMotorBusy;
    }
}