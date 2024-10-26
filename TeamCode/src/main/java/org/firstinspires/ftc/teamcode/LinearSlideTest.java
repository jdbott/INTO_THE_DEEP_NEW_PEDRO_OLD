package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;
import org.firstinspires.ftc.teamcode.HardwareClasses.LinearSlide;
import org.firstinspires.ftc.teamcode.HardwareClasses.Pivot;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Gripper;

@TeleOp
public class LinearSlideTest extends LinearOpMode {
    private LinearSlide linearSlide;

    @Override
    public void runOpMode() {
        String[] motorNames = {"slideMotor"};
        DcMotorSimple.Direction[] directions = {DcMotorSimple.Direction.REVERSE};
        LinearSlide linearSlide = new LinearSlide(hardwareMap, motorNames, directions, 112, 0, 10.5); // Example ticksPerInch and limits

        // Wait for the Play button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                linearSlide.moveSlidesToPositionInches(0);
            } else if (gamepad1.b) {
                linearSlide.moveSlidesToPositionInches(10.5);
            }

            linearSlide.update();

            telemetry.addData("Position Inches", linearSlide.slidesPositionInches());
            telemetry.addData("Is Busy", linearSlide.isSlideMotorsBusy());
            telemetry.update();
        }
    }
}