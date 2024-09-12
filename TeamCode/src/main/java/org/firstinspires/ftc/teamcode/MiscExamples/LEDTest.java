package org.firstinspires.ftc.teamcode.MiscExamples;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="LED Test")
public class LEDTest extends LinearOpMode {

    RevBlinkinLedDriver lights;

    @Override
    public void runOpMode() {

        lights = hardwareMap.get(RevBlinkinLedDriver.class, "led");
        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);

        waitForStart();

        lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_PARTY_PALETTE);

        while (opModeIsActive()) {

        }
    }
}