package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;

@TeleOp(name = "EncoderTest", group = "Test")
public class EncoderTest extends OpMode {

    private Encoder leftEncoder;
    private Encoder rightEncoder;
    private Encoder strafeEncoder;

    @Override
    public void init() {
        // Initialize encoders like in the ThreeWheelIMULocalizer
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftFront"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightBack"));
        strafeEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftBack"));

        // Reset encoders
        leftEncoder.reset();
        rightEncoder.reset();
        strafeEncoder.reset();
    }

    @Override
    public void loop() {
        // Update the encoders
        leftEncoder.update();
        rightEncoder.update();
        strafeEncoder.update();

        // Get the delta positions of the encoders (change since last update)
        double leftDelta = leftEncoder.getDeltaPosition();
        double rightDelta = rightEncoder.getDeltaPosition();
        double strafeDelta = strafeEncoder.getDeltaPosition();

        // Display encoder delta positions on telemetry
        telemetry.addData("Left Encoder Delta", leftDelta);
        telemetry.addData("Right Encoder Delta", rightDelta);
        telemetry.addData("Strafe Encoder Delta", strafeDelta);
        telemetry.update();
    }
}