package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareClasses.Limelight;

@TeleOp(name = "LimelightDetectorTest", group = "Sensor")
public class LimelightNeuralDetectorTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(3);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Retrieve status and other necessary information from the Limelight
            if (limelight.isConnected()) {
                limelight.getDetectorResults().forEach(dr -> {
                    if ("bluesample".equals(dr.getClassName())) {
                        telemetry.addData("Blue Sample", "Area: %.2f, targetXDegrees: %.2f, targetYDegrees: %.2f",
                                dr.getTargetArea(), dr.getTargetXDegrees(), dr.getTargetYDegrees());
                    }
                });
            } else {
                telemetry.addData("Status", "Limelight Not Connected");
            }

            telemetry.update();
        }
    }
}
