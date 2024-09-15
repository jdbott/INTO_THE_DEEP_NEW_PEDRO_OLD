package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.HardwareClasses.ColorV3;

import java.util.Objects;

@TeleOp(name = "CRServo", group = "TeleOp")
public class CRServoTest extends LinearOpMode {

    // Declare the continuous rotation servo
    private CRServo continuousServo;
    private ColorV3 colorV3;

    // Time tracking variables for non-blocking delay
    private long reverseStartTime = 0;
    private boolean reversing = false;

    // Variable for proximity decision delay
    private long proximityDetectedTime = 0;
    private boolean proximityInRange = false;

    @Override
    public void runOpMode() {
        // Initialize the servo
        continuousServo = hardwareMap.get(CRServo.class, "CRServo");
        colorV3 = new ColorV3(hardwareMap);

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            if (colorV3.isConnected()) {
                if (reversing) {
                    // If we are currently in the reverse phase
                    if (System.currentTimeMillis() - reverseStartTime >= 1500) {
                        // If 1 second has passed, stop reversing and run forward
                        continuousServo.setPower(-1);
                        reversing = false;  // Reset the reversing flag
                    }
                } else {
                    // If proximity is within 0.75 inches, start the delay timer
                    if (colorV3.proximity() <= 0.75) {
                        if (!proximityInRange) {
                            continuousServo.setPower(0);
                            proximityInRange = true;
                            proximityDetectedTime = System.currentTimeMillis();  // Record when proximity was detected
                        }

                        // If 0.5 seconds have passed since proximity was detected
                        if (System.currentTimeMillis() - proximityDetectedTime >= 500) {
                            if (Objects.equals(colorV3.sampleColor(), "Blue")) {
                                telemetry.addLine("Correct blue sample detected");
                                continuousServo.setPower(0);  // Keep the object
                            } else {
                                // Start reversing the servo to spit out the object
                                continuousServo.setPower(1);
                                reverseStartTime = System.currentTimeMillis();  // Record the start time
                                reversing = true;  // Set the reversing flag
                            }
                            proximityInRange = false;  // Reset proximity flag after decision
                        } else {
                            // Do nothing while waiting for the 0.5 second delay
                            continuousServo.setPower(0);
                        }
                    } else {
                        // If proximity is greater than 1.5, run forward and reset proximity flag
                        continuousServo.setPower(-1);
                        proximityInRange = false;
                    }
                }
            } else {
                requestOpModeStop();  // Stop the opmode if the color sensor is disconnected
            }

            telemetry.update();  // Update telemetry data
        }
    }
}