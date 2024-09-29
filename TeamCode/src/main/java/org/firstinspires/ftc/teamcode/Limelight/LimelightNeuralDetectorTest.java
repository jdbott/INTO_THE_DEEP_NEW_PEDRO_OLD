package org.firstinspires.ftc.teamcode.Limelight;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.HardwareClasses.Limelight;
import java.util.List;


@TeleOp(name = "LimelightDetectorTest", group = "Sensor")
public class LimelightNeuralDetectorTest extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        Limelight limelight = new Limelight(hardwareMap);

        telemetry.setMsTransmissionInterval(11);
        limelight.pipelineSwitch(1);
        limelight.start();

        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (limelight.isConnected()) {
                limelight.getDetectorResults().forEach(dr -> {
                    if ("bluesample".equals(dr.getClassName())) {
                        List<List<Double>> targetCorners = dr.getTargetCorners();

                        double diagonalPixels = DistanceEstimator.getObjectDiagonalInPixels(targetCorners);
                        double objectWidthPixels = DistanceEstimator.getObjectWidthInPixels(targetCorners);
                        double objectHeightPixels = DistanceEstimator.getObjectHeightInPixels(targetCorners);

                        // Estimate distance using diagonal or width/height based on orientation
                        double diagonalDistance = DistanceEstimator.estimateDistanceFromDiagonal(
                                Math.sqrt(Math.pow(DistanceEstimator.REAL_WORLD_WIDTH, 2) + Math.pow(DistanceEstimator.REAL_WORLD_HEIGHT, 2)),
                                diagonalPixels
                        );
                        double widthDistance = DistanceEstimator.estimateDistanceFromWidth(DistanceEstimator.REAL_WORLD_WIDTH, objectWidthPixels);
                        double heightDistance = DistanceEstimator.estimateDistanceFromHeight(DistanceEstimator.REAL_WORLD_HEIGHT, objectHeightPixels);

                        telemetry.addData("Distance (Diagonal)", "%.2f meters", diagonalDistance);
                        telemetry.addData("Distance (Width)", "%.2f meters", widthDistance);
                        telemetry.addData("Distance (Height)", "%.2f meters", heightDistance);
                    }
                });
            } else {
                telemetry.addData("Status", "Limelight Not Connected");
            }

            telemetry.update();
        }
    }
}
