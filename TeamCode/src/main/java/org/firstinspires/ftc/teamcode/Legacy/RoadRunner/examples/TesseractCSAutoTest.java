package org.firstinspires.ftc.teamcode.Legacy.RoadRunner.examples;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.ArmRR;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.GripperRR;
import org.firstinspires.ftc.teamcode.Legacy.OpenCV.CameraManagerTeamProp;
import org.firstinspires.ftc.teamcode.Legacy.RoadRunner.follower.MecanumDriveTeamChassis;

@Autonomous(name = "Tesseract Centerstage Auto Test")
public class TesseractCSAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        MecanumDriveTeamChassis drive = new MecanumDriveTeamChassis(hardwareMap, new Pose2d(10.5, -64.5, Math.toRadians(90)));

        ArmRR arm = new ArmRR(hardwareMap);
        GripperRR gripper = new GripperRR(hardwareMap);
        CameraManagerTeamProp cameraManager = new CameraManagerTeamProp(hardwareMap);
        cameraManager.initializeCamera();

        Action RedBackdropLeft;
        Action RedBackdropCenter;
        Action RedBackdropRight;
        Action desiredTrajectory = null;

        RedBackdropLeft = drive.actionBuilder(drive.pose)
                .afterTime(0, gripper.CloseGripper()) // HERE DOWN: Places purple and yellow pixel, nothing more
                .splineTo(new Vector2d(8.3, -37.3), Math.toRadians(136.7))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.33)
                .strafeTo(new Vector2d(9.7, -38.7))
                .afterTime(0, gripper.CloseGripper())
                .waitSeconds(0.33)
                .afterTime(0, arm.moveArmToPosition(300))
                .strafeToSplineHeading(new Vector2d(50, -33), Math.toRadians(0))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.33)

                .setTangent(Math.toRadians(180)) // HERE DOWN: Go through outer truss to get 2 white pixels, then come back and place on backdrop
                .afterDisp(10, arm.moveArmToPosition(85))
                .splineTo(new Vector2d(-10, -59), Math.toRadians(180))
                .splineTo(new Vector2d(-17, -59), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-55, -32, Math.toRadians(180 + 1e-6)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-59, -32, Math.toRadians(180)), Math.toRadians(180), (pose2dDual, posePath, v) -> 10)
                .afterTime(0, gripper.CloseGripper())
                .waitSeconds(0.5)
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-17, -54, Math.toRadians(0 - 1e-6)), Math.toRadians(0))
                .splineTo(new Vector2d(-10, -54), Math.toRadians(0))
                .afterTime(0, arm.moveArmToPosition(500))
                .splineTo(new Vector2d(53, -42), Math.toRadians(0))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.5)

                .setTangent(Math.toRadians(180)) // HERE DOWN: Park in corner and put arm and gripper down. Faces 180 degrees for teleop
                .splineToConstantHeading(new Vector2d(40, -52), Math.toRadians(-90))
                .afterTime(0, gripper.CloseGripper())
                .afterTime(0, arm.moveArmToPosition(0))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(57.8, -63, Math.toRadians(180)), Math.toRadians(0))

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Park in middle of field and put arm and gripper down. Faces 180 degrees for teleop
//                .splineToConstantHeading(new Vector2d(40, -22), Math.toRadians(90))
//                .afterTime(0, gripper.CloseGripper())
//                .afterTime(0, arm.moveArmToPosition(0))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(57.8, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        RedBackdropCenter = drive.actionBuilder(drive.pose)
                .afterTime(0, gripper.CloseGripper()) // HERE DOWN: Places purple and yellow pixel, nothing more
                .strafeTo(new Vector2d(10.5, -35.4))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.33)
                .strafeTo(new Vector2d(10.5, -37))
                .afterTime(0, gripper.CloseGripper())
                .waitSeconds(0.33)
                .afterTime(0, arm.moveArmToPosition(300))
                .setTangent(Math.toRadians(-90))
                .splineToSplineHeading(new Pose2d(52, -37, Math.toRadians(0)), Math.toRadians(0))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.33)

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Go through outer truss to get 2 white pixels, then come back and place on backdrop
//                .splineTo(new Vector2d(-10, -61), Math.toRadians(180))
//                .afterTime(0, arm.moveArmToPosition(100))
//                .splineTo(new Vector2d(-17, -61), Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-58, -36, Math.toRadians(180 + 1e-6)), Math.toRadians(180))
//                .afterTime(0, gripper.CloseGripper())
//                .waitSeconds(0.33)
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-17, -61, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(new Vector2d(-10, -61), Math.toRadians(0))
//                .afterTime(0, arm.moveArmToPosition(300))
//                .splineTo(new Vector2d(52, -42), Math.toRadians(0))
//                .afterTime(0, gripper.OpenGripper())
//                .waitSeconds(0.33)

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Park in corner and put arm and gripper down. Faces 180 degrees for teleop
//                .splineToConstantHeading(new Vector2d(40, -52), Math.toRadians(-90))
//                .afterTime(0, gripper.CloseGripper())
//                .afterTime(0, arm.moveArmToPosition(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(57.8, -63, Math.toRadians(180)), Math.toRadians(0))

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Park in middle of field and put arm and gripper down. Faces 180 degrees for teleop
//                .splineToConstantHeading(new Vector2d(40, -22), Math.toRadians(90))
//                .afterTime(0, gripper.CloseGripper())
//                .afterTime(0, arm.moveArmToPosition(0))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(57.8, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        RedBackdropRight = drive.actionBuilder(drive.pose)
                .afterTime(0, gripper.CloseGripper()) // HERE DOWN: Places purple and yellow pixel, nothing more
                .strafeTo(new Vector2d(23, -45))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.33)
                .strafeTo(new Vector2d(23, -46.1))
                .afterTime(0, gripper.CloseGripper())
                .waitSeconds(0.33)
                .afterTime(0, arm.moveArmToPosition(300))
                .strafeToSplineHeading(new Vector2d(52, -42), Math.toRadians(0))
                .afterTime(0, gripper.OpenGripper())
                .waitSeconds(0.33)

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Go through outer truss to get 2 white pixels, then come back and place on backdrop
//                .splineTo(new Vector2d(-10, -61), Math.toRadians(180))
//                .afterTime(0, arm.moveArmToPosition(100))
//                .splineTo(new Vector2d(-17, -61), Math.toRadians(180))
//                .splineToSplineHeading(new Pose2d(-58, -36, Math.toRadians(180 + 1e-6)), Math.toRadians(180))
//                .afterTime(0, gripper.CloseGripper())
//                .waitSeconds(0.33)
//                .setTangent(Math.toRadians(0))
//                .splineToSplineHeading(new Pose2d(-17, -61, Math.toRadians(0)), Math.toRadians(0))
//                .splineTo(new Vector2d(-10, -61), Math.toRadians(0))
//                .afterTime(0, arm.moveArmToPosition(300))
//                .splineTo(new Vector2d(52, -42), Math.toRadians(0))
//                .afterTime(0, gripper.OpenGripper())
//                .waitSeconds(0.33)

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Park in corner and put arm and gripper down. Faces 180 degrees for teleop
//                .splineToConstantHeading(new Vector2d(40, -52), Math.toRadians(-90))
//                .afterTime(0, gripper.CloseGripper())
//                .afterTime(0, arm.moveArmToPosition(0))
//                .setTangent(Math.toRadians(-90))
//                .splineToSplineHeading(new Pose2d(57.8, -63, Math.toRadians(180)), Math.toRadians(0))

//                .setTangent(Math.toRadians(180)) // HERE DOWN: Park in middle of field and put arm and gripper down. Faces 180 degrees for teleop
//                .splineToConstantHeading(new Vector2d(40, -22), Math.toRadians(90))
//                .afterTime(0, gripper.CloseGripper())
//                .afterTime(0, arm.moveArmToPosition(0))
//                .setTangent(Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(57.8, -11, Math.toRadians(180)), Math.toRadians(0))
                .build();

        while (opModeInInit()) {

            String mostRedSection = cameraManager.getMostRedSection();

            switch (mostRedSection) {
                case "left":
                    desiredTrajectory = RedBackdropLeft;
                    break;
                case "center":
                    desiredTrajectory = RedBackdropCenter;
                    break;
                default:
                    desiredTrajectory = RedBackdropRight;
                    break;
            }

            telemetry.addData("Section with most red, chosen auto", mostRedSection);
            telemetry.update();

            sleep(50);
        }

        waitForStart();

        if (isStopRequested()) return;

        if (desiredTrajectory == null) desiredTrajectory = RedBackdropRight;

        Actions.runBlocking(
                new ParallelAction(
                        desiredTrajectory,
                        arm.correctArmPosition()
                )
        );
    }
}