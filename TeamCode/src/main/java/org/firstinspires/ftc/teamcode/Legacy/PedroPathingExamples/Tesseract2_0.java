package org.firstinspires.ftc.teamcode.Legacy.PedroPathingExamples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Arm;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.Gripper;
import org.firstinspires.ftc.teamcode.Legacy.HardwareClasses.OldRobotConstants;
import org.firstinspires.ftc.teamcode.PedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.PedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.PedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.PedroPathing.util.Timer;

@Autonomous(name = "Tesseract 2+0", group = "Pedro Auto")
public class Tesseract2_0 extends OpMode {

    // Define points for the path
    private final Point redBackdropStartPose = OldRobotConstants.RED_BACKDROP_START_POSE;
    private final Point redToLeftSpikeMarkMiddlePose = OldRobotConstants.RED_TO_LEFT_SPIKE_MARK_MIDDLE_POSE;
    private final Point redLeftSpikeMark = OldRobotConstants.RED_LEFT_SPIKE_MARK;
    private final Point redLeftBackUpFromSpikeMark = OldRobotConstants.RED_LEFT_BACK_UP_FROM_SPIKE_MARK;
    private final Point redLeftBackdrop = OldRobotConstants.RED_LEFT_BACKDROP;
    private final Point redToCornerParkingMiddlePose = OldRobotConstants.RED_TO_CORNER_PARKING_MIDDLE_POSE;
    private final Point redCornerParking = OldRobotConstants.RED_CORNER_PARKING;

    // Define arm positions
    private final double ARM_UP = OldRobotConstants.ARM_UP;
    private final double ARM_DOWN = OldRobotConstants.ARM_DOWN;

    private Telemetry telemetryA;
    private Follower follower;
    private Gripper gripper;
    private Arm arm;
    private Timer pathTimer;
    private Path toLeftSpikeMark, backUpFromLeftSpikeMarkALittle, toLeftBackdrop, parkInCorner;

    // Define the PathState enum
    public enum PathState {
        START, OPEN_GRIPPER_SPIKE, BACKUP, CLOSE_GRIPPER_SPIKE, MOVE_TO_BACKDROP, OPEN_GRIPPER_BACKDROP, PARK, DONE
    }

    private PathState pathState;

    @Override
    public void init() {
        // Initialize hardware and telemetry
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(15, -63, Math.toRadians(90)));
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        gripper = new Gripper(hardwareMap);
        arm = new Arm(hardwareMap);
        pathTimer = new Timer();
        pathState = PathState.START;
        buildPaths();
        telemetryA.addData("Status:", "initialized");
        telemetryA.update();
    }

    @Override
    public void start() {
        // Close the gripper and start the initial path
        gripper.CloseGripper();
        follower.followPath(toLeftSpikeMark, true);
        setPathState(PathState.START);
        resetRuntime();
    }

    @Override
    public void loop() {
        try {
            // Update the follower and telemetry
            follower.update();
        } catch (NullPointerException e) {
            telemetryA.addData("Error", "NullPointerException in follower.update()");
            telemetryA.addData("Message", e.getMessage());
        }
        telemetryA.update();
        arm.update();
        autoPathUpdate();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case START:
                // Transition to opening the gripper when the follower is not busy
                if (!follower.isBusy()) {
                    gripper.OpenGripper();
                    setPathState(PathState.OPEN_GRIPPER_SPIKE);
                }
                break;

            case OPEN_GRIPPER_SPIKE:
                // Open the gripper and transition to backup
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(backUpFromLeftSpikeMarkALittle, true);
                    setPathState(PathState.BACKUP);
                }
                break;

            case BACKUP:
                // Transition to closing the gripper when the follower is not busy
                if (!follower.isBusy()) {
                    gripper.CloseGripper();
                    setPathState(PathState.CLOSE_GRIPPER_SPIKE);
                }
                break;

            case CLOSE_GRIPPER_SPIKE:
                // Close the gripper and move the arm up, then transition to moving to the backdrop
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(toLeftBackdrop, true);
                    arm.moveArmToPosition(ARM_UP);
                    setPathState(PathState.MOVE_TO_BACKDROP);
                }
                break;

            case MOVE_TO_BACKDROP:
                // Transition to opening the gripper when the follower and arm are not busy
                if (!follower.isBusy() && !arm.isArmMotorBusy()) {
                    gripper.OpenGripper();
                    setPathState(PathState.OPEN_GRIPPER_BACKDROP);
                }
                break;

            case OPEN_GRIPPER_BACKDROP:
                // Open gripper and wait until it's open
                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
                    follower.followPath(parkInCorner);
                    setPathState(PathState.PARK);
                }
                break;

            case PARK:
                // Park the robot
                if (follower.getCurrentTValue() > 0.6) {
                    parkInCorner.setConstantHeadingInterpolation(180);
                    arm.moveArmToPosition(ARM_DOWN);
                    gripper.CloseGripper();
                }
                if (!follower.isBusy()) {
                    setPathState(PathState.DONE);
                }
                break;

            case DONE:
                // Stop the op mode
                requestOpModeStop();
                break;
        }
    }

    public void setPathState(PathState state) {
        // Set the path state and reset the timer
        pathState = state;
        pathTimer.resetTimer();
    }

    public void buildPaths() {
        // Build the paths using Bezier curves and lines
        toLeftSpikeMark = new Path(new BezierCurve(
                redBackdropStartPose,
                redToLeftSpikeMarkMiddlePose,
                redLeftSpikeMark));
        toLeftSpikeMark.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(137.6));

        backUpFromLeftSpikeMarkALittle = new Path(new BezierLine(
                redLeftSpikeMark,
                redLeftBackUpFromSpikeMark));

        toLeftBackdrop = new Path(new BezierCurve(
                redLeftBackUpFromSpikeMark,
                redLeftBackdrop
        ));
        toLeftBackdrop.setLinearHeadingInterpolation(Math.toRadians(137.6), Math.toRadians(0), 0.5);

        parkInCorner = new Path(new BezierCurve(
                redLeftBackdrop,
                redToCornerParkingMiddlePose,
                redCornerParking
        ));
    }
}