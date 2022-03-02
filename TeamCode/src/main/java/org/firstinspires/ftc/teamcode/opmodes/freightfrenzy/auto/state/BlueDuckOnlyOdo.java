package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.state;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.TrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

import java.util.Locale;
import java.util.Objects;

@Autonomous(name = "Odo Blue Duck Only", preselectTeleOp = "Blue TeleOp")
public class BlueDuckOnlyOdo extends BaseOpMode {
    //STARTING LOCATION
    //COORDINATES POSITIVE THETA IS CCW
    //-X is down
    //NEGATIVE Y is Right
    Pose2d startPose = new Pose2d(-41, 63, Math.toRadians(-90));
    public final long delay = 0;

    /**
     * Runs when the OpMode initializes
     */

    @Override
    public void setup() {
        robot.drivetrain.resetEncoderValues(2);
        robot.drivetrain.setLocalizer(new TrackingWheelLocalizer(hardwareMap));
    }

    @Override
    public void preRunLoop() {
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);
        duckSpinnerMovement();
        TrajectorySequence toStorage = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-64.5, 36, Math.toRadians(-90)))
                .build();
        robot.drivetrain.followTrajectorySequence(toStorage);
    }


    /**
     * Main OpMode loop, automatically updates the robot
     */

    @Override
    public void runLoop() {
        Pose2d position = robot.drivetrain.getPoseEstimate();
        Pose2d velocity = Objects.requireNonNull(robot.drivetrain.getPoseVelocity());
        PositionUtil.set(position);
        // Print pose to telemetry
        telemetry.addData("x", position.getX());
        telemetry.addData("y", position.getY());
        telemetry.addData("h", Math.toDegrees(position.getHeading()));
        telemetry.addData("runtime", String.format(Locale.ENGLISH, "%fs", getRuntime()));
        telemetry.addData("vX", velocity.getX());
        telemetry.addData("vY", velocity.getY());
        telemetry.addData("vH", Math.toDegrees(velocity.getHeading()));
        telemetry.update();
    }

    private void duckSpinnerMovement()
    {
        TrajectorySequence toDuckSpinner = robot.drivetrain.trajectorySequenceBuilder(robot.drivetrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(-62, 59, Math.toRadians(-80)))
                //.lineToLinearHeading(new Pose2d(-60 , -46, Math.toRadians(0)))
                //.strafeRight(6)
                .build();
        robot.drivetrain.followTrajectorySequence(toDuckSpinner);

        Trajectory toDuckSpinner2 = robot.drivetrain.trajectoryBuilder(robot.drivetrain.getPoseEstimate(),true)
                //.lineToLinearHeading(new Pose2d(-41, -46, Math.toRadians(0)))
                .lineToLinearHeading(new Pose2d(-62 , -55, Math.toRadians(17)))
                //.strafeRight(6)
                .build();
        //robot.drivetrain.followTrajectory(toDuckSpinner2);

        robot.carousel.setPower(.5);
        sleep(3000);
        robot.carousel.setPower(0);
        sleep(500);
    }

}