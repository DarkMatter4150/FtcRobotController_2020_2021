package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;

/*
 * This is an example of a more complex path to really test the tuning.
 */

@SuppressWarnings("unused")
@Autonomous(group = "drive")
public class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CheckmateRobot robot = new CheckmateRobot(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = robot.drivetrain.trajectoryBuilder(new Pose2d())
                .splineToSplineHeading(new Pose2d(24, 24, Math.toRadians(90)), Math.toRadians(0))
                .build();

        robot.drivetrain.followTrajectory(traj);

        sleep(2000);

        robot.drivetrain.followTrajectory(
                robot.drivetrain.trajectoryBuilder(traj.end(), true)
                        .splineToSplineHeading(new Pose2d(24, 24, Math.toRadians(90)), Math.toRadians(0))
                        .build()
        );
        robot.cleanup();
    }
}
