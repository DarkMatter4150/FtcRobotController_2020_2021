package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.other;/*
package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.CheckmateRobot;
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil;

*/
/*
 * This is an example of a more complex path to really test the tuning.
 *//*


@SuppressWarnings("unused")
@Autonomous(group = "drive")
public class BlueRightAbsolute extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CheckmateRobot robot = new CheckmateRobot(hardwareMap);
        Pose2d startPose = new Pose2d(-39, 63, Math.toRadians(-90));
        PositionUtil.set(startPose);
        robot.drivetrain.setPoseEstimate(startPose);
        waitForStart();

        if (isStopRequested()) return;

        sleep(3000);

        Trajectory toAllianceHub = robot.drivetrain.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(11,45))
                .build();
        robot.drivetrain.followTrajectory(toAllianceHub);



        sleep(500);

        TrajectorySequence toSpinnerSeq = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(getX(robot), getY(robot), getRot(robot)))
                .lineToLinearHeading(new Pose2d(-46, -45, Math.toRadians(0)))
                .lineToConstantHeading(new Vector2d(-46, -60))
                .build();

        robot.drivetrain.followTrajectorySequence(toSpinnerSeq);

        TrajectorySequence toWarehouse = robot.drivetrain.trajectorySequenceBuilder(new Pose2d(getX(robot), getY(robot), getRot(robot)))
                .lineToConstantHeading(new Vector2d(-40, -45))
                .lineToConstantHeading(new Vector2d(50,-45))
                .build();
        robot.drivetrain.followTrajectorySequence(toWarehouse);

        sleep(3000);

        Trajectory returnHome = robot.drivetrain.trajectoryBuilder(new Pose2d(getX(robot), getY(robot), getRot(robot)))
                .lineToLinearHeading(startPose)
                .build();
        robot.drivetrain.followTrajectory(returnHome);
        robot.cleanup();
    }

    public double getX(CheckmateRobot robot) {
        return robot.drivetrain.getPoseEstimate().getX();
    }
    public double getY(CheckmateRobot robot) {
        return robot.drivetrain.getPoseEstimate().getY();
    }
    public double getRot(CheckmateRobot robot) {
        return robot.drivetrain.getPoseEstimate().getHeading();
    }
}
*/
