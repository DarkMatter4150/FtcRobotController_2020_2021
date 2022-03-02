package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto.other/*
package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.auto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.robot.CheckmateRobot
import org.firstinspires.ftc.teamcode.robot.util.FieldLocations
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil
import org.firstinspires.ftc.teamcode.robot.util.toVector

*/
/*
* This is an example of a more complex path to really test the tuning.
*//*

@Autonomous(group = "drive")
class RedRightAbsolute : LinearOpMode() {
    @Throws(InterruptedException::class)
    override fun runOpMode() {
        val robot = CheckmateRobot(hardwareMap)
        val startPose = Pose2d(9, -63, Math.toRadians(90.0))
        PositionUtil.set(startPose)
        robot.drivetrain.poseEstimate = startPose
        waitForStart()
        if (isStopRequested) return
        sleep(3000)
        val toAllianceHub = robot.drivetrain.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(Pose2d(12, -53, Math.toRadians(-30.0)))
            .lineToConstantHeading(Vector2d(-5, -47))
            .build()
        robot.drivetrain.followTrajectorySequence(toAllianceHub)
        val pose = FieldLocations.RED_ALLIANCE_SHIPPING_HUB_STEP_2.pose
        val toalliancehub = robot.drivetrain.trajectorySequenceBuilder(startPose)
            .lineToLinearHeading(FieldLocations.RED_ALLIANCE_SHIPPING_HUB_STEP_1.pose)
            .lineToConstantHeading(FieldLocations.RED_ALLIANCE_SHIPPING_HUB_STEP_2.pose.toVector())
            .build()
        sleep(3000)
        val toSpinnerSeq = robot.drivetrain.trajectorySequenceBuilder(
            Pose2d(
                getX(robot),
                getY(robot),
                getRot(robot)
            )
        )
            .lineToLinearHeading(Pose2d(-46.0, -45.0, Math.toRadians(0.0)))
            .lineToConstantHeading(Vector2d(-46.0, -60.0))
            .build()
        robot.drivetrain.followTrajectorySequence(toSpinnerSeq)
        val toWarehouse = robot.drivetrain.trajectorySequenceBuilder(
            Pose2d(
                getX(robot),
                getY(robot),
                getRot(robot)
            )
        )
            .lineToConstantHeading(Vector2d(-40.0, -45.0))
            .lineToConstantHeading(Vector2d(50.0, -45.0))
            .build()
        robot.drivetrain.followTrajectorySequence(toWarehouse)
        sleep(3000)
        val returnHome =
            robot.drivetrain.trajectoryBuilder(Pose2d(getX(robot), getY(robot), getRot(robot)))
                .lineToLinearHeading(startPose)
                .build()
        robot.drivetrain.followTrajectory(returnHome)
        robot.cleanup()
    }

    fun getX(robot: CheckmateRobot): Double {
        return robot.drivetrain.poseEstimate.x
    }

    fun getY(robot: CheckmateRobot): Double {
        return robot.drivetrain.poseEstimate.y
    }

    fun getRot(robot: CheckmateRobot): Double {
        return robot.drivetrain.poseEstimate.heading
    }
}*/
