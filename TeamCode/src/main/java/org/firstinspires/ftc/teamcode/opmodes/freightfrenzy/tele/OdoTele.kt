package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.RobotPose2d
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.abstracts.Triggerables
import org.firstinspires.ftc.teamcode.robot.subsystems.OdometryDeploy
import org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers.TrackingWheelLocalizer
import org.firstinspires.ftc.teamcode.robot.util.Odometry

@Config
@TeleOp(name = "Odo TeleOp")
class OdoTele : BaseOpMode() {
    lateinit var odometry: Odometry
    override fun setup() {
        odometry = Odometry(
            RobotPose2d(
                0.0,
                0.0,
                0.0
            ),
            robot.drivetrain.getEncoder(0),
            robot.drivetrain.getEncoder(1),
            robot.drivetrain.getEncoder(2),
            telemetry
        )
        odometry.init()
        //manipulate(robot,gp1,gp2,timer, AllianceColor.BLUE)
        gp1.x.onActivate = Triggerables.TriggerableCallback {
            robot.deployer.rightPosition = OdometryDeploy.ServoPositions.RIGHT_UP
            robot.deployer.leftPosition = OdometryDeploy.ServoPositions.LEFT_UP
            robot.deployer.backPosition = OdometryDeploy.ServoPositions.BACK_UP
        }
        gp1.x.onDeactivate = Triggerables.TriggerableCallback {
            robot.deployer.rightPosition = OdometryDeploy.ServoPositions.RIGHT_DOWN
            robot.deployer.leftPosition = OdometryDeploy.ServoPositions.LEFT_DOWN
            robot.deployer.backPosition = OdometryDeploy.ServoPositions.BACK_DOWN
        }

        gp1.y.onActivate = Triggerables.TriggerableCallback { robot.drivetrain.resetIMU() }
        //gp1.x.onActivate = Triggerables.TriggerableCallback { robot.drivetrain.localizer = RealsenseLocalizer(hardwareMap)}
        gp1.a.onActivate = Triggerables.TriggerableCallback { robot.drivetrain.localizer = TrackingWheelLocalizer(hardwareMap)}

        robot.drivetrain.resetEncoderValues(2)
    }

    override fun runLoop() {

        odometry.update()

        val position = robot.drivetrain.poseEstimate
        val velocity = robot.drivetrain.poseVelocity
        //PositionUtil.set(position)
        // Print pose to telemetry
        //telemetry.addData("x", position.x)
        //telemetry.addData("y", position.y)
        //telemetry.addData("odoH", Math.toDegrees(position.heading))
        //telemetry.addData("imuHeading: ", robot.drivetrain.imuHeading)

        if (velocity != null) {
            //telemetry.addData("vX", velocity.x)
            //telemetry.addData("vY", velocity.y)
            //telemetry.addData("vH", Math.toDegrees(velocity.heading))
        }
        telemetry.update()

        when (opModeType) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.drivetrain.fieldOrientedDrive(robot, gp1)

            OpModeType.Autonomous -> {
                // Replace false here with a check to cancel the sequence
                if (false) robot.drivetrain.cancelSequence()
                if (!robot.drivetrain.isBusy) opModeType = OpModeType.TeleOp
            }
            else ->
                // If we end up here, something went horribly wrong.
                // Generally, the best plan of action is to ignore
                //  it and move on.
                opModeType = OpModeType.TeleOp
        }
    }
}