package org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.robot.abstracts.BaseOpMode
import org.firstinspires.ftc.teamcode.robot.util.PositionUtil
import org.firstinspires.ftc.teamcode.robot.abstracts.Triggerables.TriggerableCallback
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import java.util.*

@Disabled
@Config
@TeleOp(name = "Blue TeleOp Old")
class BlueTeleOld : BaseOpMode() {

    private val timer = ElapsedTime()

    override fun setup() {
        // Retrieve our pose from the PoseStorage.currentPose static field
        robot.drivetrain.poseEstimate = PositionUtil.get()

        // Right bumper runs the carousel
        gp2.rightBumper.onActivate = TriggerableCallback { robot.carousel.power = 0.4 }
        gp2.rightBumper.onDeactivate = TriggerableCallback { robot.carousel.power = 0.0 }

        gp2.y.whileActive = TriggerableCallback { robot.carousel.spinner(timer,gp2,"blue") }

        // Left bumper runs the carousel the other way
        gp2.leftBumper.onActivate = TriggerableCallback { robot.carousel.power = -0.4 }
        gp2.leftBumper.onDeactivate = TriggerableCallback { robot.carousel.power = 0.0 }


        // Left stick Y axis runs the arm
        gp2.leftStickY.activationThreshold = 0.4
        gp2.leftStickY.whileActive =
            TriggerableCallback {
                robot.lift.height = robot.lift.height - liftChangeSpeed
                robot.lift.smoothMode = true
            }
        gp2.leftStickY.whileActiveNeg =
            TriggerableCallback {
                robot.lift.height = robot.lift.height + liftChangeSpeed
                robot.lift.smoothMode = true
            }

        gp2.leftStickY.onDeactivate =
            TriggerableCallback {
                robot.lift.height = robot.lift.currentPosition
                robot.lift.smoothMode = false
            }

        // Dpad does set points

        gp2.dpadUp.onActivate = TriggerableCallback { robot.lift.target = Lift.Points.HIGH }
        gp2.dpadRight.onActivate = TriggerableCallback { robot.lift.target = Lift.Points.LOW }
        gp2.dpadLeft.onActivate = TriggerableCallback { robot.lift.target = Lift.Points.MID }
        gp2.dpadDown.onActivate = TriggerableCallback { robot.lift.target = Lift.Points.MIN }

        // X wiggles the bucket
        gp2.x.onActivate = TriggerableCallback {
            robot.bucket.position = Bucket.Positions.REST
            if (!gp2.rightTrigger.active) {
                robot.intake.power = 0.8
            }
            else {
                robot.intake.power =
                    gp2.rightTrigger.rawValue.invoke().toDouble()
            }

        }
        gp2.x.onDeactivate =
            TriggerableCallback {
                robot.bucket.position = Bucket.Positions.FORWARD
                robot.intake.power = 0.0
            }

        gp2.a.whileActive = TriggerableCallback {

            if (robot.lift.height > 15000) {
                if (robot.lift.height > 34000) {
                    robot.bucket.position = Bucket.Positions.DUMP_HIGH
                }
                else {
                    robot.bucket.position = Bucket.Positions.DUMP
                }
                if (!gp2.rightTrigger.active) {
                    robot.intake.power = 0.0
                }
                else {
                    robot.intake.power =
                        -gp2.rightTrigger.rawValue.invoke().toDouble()
                }
            }

            else {
                robot.intake.power =
                    -gp2.rightTrigger.rawValue.invoke().toDouble()
            }

        }

        gp2.a.onDeactivate =
            TriggerableCallback {
                robot.bucket.position = Bucket.Positions.FORWARD
                robot.intake.power = 0.0
            }

        // Right trigger dumps the bucket
        gp2.rightTrigger.activationThreshold = 0.1
        gp2.leftTrigger.activationThreshold = 0.1

        gp2.rightTrigger.whileActive =
            TriggerableCallback { robot.intake.power =
                gp2.rightTrigger.rawValue.invoke().toDouble()
            }

        gp2.rightTrigger.onDeactivate =
            TriggerableCallback { robot.intake.power = 0.0 }

        gp2.leftTrigger.whileActive =
            TriggerableCallback { robot.intake.power =
                -gp2.leftTrigger.rawValue.invoke().toDouble() * 0.45
            }

        gp2.leftTrigger.onDeactivate =
            TriggerableCallback { robot.intake.power = 0.0 }

        gp1.y.onActivate = TriggerableCallback { robot.drivetrain.resetIMU() }

        // A & B run the intake
        /*gp2.a.onActivate = TriggerableCallback { robot.intake.power = -1.0 }
        gp2.a.onDeactivate = TriggerableCallback { robot.intake.power = 0.0 }
        gp2.b.onActivate = TriggerableCallback { robot.intake.power = 1.0 }
        gp2.b.onDeactivate = TriggerableCallback { robot.intake.power = 0.0 }*/

    }

    override fun runLoop() {
        updatePosition()
        when (opModeType) {
            OpModeType.TeleOp ->
                // Moves the robot based on the GP1 left stick
                robot.drivetrain.fieldOrientedDrive(robot, gp1)
                /*robot.drivetrain.setWeightedDrivePower(
                    Pose2d(
                        -gp1.leftStickY.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        ),
                        -gp1.leftStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        ),
                        -gp1.rightStickX.correctedValue * Range.scale(
                            gp1.rightTrigger.correctedValue.toDouble(),
                            -1.0,
                            1.0,
                            0.0,
                            1.0
                        )
                    )
                )*/
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

    private fun updatePosition() {
        val position = robot.drivetrain.poseEstimate
        val velocity = robot.drivetrain.poseVelocity
        PositionUtil.set(position)
        // Print pose to telemetry
        telemetry.addData("liftHeight", robot.lift.height)
        telemetry.addData("Current Height", robot.lift.currentPosition)
        telemetry.addData("Current Equation", robot.lift.equation)
        telemetry.addData("x", position.x)
        telemetry.addData("y", position.y)
        telemetry.addData("h", Math.toDegrees(position.heading))
        telemetry.addData("runtime", String.format(Locale.ENGLISH, "%fs", runtime))
        telemetry.addData("Elapsed Time: ", timer.milliseconds())
        if (velocity != null) {
            telemetry.addData("vX", velocity.x)
            telemetry.addData("vY", velocity.y)
            telemetry.addData("vH", Math.toDegrees(velocity.heading))
        }

        telemetry.addData("Encoder: ", robot.lift.encoder.currentPosition)
        telemetry.update()
    }

    companion object {
        var liftChangeSpeed: Int = 50
    }
}