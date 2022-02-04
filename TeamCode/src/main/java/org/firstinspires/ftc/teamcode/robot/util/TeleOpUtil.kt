package org.firstinspires.ftc.teamcode.robot.util

import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.opmodes.freightfrenzy.tele.BlueTeleOld
import org.firstinspires.ftc.teamcode.robot.FreightFrenzyRobot
import org.firstinspires.ftc.teamcode.robot.abstracts.SuperController
import org.firstinspires.ftc.teamcode.robot.subsystems.Bucket
import org.firstinspires.ftc.teamcode.robot.subsystems.Lift
import org.firstinspires.ftc.teamcode.robot.abstracts.Triggerables.TriggerableCallback
import org.firstinspires.ftc.teamcode.robot.subsystems.Capper
import java.util.*

enum class AllianceColor(@JvmField val color: String, @JvmField val multiplier: Double) {
    BLUE("blue",1.0),
    RED("red", -1.0);
}

object TeleOpUtil {

    @JvmStatic fun manipulate(robot: FreightFrenzyRobot, gp1: SuperController, gp2: SuperController, timer: ElapsedTime, alliance: AllianceColor) {
        // Retrieve our pose from the PoseStorage.currentPose static field
        robot.drivetrain.poseEstimate = PositionUtil.get()

        // Right bumper runs the carousel
        gp2.rightBumper.onActivate = TriggerableCallback { robot.carousel.power = 0.4 * -alliance.multiplier }
        gp2.rightBumper.onDeactivate = TriggerableCallback { robot.carousel.power = 0.0 }

        gp2.y.whileActive = TriggerableCallback { robot.carousel.spinner(timer,gp2,alliance.color) }

        // Left bumper runs the carousel the other way
        gp2.leftBumper.onActivate = TriggerableCallback { robot.carousel.power = 0.4 * alliance.multiplier }
        gp2.leftBumper.onDeactivate = TriggerableCallback { robot.carousel.power = 0.0 }


        // Left stick Y axis runs the arm
        gp2.leftStickY.activationThreshold = 0.4
        gp2.leftStickY.whileActive =
            TriggerableCallback {
                robot.lift.height = robot.lift.height - BlueTeleOld.liftChangeSpeed
                robot.lift.smoothMode = true
            }
        gp2.leftStickY.whileActiveNeg =
            TriggerableCallback {
                robot.lift.height = robot.lift.height + BlueTeleOld.liftChangeSpeed
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

        gp1.a.onActivate = TriggerableCallback {
            if (robot.capper.armPosition.position == Capper.ArmPositions.INIT.position) {
                robot.capper.armPosition = Capper.ArmPositions.UP
            } else if (robot.capper.armPosition.position == Capper.ArmPositions.UP.position){
                robot.capper.armPosition = Capper.ArmPositions.DOWN
            } else {
                robot.capper.armPosition = Capper.ArmPositions.INIT
            }
            }
        gp1.a.onDeactivate = TriggerableCallback {
        }

        gp1.b.onActivate = TriggerableCallback {
            if (robot.capper.clawPosition.position != Capper.ClawPositions.CLOSE.position) {
                robot.capper.clawPosition = Capper.ClawPositions.CLOSE
            }
            else {
                robot.capper.clawPosition = Capper.ClawPositions.OPEN
            }

        }
        gp1.b.onDeactivate = TriggerableCallback {

        }

        // A & B run the intake
        /*gp2.a.onActivate = TriggerableCallback { robot.intake.power = -1.0 }
        gp2.a.onDeactivate = TriggerableCallback { robot.intake.power = 0.0 }
        gp2.b.onActivate = TriggerableCallback { robot.intake.power = 1.0 }
        gp2.b.onDeactivate = TriggerableCallback { robot.intake.power = 0.0 }*/
    }

    fun updatePosition(robot: FreightFrenzyRobot, telemetry: Telemetry, timer: ElapsedTime, runtime: Double) {
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
        telemetry.addData("Heading: ", robot.drivetrain.imuHeading)
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
}