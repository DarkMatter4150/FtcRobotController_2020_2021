package org.firstinspires.ftc.teamcode.robot.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.teamcode.robot.HardwareNames.Motors
import org.firstinspires.ftc.teamcode.robot.abstracts.AbstractSubsystem
import kotlin.math.PI

@Config
class Lift1(hardwareMap: HardwareMap, private val bucket: Bucket, private val intake: Intake) : AbstractSubsystem {
    val liftMotor1: DcMotorEx
    val liftMotor2: DcMotorEx
    val encoder: DcMotorEx

    // Keep track of the last position the motor was set to
    private var lastPosition = 0.0

    private var tolerance: Int = 50

    // This is weird because of the dashboard
    companion object {
        @JvmField var liftBounds = LiftBounds(0.0, 26.0)
        @JvmField var liftSetPoints = LiftSetPoints(0.0,12.0, 5000.0, 21.0)
        @JvmField var runIntakeThreshold = 5.2
    }
    data class LiftBounds(@JvmField var min: Double, @JvmField var max: Double)
    data class LiftSetPoints(
        @JvmField var min: Double, @JvmField var low: Double, @JvmField var mid: Double, @JvmField var high: Double)

    enum class Points {
        LOW, MID, HIGH, MIN, MAX
    }

    // Multiplier for the height of the lift
    private val spoolDiameter = 2 // inches
    private val encoderTicksPerRev = 8192
    private val ticksPerInch = 1600.0

    // Latches to prevent calling functions multiple times per triggering event
    private var zeroPositionLatch = false
    private var runIntakeLatch = false

    override fun update() {
        val tempHeight = height

        if (!liftMotor1.isBusy) {
            if (runIntakeLatch) {
                runIntakeLatch = false
                // When the lift stops moving, stop the intake
                intake.power = 0.0
            }
        } else {
            if (tempHeight < lastPosition && tempHeight < runIntakeThreshold
                && !runIntakeLatch) {
                runIntakeLatch = true
                // When the lift starts moving down below the threshold, run the intake
                intake.power = .5
            }
        }

        if (!liftMotor1.isBusy && tempHeight == 0.0) {
            if (!zeroPositionLatch) {
                zeroPositionLatch = true
                // When the lift stops moving at the 0 point, set the set the bucket position
                //  to ZERO
                bucket.position = Bucket.Positions.FORWARD
            }
        } else {
            zeroPositionLatch = false
        }
    }

    var height: Double
        set(value) {
            // If this value is different from the last
            if (height != value) {
                lastPosition = height
                // Calculate the ticks
                val positionTicks = (
                        Range.clip(
                            value, liftBounds.min, liftBounds.max
                        ) * ticksPerInch).toInt()
                bucket.position = Bucket.Positions.BACKWARD
                //runToPosition(positionTicks)
            }
        }
        get() = (-encoder.currentPosition / ticksPerInch)

    var target: Points? = null
        set(value) {
            field = value
            when (value) {
                Points.MIN -> height = liftBounds.min
                Points.LOW -> height = liftSetPoints.low
                Points.MID -> height = liftSetPoints.mid
                Points.HIGH -> height = liftSetPoints.high
                Points.MAX -> height = liftBounds.max
            }

            //runToPosition(height)
        }

    init {
        // Initialize the motor
        liftMotor1 = hardwareMap.get(DcMotorEx::class.java, Motors.LIFT1.name)
        liftMotor1.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Reverse the motor if the config says to
        if (Motors.LIFT1.reverse) {
            liftMotor1.direction = DcMotorSimple.Direction.REVERSE
        }

        // Set it to run to a target position and hold it
        liftMotor1.targetPosition = 0
        liftMotor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor1.power = 1.0

        // Initialize the motor
        liftMotor2 = hardwareMap.get(DcMotorEx::class.java, Motors.LIFT2.name)
        liftMotor2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Reverse the motor if the config says to
        if (Motors.LIFT2.reverse) {
            liftMotor2.direction = DcMotorSimple.Direction.REVERSE
        }

        // Set it to run to a target position and hold it
        liftMotor2.targetPosition = 0
        liftMotor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor2.power = 1.0

        encoder = hardwareMap.get(DcMotorEx::class.java, Motors.LEFT_FRONT.name)
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun runToPosition(position: Int) {
        var direction: Double
        direction = 1.0
        var speed: Double
        speed = 0.3
        if ((position - height) > 0 && Math.abs(position - height) > tolerance) {
            direction = 1.0
        }
        if ((position - height) < 0 && Math.abs(position - height) > tolerance) {
            direction = -1.0
        }
        else  {
            direction = 0.0
        }

        liftMotor1.velocity = direction * speed
        liftMotor2.velocity = direction * speed
        liftMotor1.power = direction * speed
        liftMotor2.power = direction * speed
    }
}