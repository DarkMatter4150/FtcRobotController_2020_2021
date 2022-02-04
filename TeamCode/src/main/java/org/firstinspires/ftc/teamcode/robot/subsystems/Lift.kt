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
import kotlin.math.abs

@Config
class Lift(hardwareMap: HardwareMap, private val bucket: Bucket, private val intake: Intake, private val capper: Capper) : AbstractSubsystem {
    val liftMotor1: DcMotorEx
    val liftMotor2: DcMotorEx
    val encoder: DcMotorEx

    // Keep track of the last position the motor was set to
    private var lastPosition = 0.0

    var currentPosition: Int = 0
    var direction: Double = 1.0
    var speed: Double = 0.0
    var height: Int = 0
    var smoothMode: Boolean = false
    var equation: Double = 0.0

    private var tolerance: Int = 100

    // This is weird because of the dashboard
    companion object {
        @JvmField var liftBounds = LiftBounds(0.0, 34990.0)
        @JvmField var liftSetPoints = LiftSetPoints(3000.0, 30000.0, 34700.0)
    }
    data class LiftBounds(@JvmField var min: Double, @JvmField var max: Double)
    data class LiftSetPoints(
        @JvmField var low: Double, @JvmField var mid: Double, @JvmField var high: Double)

    enum class Points {
        LOW, MID, HIGH, MIN, MAX
    }

    override fun update() {
        currentPosition = -encoder.currentPosition
        runToPosition(height)
    }


    var target: Points? = null
        set(value) {
            field = value
            when (value) {
                Points.MIN -> height = liftBounds.min.toInt()
                Points.LOW -> height = liftSetPoints.low.toInt()
                Points.MID -> height = liftSetPoints.mid.toInt()
                Points.HIGH -> height = liftSetPoints.high.toInt()
                Points.MAX -> height = liftBounds.max.toInt()
            }
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
        liftMotor1.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor1.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor1.power = 0.0

        // Initialize the motor
        liftMotor2 = hardwareMap.get(DcMotorEx::class.java, Motors.LIFT2.name)
        liftMotor2.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER

        // Reverse the motor if the config says to
        if (Motors.LIFT2.reverse) {
            liftMotor2.direction = DcMotorSimple.Direction.REVERSE
        }

        // Set it to run to a target position and hold it
        liftMotor2.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        liftMotor2.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        liftMotor2.power = 0.0

        encoder = hardwareMap.get(DcMotorEx::class.java, Motors.LEFT_FRONT.name)
        encoder.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        encoder.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
    }

    private fun runToPosition(position: Int) {
        //equation = ((0.00002)*abs(abs(position) - abs(currentPosition))) + 0.3
        if ((abs(0.1*(abs(position) - abs(currentPosition)))+400) != 0.0) {
            equation = ((-400.0)/(abs(0.1*(abs(position) - abs(currentPosition)))+400.0)) + 1.09
        }
        else {
            equation = ((0.00002)*abs(abs(position) - abs(currentPosition))) + 0.3
        }


        if ((position - currentPosition) > 0 && abs(abs(position) - abs(currentPosition)) > tolerance) {
            direction = 1.0
            speed = equation
        }
        else if ((position - currentPosition) < 0 && abs(abs(position) - abs(currentPosition)) > tolerance) {
            direction = -0.7
            speed = equation
        }
        else  {
            if (!smoothMode) {
                speed = 0.0
            }

        }

        if (capper.armPosition != Capper.ArmPositions.DOWN) {// && bucket.position != Bucket.Positions.BACKWARD) {
            liftMotor1.power = direction * speed
            liftMotor2.power = direction * speed
        }
        else {
            speed = 0.0
            height = currentPosition;
        }
    }
}