package org.firstinspires.ftc.teamcode.robot.subsystems.drivetrain.localizers

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.hardware.HardwareMap
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.teamcode.robot.HardwareNames
import org.firstinspires.ftc.teamcode.robot.util.Encoder

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
class TrackingWheelLocalizer(hardwareMap: HardwareMap) : ThreeTrackingWheelLocalizer(
    listOf(
        Pose2d(0.0, LATERAL_DISTANCE / 2.0, 0.0),  // left
        Pose2d(0.0, -LATERAL_DISTANCE / 2.0, 0.0),  // right
        Pose2d(FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
    )
) {
     val leftEncoder: Encoder
     val rightEncoder: Encoder
     val backEncoder: Encoder
    override fun getWheelPositions(): List<Double> {
        return listOf(
            encoderTicksToInches(leftEncoder.currentPosition.toDouble()) * X_MULTIPLIER,
            encoderTicksToInches(rightEncoder.currentPosition.toDouble()) * -X_MULTIPLIER,
            encoderTicksToInches(backEncoder.currentPosition.toDouble()) * Y_MULTIPLIER
        )
    }

    override fun getWheelVelocities(): List<Double> {
        // DONE: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method
        return listOf(
            encoderTicksToInches(leftEncoder.correctedVelocity),
            encoderTicksToInches(rightEncoder.correctedVelocity),
            encoderTicksToInches(backEncoder.correctedVelocity)
        )
    }

    companion object {

        // TODO: CHANGE THESE CONSTANTS
        @JvmField var TICKS_PER_REV = 8192.0
        @JvmField var WHEEL_RADIUS = 0.944882 // in, = 24 mm
        @JvmField var GEAR_RATIO = 1.0 // output (wheel) speed / input (encoder) speed
        @JvmField
        var LATERAL_DISTANCE = 14.75 // in; distance between the left and right wheels
        @JvmField var FORWARD_OFFSET = -6.6 // in; offset of the lateral wheel
        @JvmField var X_MULTIPLIER = 1.0 // Multiplier in the X direction
        @JvmField var Y_MULTIPLIER = 1.0 // Multiplier in the Y direction

        fun encoderTicksToInches(ticks: Double): Double {
            return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV
        }
    }

    init {
        rightEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, HardwareNames.Encoders.RIGHT.name)
        )
        leftEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, HardwareNames.Encoders.LEFT.name)
        )
        backEncoder = Encoder(
            hardwareMap.get(DcMotorEx::class.java, HardwareNames.Encoders.FRONT.name)
        )

        /*
        if (HardwareNames.Encoders.RIGHT.reverse) {
            rightEncoder.direction = Encoder.Direction.REVERSE
        }
        if (HardwareNames.Encoders.LEFT.reverse) {
            leftEncoder.direction = Encoder.Direction.REVERSE
        }
        if (HardwareNames.Encoders.FRONT.reverse) {
            frontEncoder.direction = Encoder.Direction.REVERSE
        }
         */
    }
}