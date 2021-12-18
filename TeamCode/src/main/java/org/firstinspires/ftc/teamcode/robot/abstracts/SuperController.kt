package org.firstinspires.ftc.teamcode.robot.abstracts

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.hardware.Gamepad

/**
 * A better controller with extra custom features
 */
@Suppress("unused")
class SuperController(
    /**
     * Internal gamepad
     */
    @JvmField
    val gamepad: Gamepad
){
    /**
     * Update all triggerables
     */
    fun update() {
        leftBumper.update()
        rightBumper.update()
        a.update()
        b.update()
        x.update()
        y.update()
        dpadUp.update()
        dpadDown.update()
        dpadLeft.update()
        dpadRight.update()
        start.update()
        back.update()
        leftStickButton.update()
        rightStickButton.update()
        guide.update()
        share.update()
        options.update()
        touchpad.update()
        cross.update()
        circle.update()
        square.update()
        triangle.update()
        ps.update()
        leftTrigger.update()
        rightTrigger.update()
        leftStickX.update()
        leftStickY.update()
        rightStickX.update()
        rightStickY.update()
    }

    /**
     * Minimum dead zone threshold
     */
    private val deadZoneMin = 0.2F

    /**
     * left analog stick horizontal axis
     */
    @JvmField
    val leftStickX = Triggerables.ControllerStick { gamepad.left_stick_x }
    /**
     * left analog stick vertical axis
     */
    @JvmField
    val leftStickY = Triggerables.ControllerStick { gamepad.left_stick_y }
    /**
     * right analog stick horizontal axis
     */
    @JvmField
    val rightStickX = Triggerables.ControllerStick { gamepad.right_stick_x }
    /**
     * right analog stick vertical axis
     */
    @JvmField
    val rightStickY = Triggerables.ControllerStick { gamepad.right_stick_y }

    /**
     * dpad up
     */
    @JvmField
    val dpadUp = Triggerables.ControllerButton { gamepad.dpad_up }

    /**
     * dpad down
     */
    @JvmField
    val dpadDown = Triggerables.ControllerButton { gamepad.dpad_down }

    /**
     * dpad left
     */
    @JvmField
    val dpadLeft = Triggerables.ControllerButton { gamepad.dpad_left }

    /**
     * dpad right
     */
    @JvmField
    val dpadRight = Triggerables.ControllerButton { gamepad.dpad_right }

    /**
     * Triggerables.Button a
     */
    @JvmField
    val a = Triggerables.ControllerButton { gamepad.a }

    /**
     * Triggerables.Button b
     */
    @JvmField
    val b = Triggerables.ControllerButton { gamepad.b }

    /**
     * Triggerables.Button x
     */
    @JvmField
    val x = Triggerables.ControllerButton { gamepad.x }

    /**
     * Triggerables.Button y
     */
    @JvmField
    val y = Triggerables.ControllerButton { gamepad.y }

    /**
     * Triggerables.Button guide - often the large Triggerables.Button in the middle of the controller. The OS may capture this Triggerables.Button before it is sent to the app; in which case you'll never receive it.
     */
    @JvmField
    val guide = Triggerables.ControllerButton { gamepad.guide }

    /**
     * Triggerables.Button start
     */
    @JvmField
    val start = Triggerables.ControllerButton { gamepad.start }

    /**
     * Triggerables.Button back
     */
    @JvmField
    val back = Triggerables.ControllerButton { gamepad.back }

    /**
     * Triggerables.Button left bumper
     */
    @JvmField
    val leftBumper = Triggerables.ControllerButton { gamepad.left_bumper }

    /**
     * Triggerables.Button right bumper
     */
    @JvmField
    val rightBumper = Triggerables.ControllerButton { gamepad.right_bumper }

    /**
     * left stick Triggerables.Button
     */
    @JvmField
    val leftStickButton = Triggerables.ControllerButton { gamepad.left_stick_button }

    /**
     * right stick Triggerables.Button
     */
    @JvmField
    val rightStickButton = Triggerables.ControllerButton { gamepad.right_stick_button }

    /**
     * left trigger
     */
    @JvmField
    val leftTrigger = Triggerables.ControllerTrigger { gamepad.left_trigger }

    /**
     * right trigger
     */
    @JvmField
    val rightTrigger = Triggerables.ControllerTrigger { gamepad.right_trigger }

    /**
     * PS4 Support - Circle
     */
    @JvmField
    val circle = Triggerables.ControllerButton { gamepad.circle }

    /**
     * PS4 Support - Cross
     */
    @JvmField
    val cross = Triggerables.ControllerButton { gamepad.cross }

    /**
     * PS4 Support - Triangle
     */
    @JvmField
    val triangle = Triggerables.ControllerButton { gamepad.triangle }

    /**
     * PS4 Support - Square
     */
    @JvmField
    val square = Triggerables.ControllerButton { gamepad.square }

    /**
     * PS4 Support - Share
     */
    @JvmField
    val share = Triggerables.ControllerButton { gamepad.share }

    /**
     * PS4 Support - Options
     */
    @JvmField
    val options = Triggerables.ControllerButton { gamepad.options }

    /**
     * PS4 Support - touchpad
     */
    @JvmField
    val touchpad = Triggerables.ControllerButton { gamepad.touchpad }
    val touchpadFinger1: Vector2d?
        get() {
            return if (gamepad.touchpad_finger_1) {
                Vector2d(gamepad.touchpad_finger_1_x.toDouble(),
                    gamepad.touchpad_finger_1_y.toDouble()
                )
            } else { null }
        }
    val touchpadFinger2: Vector2d?
        get() {
            return if (gamepad.touchpad_finger_2) {
                Vector2d(gamepad.touchpad_finger_2_x.toDouble(),
                    gamepad.touchpad_finger_2_y.toDouble()
                )
            } else { null }
        }

    /**
     * PS4 Support - PS Triggerables.Button
     */
    @JvmField
    val ps = Triggerables.ControllerButton { gamepad.ps }
}