/*
The MIT License (MIT)

Copyright © 2021 Checkmate Robotics

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the “Software”), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the
following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT
LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

package org.firstinspires.ftc.teamcode.robot;

/**
 * Class to manage names of hardware connected to the robot
 */
public class HardwareNames {
    /**
     * Contains hardware info for motors
     */
    public enum Motors {
        // Drivetrain
        LEFT_FRONT ("fl", true),
        LEFT_REAR ("bl", false),
        RIGHT_REAR ("br", false),
        RIGHT_FRONT ("fr", true),

        // Carousel mechanism
        CAROUSEL ("carousel", false),

        INTAKE ("intake", false),

        // Lift mechanism
        LIFT1 ("lift1", false),

        LIFT2 ("lift2", true);


        public final String name;
        public final boolean reverse;

        Motors(String name, boolean reverse){
            this.name = name;
            this.reverse = reverse;
        }
    }

    public enum Encoders {
        LEFT ("leftRear", true),
        FRONT ("leftFront", false),
        RIGHT ("rightRear", false),
        LIFT1 ("lift1", false),
        LIFT2 ("lift2", true);

        public final String name;
        public final boolean reverse;

        Encoders(String name, boolean reverse){
            this.name = name;
            this.reverse = reverse;
        }
    }

    /**
     * Contains hardware info for servos
     */
    public enum Servos {

        BUCKET ("intakeServo",-1,1, false),
        INTAKESERVO ("intakeServo",-1,1, false),
        ARM ("armServo",-1,1, false),
        CLAW ("clawServo",-1,1, false);

        public final String name;
        public final double upperLimit;
        public final double lowerLimit;
        public final boolean reversed;

        Servos(String name, double lowerLimit, double upperLimit, boolean reversed){
            this.name = name;
            this.upperLimit = upperLimit;
            this.lowerLimit = lowerLimit;
            this.reversed = reversed;
        }
    }

    /**
     * Contains hardware info for continuous rotation servos
     */
    public enum CRServos {
        DUMMY ("", false);

        public final String name;
        public final boolean reverse;

        CRServos(String name, boolean reverse) {
            this.name = name;
            this.reverse = reverse;
        }
    }

    /**
     * Contains hardware info for cameras
     */
    public enum Cameras {
        WEBCAM ("Webcam 1");

        public final String name;

        Cameras(String name) {
            this.name = name;
        }
    }
}
