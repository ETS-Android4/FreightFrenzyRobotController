/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Full Teleop Old", group="Iterative Opmode")
@Config
@Disabled
public class FullTeleopOld extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rbDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor lfDrive = null;
    private DcMotor wheelMotor = null;
    private DcMotor armMotorLeft = null;
    private DcMotor armMotorRight = null;
    private Servo armServoLeft = null;
    private Servo armServoRight = null;

    // FTC Dashboard Editable Variables
    public static double DRIVE_SPEED_MULTIPLIER = 0.9;
    public static double ARM_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.6;
    public static double WHEEL_SPEED = 1.0;
    public static boolean LEFT_ARM_MOTOR_REVERSED = false;
    public static boolean RIGHT_ARM_MOTOR_REVERSED = false;
//    public static boolean LEFT_ARM_SERVO_REVERSED = false;
//    public static boolean RIGHT_ARM_SERVO_REVERSED = false;
//    public static boolean DRIVE_INVERTED = false;
//    public static boolean STRAFE_INVERTED = false;
//    public static boolean ROTATE_INVERTED = false;
//
//    public static double SERVO_POSITION_RIGHT = 0.0;
//    public static double SERVO_POSITION_LEFT = 0.0;

    // Servo Positions
    public static double[] CLAW_BACK = {0.05, 0};
    public static double[] CLAW_OPEN = {0.22, 0.3};
    public static double[] CLAW_GRAB = {0.48, 0.545};

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rbDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        lbDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rfDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        lfDrive = hardwareMap.get(DcMotor.class, "left_front_drive");
        wheelMotor = hardwareMap.get(DcMotor.class, "wheel_motor");
        armMotorLeft = hardwareMap.get(DcMotor.class, "arm_motor_left");
        armMotorRight = hardwareMap.get(DcMotor.class, "arm_motor_right");

        armServoLeft = hardwareMap.get(Servo.class, "arm_servo_left");
        armServoRight = hardwareMap.get(Servo.class, "arm_servo_right");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rbDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        wheelMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        armMotorRight.setDirection(DcMotor.Direction.REVERSE);

        // Enable breaking on zero power
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Servo Configuration
        armServoLeft.setDirection(Servo.Direction.REVERSE);
//        armServoRight.setDirection(Servo.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//        telemetry.addData("Status", "Running");
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double drive = gamepad1.left_stick_y; //(DRIVE_INVERTED ? -1 : 1) *
        double strafe = -gamepad1.left_stick_x; //(STRAFE_INVERTED ? -1 : 1) *
        double rotate = -gamepad1.right_stick_x * TURN_SPEED_MULTIPLIER; //(ROTATE_INVERTED ? -1 : 1) *
//        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * DRIVE_SPEED_MULTIPLIER;
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x * -1 * TURN_SPEED_MULTIPLIER;
        final double rbPower = DRIVE_SPEED_MULTIPLIER * (drive - rotate + strafe); //r * Math.cos(robotAngle) - rightX;
        final double lbPower = DRIVE_SPEED_MULTIPLIER * (drive + rotate - strafe); //r * Math.sin(robotAngle) + rightX;
        final double rfPower = DRIVE_SPEED_MULTIPLIER * (drive - rotate - strafe); //r * Math.sin(robotAngle) - rightX;
        final double lfPower = DRIVE_SPEED_MULTIPLIER * (drive + rotate + strafe); //r * Math.cos(robotAngle) + rightX;

        rbDrive.setPower(Range.clip(rbPower, -1, 1));
        lbDrive.setPower(Range.clip(lbPower, -1, 1));
        rfDrive.setPower(Range.clip(rfPower, -1, 1));
        lfDrive.setPower(Range.clip(lfPower, -1, 1));

        // Duck wheel motor
        if (gamepad1.dpad_left) {
            wheelMotor.setPower(-WHEEL_SPEED);
        } else if (gamepad1.dpad_right) {
            wheelMotor.setPower(WHEEL_SPEED);
        } else {
            wheelMotor.setPower(0);
        }

        // Arm motors
        float armSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
        armMotorLeft.setPower((LEFT_ARM_MOTOR_REVERSED ? -1 : 1) * armSpeed * ARM_SPEED_MULTIPLIER);
        armMotorRight.setPower((RIGHT_ARM_MOTOR_REVERSED ? -1 : 1) * armSpeed * ARM_SPEED_MULTIPLIER);

        // Arm servos
        if (gamepad1.a) {
            armServoLeft.setPosition(CLAW_BACK[0]);
            armServoRight.setPosition(CLAW_BACK[1]);
        } else if (gamepad1.b) {
            armServoLeft.setPosition(CLAW_OPEN[0]);
            armServoRight.setPosition(CLAW_OPEN[1]);
        } else if (gamepad1.y) {
            armServoLeft.setPosition(CLAW_GRAB[0]);
            armServoRight.setPosition(CLAW_GRAB[1]);
        }

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Servo Left Position", armServoLeft.getPosition());
        telemetry.addData("Servo Right Position", armServoRight.getPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}
