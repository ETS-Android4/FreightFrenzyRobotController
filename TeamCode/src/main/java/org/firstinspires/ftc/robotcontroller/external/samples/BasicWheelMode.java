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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic Duckwheel", group="Iterative Opmode")
@Config
public class BasicWheelMode extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rbDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor lfDrive = null;
    private DcMotor wheelMotor = null;

    // FTC Dashboard Editable Variables
    public static double DRIVE_SPEED_MULTIPLIER = 1.0;
    public static double TURN_SPEED_MULTIPLIER = 0.5;
    public static double WHEEL_SPEED = 1.0;

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

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rbDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        lfDrive.setDirection(DcMotor.Direction.FORWARD);
        wheelMotor.setDirection(DcMotor.Direction.FORWARD);

        // Enable breaking on zero power
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y) * DRIVE_SPEED_MULTIPLIER;
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x * -1 * TURN_SPEED_MULTIPLIER;
        final double rbPower = r * Math.cos(robotAngle) - rightX;
        final double lbPower = r * Math.sin(robotAngle) + rightX;
        final double rfPower = r * Math.sin(robotAngle) - rightX;
        final double lfPower = r * Math.cos(robotAngle) + rightX;

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

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}
