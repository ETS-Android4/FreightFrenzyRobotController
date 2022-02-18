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

package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.vision.OpenCVElementTracker;

@TeleOp(name="Debug Util", group="Iterative Opmode")
@Config
public class DebugOpMode extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot robot;

//    private VuforiaLocalizer vuforiaLocalizer;
    public static double ELEMENT_TRACKER_X_OFFSET = 0;
    private OpenCVElementTracker elementTracker;

    // FTC Dashboard Editable Variables
    public static double DRIVE_SPEED_MULTIPLIER = 0.9;
    public static double ARM_SPEED_MULTIPLIER = 0.3;
    public static double TURN_SPEED_MULTIPLIER = 0.6;
    public static double WHEEL_SPEED = 1.0;

    // Servo Positions
    public static double LEFT_CLAW_SERVO = 0.3;
    public static double RIGHT_CLAW_SERVO = 0.3;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("Status", "Initializing");

        robot = new Robot(hardwareMap);

//        vuforiaLocalizer = new VuforiaLocalizer(hardwareMap);
//        vuforiaLocalizer.initialize();
        elementTracker = new OpenCVElementTracker(hardwareMap, ELEMENT_TRACKER_X_OFFSET);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        telemetry.addData("Element Position", elementTracker.getLocation());
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        elementTracker.stop();
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double drive = gamepad1.left_stick_x;
        double strafe = gamepad1.left_stick_y;
        double rotate = gamepad1.right_stick_x;

        robot.drive(drive, strafe, rotate * TURN_SPEED_MULTIPLIER, DRIVE_SPEED_MULTIPLIER);

        // Duck wheel motor
        if (gamepad1.dpad_left) {
            robot.duckWheel.setPower(-WHEEL_SPEED);
        } else if (gamepad1.dpad_right) {
            robot.duckWheel.setPower(WHEEL_SPEED);
        } else {
            robot.duckWheel.setPower(0);
        }

        // Arm motors
        double armSpeed = (gamepad1.right_trigger - gamepad1.left_trigger) * ARM_SPEED_MULTIPLIER;
        robot.setArmSpeed(armSpeed);

        // Arm servos
        if (gamepad1.x) {
            robot.setClawPosition(new double[]{LEFT_CLAW_SERVO, RIGHT_CLAW_SERVO});
        }

        // Reset encoders for all motors
        if (gamepad1.y) {
            robot.resetEncoders();
            robot.useEncoders();
        }

//        Pose2d locationEstimate = vuforiaLocalizer.getLocationEstimate();
//        TelemetryPacket positionPacket = new TelemetryPacket();
//        positionPacket.fieldOverlay().setStroke("black").strokeRect(locationEstimate.getX(), locationEstimate.getY(), 18, 18);

        // Send time and encoder values
        telemetry.addData("Status", "Run Time: " + runtime.toString());
//        telemetry.addData("Heading", locationEstimate.getHeading());
        telemetry.addData("rbDrive", robot.rbDrive.getCurrentPosition());
        telemetry.addData("lbDrive", robot.lbDrive.getCurrentPosition());
        telemetry.addData("rfDrive", robot.rfDrive.getCurrentPosition());
        telemetry.addData("lfDrive", robot.lfDrive.getCurrentPosition());
        telemetry.addData("duckWheel", robot.duckWheel.getCurrentPosition());
        telemetry.addData("armMotorLeft", robot.armMotorLeft.getCurrentPosition());
        telemetry.addData("armMotorRight", robot.armMotorRight.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Status", "Stopped");
    }

}
