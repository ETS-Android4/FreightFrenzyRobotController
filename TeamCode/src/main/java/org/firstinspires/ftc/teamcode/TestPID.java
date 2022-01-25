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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Test PID", group="Iterative Opmode")
@Config
public class TestPID extends OpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rbDrive = null;
    private DcMotor lbDrive = null;
    private DcMotor rfDrive = null;
    private DcMotor lfDrive = null;

    boolean running = true;

    double actual_rb = 0;
    double actual_lb = 0;
    double actual_rf = 0;
    double actual_lf = 0;

    double target_rb = 0;
    double target_lb = 0;
    double target_rf = 0;
    double target_lf = 0;

    double last_rb = 0;
    double last_lb = 0;
    double last_rf = 0;
    double last_lf = 0;

    public static double POWER = 0;

    public static double RPM_LOOP_MS = 250;
    public static double TICKS_PER_REV = 537.6;
    public static double DRIVE_GEAR_REDUCTION = 0.5;
    public static double WHEEL_DIAMETER = 4.0;
    public static double INCH_PER_TICK = (WHEEL_DIAMETER * Math.PI) / (TICKS_PER_REV * DRIVE_GEAR_REDUCTION);
    public static double MS_TO_S = 1000;

    public static double MAX_POWER_RPS = 5.1;

    public static double Kp = 0;
    public static double Kd = 0;

    double last_loop_time = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initializing");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rbDrive  = hardwareMap.get(DcMotor.class, "right_back_drive");
        lbDrive = hardwareMap.get(DcMotor.class, "left_back_drive");
        rfDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        lfDrive = hardwareMap.get(DcMotor.class, "left_front_drive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rbDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.FORWARD);
        rfDrive.setDirection(DcMotor.Direction.REVERSE);
        lfDrive.setDirection(DcMotor.Direction.REVERSE);

        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Enable breaking on zero power
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        new Thread(() -> {
            while (running) {
                int last_rb = rbDrive.getCurrentPosition();
                int last_lb = lbDrive.getCurrentPosition();
                int last_rf = rfDrive.getCurrentPosition();
                int last_lf = lfDrive.getCurrentPosition();
                double ms = runtime.milliseconds();
                while ((runtime.milliseconds() - ms) < RPM_LOOP_MS) {}
                int delta_rb = rbDrive.getCurrentPosition() - last_rb;
                int delta_lb = lbDrive.getCurrentPosition() - last_lb;
                int delta_rf = rfDrive.getCurrentPosition() - last_rf;
                int delta_lf = lfDrive.getCurrentPosition() - last_lf;
                actual_rb = ((delta_rb / TICKS_PER_REV) / RPM_LOOP_MS) * MS_TO_S; //(delta_rb / RPM_LOOP_MS) / MS_TO_S;
                actual_lb = ((delta_lb / TICKS_PER_REV) / RPM_LOOP_MS) * MS_TO_S;
                actual_rf = ((delta_rf / TICKS_PER_REV) / RPM_LOOP_MS) * MS_TO_S;
                actual_lf = ((delta_lf / TICKS_PER_REV) / RPM_LOOP_MS) * MS_TO_S;
            }
        }).start();

//        new Thread(() -> {
//            while (running) {
//                int last = rbDrive.getCurrentPosition();
//                double ms = runtime.milliseconds();
//                while (rbDrive.getCurrentPosition() == last) {};
//                int delta = rbDrive.getCurrentPosition() - last;
//            }
//        }).start();

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
        last_loop_time = runtime.milliseconds();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

//        rbDrive.setPower(POWER);
//        lbDrive.setPower(POWER);
//        rfDrive.setPower(POWER);
//        lfDrive.setPower(POWER);

        target_rb = -POWER;
        target_lb = -POWER;
        target_rf = POWER;
        target_lf = POWER;

        double error_rb = (target_rb - actual_rb);
        double error_lb = (target_lb - actual_lb);
        double error_rf = (target_rf - actual_rf);
        double error_lf = (target_lf - actual_lf);

        double loop_time = (runtime.milliseconds() - last_loop_time);
        last_loop_time = runtime.milliseconds();
        rbDrive.setPower(((target_rb) + (error_rb * Kp) + (Kd * (error_rb - last_rb) / loop_time))/MAX_POWER_RPS);
        lbDrive.setPower(((target_lb) + (error_lb * Kp) + (Kd * (error_lb - last_lb) / loop_time))/MAX_POWER_RPS);
        rfDrive.setPower(((target_rf) + (error_rf * Kp) + (Kd * (error_rf - last_rf) / loop_time))/MAX_POWER_RPS);
        lfDrive.setPower(((target_lf) + (error_lf * Kp) + (Kd * (error_lf - last_lf) / loop_time))/MAX_POWER_RPS);

        telemetry.addData("Right Back RPS", actual_rb);
        telemetry.addData("Left Back RPS", actual_lb);
        telemetry.addData("Right Front RPS", actual_rf);
        telemetry.addData("Left Front RPS", actual_lf);
        telemetry.addData("Right Back D", (error_rb - last_rb) / loop_time);

        last_rb = error_rb;
        last_lb = error_lb;
        last_rf = error_rf;
        last_lf = error_lf;

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        running = false;
        telemetry.addData("Status", "Stopped");
    }

}
