package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class DuckWheelAutoRed extends LinearOpMode {

    DcMotor duckWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        duckWheel = hardwareMap.get(DcMotor.class, "wheel_motor");
        drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(90)));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(24)
                .build();

        drive.followTrajectory(traj);
        drive.turn(Math.toRadians(0));

        sleep(1000);
        duckWheel.setPower(1);
        sleep(3000);
        duckWheel.setPower(0);

//        drive.followTrajectory(
//                drive.trajectoryBuilder(drive.getPoseEstimate(), true)
//                        .forward(24)
//                        .build()
//        );
    }
}
