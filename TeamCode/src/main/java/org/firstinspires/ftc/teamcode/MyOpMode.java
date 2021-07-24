package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous(name= "Odometry test", group="odometry")

public class MyOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 10, y: -8, heading: 90 degrees
       Pose2d startPose = new Pose2d(5,-8,Math.toRadians(90));//new Pose2d(5, -8, Math.toRadians(90));KL change

       drive.setPoseEstimate(startPose);
/*
        Trajectory tr= drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(10, 20), 180)
                .splineTo(new Vector2d(30, 40), 0)
                .build();
        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5)
                .build();
*/

        Trajectory traj1 = drive.trajectoryBuilder(startPose) // kl change
                .strafeRight(10)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .splineTo(new Vector2d(5, 6), 0)
                .splineTo(new Vector2d(9, -10), 0)
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);

      /*  Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

       Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(20, 9), Math.toRadians(45))
                .build();

        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);   */
       // drive.followTrajectory(traj2);

/*
        Pose2d startPose = new Pose2d(62, -22, Math.toRadians(180));

        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeTo(new Vector2d(0, 0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(5)
                .build();

       */
    }
}
