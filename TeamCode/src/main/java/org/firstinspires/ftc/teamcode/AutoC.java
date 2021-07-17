package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name= "XXX Techies Autonomous C", group="Pushbot")
public class AutoC extends TechiesAutonomous {

    protected void  DropGoal(){
        encoderDrive(DRIVE_SPEED, -154, -154, -154, -154,  5.0);
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10,  5.0);
        encoderDrive(DRIVE_SPEED, -23, 23, 23, -23,  5.0);
        encoderDrive(DRIVE_SPEED, -28, -28, -28, -28,  5.0);
        encoderDrive(DRIVE_SPEED, 27, -27, -27, 27,  5.0);
    }

    protected void driveToShooter() {
        encoderDrive(DRIVE_SPEED, -17, 17, 17, -17,  5.0);
        encoderDrive(DRIVE_SPEED, 78, 78, 78, 78,  5.0);
        sleep(1000);
    }
 //   public void pickupring(){

    //}
    protected void driveToLine() {

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
    }


}
