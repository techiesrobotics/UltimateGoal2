package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name= "XXX Techies Autonomous B", group="Pushbot")
public class AutoB extends TechiesAutonomous {

    protected void DropGoal() {
        encoderDrive(DRIVE_SPEED, -117, -117, -117 , -117,  5.0);
        encoderDrive(DRIVE_SPEED, 10, 10, 10, 10,  5.0);
        encoderDrive(DRIVE_SPEED, 20, -20, -20, 20,  5.0);
        encoderDrive(DRIVE_SPEED, -28, -28, -28, -28,  5.0);
        encoderDrive(DRIVE_SPEED, -24, 24, 24, -24,  5.0);
    }

    protected void driveToShooter() {
        encoderDrive(DRIVE_SPEED, 12, -12, -12, 12,  5.0);
        encoderDrive(DRIVE_SPEED, 49, 49, 49, 49, 5.0);
        encoderDrive(DRIVE_SPEED, -26.5, 26.5, 26.5, -26.5,  5.0);
        encoderDrive(DRIVE_SPEED, -1, 1, -1, 1,  5.0);
        sleep(1000);
    }
   // public void intakering() {
      //  encoderDrive(DRIVE_SPEED, 55, 55, 55, 55,  5.0);


  //  }
    //public void driveToShooter2() {
     //   encoderDrive(DRIVE_SPEED, -60, -60, -60, -60, 5.0);
  //  }

    protected void driveToLine() {

        encoderDrive(DRIVE_SPEED, -10, -10, -10, -10, 5.0);
    }

}
