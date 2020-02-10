package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Red Foundation 2 Block (TESTING)", group="Autonomous")
public class RedFoundation2Block extends LinearOpMode {




    //NOTES:
    //need to fix the init mode on the Robot, it still doesn't have the webcam connected
    //call center() to get the center from the camera
    //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int foundation_dist = 2000;
    final int foundation_buff = 500;
    final int foundation_pull = 1100;
    final int side_dist = 900;
    final int side_dir = 1;


    final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() {
        try {
            runOpMode2();
        }
        catch (Throwable t) {
            if (t instanceof Robot.StopException) {
                Robot.reset_pow();
                return;

            }
            throw t;


        }
    }
    public void runOpMode2() {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap, gamepad1, gamepad2);
        //Robot.pistonHome(); //coudl include this?

        Robot r = new Robot();
        Robot.AUTO auto = r.new AUTO(this);
        Robot.pistonHome();
        while (!isStarted()) {

            //wait

        }
        Robot.resetTime();
        Robot.reset_pow();

        //move down for a bit longer
        //auto.moveDirMax(0,1,0,foundation_dist,3000,0,0,-1); //move servo down and move towards foundation
        auto.moveDirMax(.2,1,0,foundation_dist,3000,0,0,-1); //move servo down and move towards foundation
        //auto.moveDirMax(1,0,0,400,500,0,0,-.6);

        auto.moveDir(0,.5,0,foundation_buff,3000,0,0,-1); //move
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 500  ) { //one second less, since running it earlier
                Robot.runServoDown();
            }
            Robot.reset_pow();
        }
        auto.moveDirMax(0,-1,0,foundation_dist,3000);
        auto.turnOrient(0,-1,3000,0,0,.30);
        auto.moveDirMax(0,-1,0,foundation_pull,2000,-1,0,1);
        //auto.moveDirMax(side_dir,0,0,side_dist,1000,-1,0,1); //move to side, perfer to move towards wall side
        //coudl either grab random blocks or actually try to recognize the correct blocks
        auto.turnOrient(0,-1,600,0,0,.30); //
        auto.moveDirMax(.41,-1,0,2550,4500,-1,0,1);
        Robot.resetTime();
        auto.turnOrient(1,0,2500); //turn towards blocks
        //now we want to recognize, then move diagonally
        auto.recognize_block();



        telemetry.addData("block predicted at",auto.most_recent_position);


        //UNTESTED BELOW

        if(most_recent_position == 1) { //then to the right
            auto.moveDirMax(-.4,-1,0, (900),2000,-1,1);

            //maybe coudl slightly improve this, but not by much, sine its limited by the speed of the pantagraph

        }
        else if(most_recent_position == 2) {
            auto.moveDirMax(.4,-1,0, 1100,2000,-1,1);

            //moveOrient(1,-1,0,0,block_distance,1000); //these don't work

        }
        else if (most_recent_position == 0) {
            //the distance will have to be shorter here
            //need to divide by sqrt(2), since this is a 45,45,90 triangle, and we want ot move
            //and equal amount in terms of y
            auto.moveDirMax(-.8, -1, 0, 1200, 1000,-1,1);
        }
        auto.moveDirMax(0, -1, 0, 1200, 1000,-1,1);
        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -1, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*1200)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!auto.checkDone(1500)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake(1); //maybe .7 will be more relaible, not sure
            }
            Robot.reset_pow();
        }
        int dist_back = 2000;
        int dist_to_foundation = 3000;
        int extra_dist = 500;
        int block_distance_x = 800;
        int block_dist_contrib = most_recent_position*block_distance_x;
        auto.moveDirMax(1, -1, 0, dist_back, 2500,-1,1); //moves back, and should align with one side
        auto.turnOrient(0,-1,3000,-1,0,0); //re orient towards foundation
        auto.moveDirMax(0, -1, 0, dist_to_foundation + block_dist_contrib, 1000,-1,0); //move towards foundation
        auto.moveDirMax(0, -1, 0, extra_dist, 1000,-1,-1); //while hitting foundation, drop it off

        auto.moveDirMax(1, -1, 0, (int) (dist_to_foundation + extra_dist + dist_back/1.4 + block_dist_contrib), 1000,-1,1);
        //goes backwards towards blocks
        auto.turnOrient(1,0,2500); //turn towards blocks
        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -1, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*1200)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!auto.checkDone(1500)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake(1); //maybe .7 will be more relaible, not sure
            }
            Robot.reset_pow();
        }

        auto.moveDirMax(1, -1, 0, dist_back, 2500,-1,1); //moves back, and should align with one side



        //now grab the block
        //then move back to park





    }



}
