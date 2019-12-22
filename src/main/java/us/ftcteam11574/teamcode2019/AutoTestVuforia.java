package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutoModeTESTVU (DONT USE)", group="Autonomous")
public class AutoTestVuforia extends LinearOpMode {




    //NOTES:
        //need to fix the init mode on the Robot, it still doesn't have the webcam connected
        //call center() to get the center from the camera
        //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int line_buffer = 1000;
    final int line_dist_from_start = 3500; //distance from start to the line under the skybridge
    final int block_distance = 1500; //Distance from the middle block to either the rigth or left block
    final int move_from_wall = 100; //Distance to move foward orm the wall to not be at risk to get caught on the wall
    final int block_forward_dist = 2000; //Distance to move foward while grabbing the block
    final int[][] ranges = { {0,640/3}, {640/3,640/2}, {640/2,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() throws InterruptedException {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);

        while(!isStarted()) {
            //read the camera, and store the position, increase the confidence rating based ont eh consitancy of readings
            int[] center = Robot.center();
            int pos = -1;
            for (int i = 0; ranges.length > i; i++) {
                if (center[0] > ranges[i][0] && center[0] < ranges[i][1]) {
                    pos = i;
                }
            }
           if(pos != -1) {
               most_recent_position = pos;
               //confiedences not implemented yet
               //will need to read motion sensors or somethingd
               //probably don't program it unless its not working well or something
           }


        }





        //forward rotation is 1,0


        moveDirMax(0,-1,0,move_from_wall,500);

        if(most_recent_position != 1) { //if its 1, then it was in the middle
            if(most_recent_position ==2) { //then to the right
                //Move Diagonally, sicne it should be faster, make sure orient keep this at the right angle
                //
                moveOrient(-1,-1,0,0,block_distance,1000); //these don't work
            }
            else {
                moveOrient(1,-1,0,0,block_distance,1000); //these don't work
            }
        }
        else {
            //the distance will have to be shorter here
            //need to divide by sqrt(2), since this is a 45,45,90 triangle, and we want ot move
            //and equal amount in terms of y
            moveOrient(0,-1,0,0,(int) (block_distance/Math.sqrt(2)),1000);
        }
        //moveOrient(0,0,-1,0,5000,2000); //not sure the correct distance, since this is based on the error we end up having
        //need a turn orient mode, where it turns itself given a certain amoutn of time, and checks if its within a certain distance


        //reorient the robot

        moveOrient(0,-.7,0,0,block_forward_dist,3000,-1,1);

        //move based on position
        if (most_recent_position ==1) {
            moveOrient(0, 1, 0, 0, block_forward_dist, 3000);
        }
        else if(most_recent_position==0) {
            //this will have to be a somewhat complex calculation
            double change_x = -block_distance/Math.sqrt(2);
            double change_y = block_forward_dist;
            double dist = Math.sqrt(change_x*change_x + change_y*change_y);
            moveOrient(change_x/dist,change_y/dist,0,0,(int)dist,4000);
            //
        }
        else if(most_recent_position==2) {
            //this will have to be a somewhat complex calculation
            double change_x = block_distance/Math.sqrt(2); //positive on this one
            double change_y = block_forward_dist;
            double dist = Math.sqrt(change_x*change_x + change_y*change_y);
            moveOrient(change_x/dist,change_y/dist,0,0,(int)dist,4000);
            //
        }
        //now they should all be on the exact same spot
        //Shoudl have skystone, and should be at move_from_wall distance from start foward
        //Hopefully, should just take aroudn 5-6 seconds

        //now, the next step is to drop off the block

        moveOrient(-1,0,0,0,line_dist_from_start+line_buffer,6000); //move to the line

        moveOrient(1,0,0,0,line_buffer,4000,1,-1); //out take block while I move

        //should end on a line

        //now we should be at the line
            //NOTE:
            //we will want to move the panagraph up during this time
                //actually, not for now, since it seems ot drop the block pretty often, so we will move the pantagraph up on the way down
        //


        







        //maybe make this equal to teh game pad stuff, so we just mimic what woudl happen with game pad

    }


    public void sleep(int maxtime) {
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < maxtime) {
            //nothing
        }

    }

    public void moveOrientUpdate(double vx, double vy, double rot,int rot2, int dist, int max_time) {
        //In this mdoe, were going to try to update the motors while were moving
        //this will require that we update the position in a smart way

        Robot.resetTime();
        double[] powers = Robot.orientMode(vx, vy, rot,rot2);
        double[] powers_orig = Robot.orientMode(vx, vy, rot,rot2);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotors(powers_orig, 1);
        //Right now it will stop if more than 0 motors are finsihed
        while (!checkDone(max_time) || Robot.motorsFinished() > 0) { //need to check if some are done
            //powers = Robot.orientMode(vx, vy, rot,rot2);

            boolean can_change = true; //NOTE: SET TO FALSE FOR NOW
            //boolean can_change = true;
            for (int i = 0; powers.length > i; i++) {
                if ( (powers[i] == 0 || powers_orig[i] == 0) || (Math.abs(powers[i])/powers[i] != Math.abs(powers_orig[i])/powers_orig[i])  ) {
                    can_change = false;
                }
            }
            if(can_change) {
                Robot.setMotors(powers, 1); //This probably won't work properly
                /*
                for (int i = 0; Robot.motors.length > i; i++) {
                    Robot.motors[i].setTargetPosition(Robot.motors[i].getCurrentPosition() + );
                }
                 */
                //need to change the target direction probably
            }


            //Robot.setMotors(powers_orig, 1);
            //Robot.pantagraph.setPower(pant_pow);
            //Robot.intakeR.setPower(in_pow);
            //Robot.intakeL.setPower(in_pow);

        }
        Robot.reset_pow();

    }
    public void turnOrient(double gamepady, double gamepadx, int max_time, int max_dist) {


        double rot = -Robot.gamepad1.right_stick_y;
        double rot2 = Robot.gamepad1.right_stick_x;


        {
            double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
            double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
            double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
            double goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
            double nrot = 0;


            if (Math.abs(goal_rot_ang) > 0.40) { //amount goal angle needs to be off to actually turn in that direction

                nrot = goal_rot_ang * (mag_rot / 2.0); //rotation is the direction of rotation
                //test value of /2.0 just ot
                //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
            } else {

                nrot = goal_rot_ang * (mag_rot) * (Math.abs(goal_rot_ang) + .1); //rotation is the direction of rotation
                // nrot = nrot>0?.1 + nrot:-.1+nrot;
            }

            Robot.resetTime();

            double[] powers = motorPower.calcMotorsFull(0, 0, nrot);

            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.motors[i].setTargetPosition((int) ((Math.abs(powers[i]) / powers[i]) * max_dist)); //can amke this faster

            }
            Robot.setMotors(0, 0, nrot, true);
        }
        double goal_rot_ang = 100;
        while(!checkDone(max_time) && Math.abs(goal_rot_ang) > .05) { //not sure if .05 is the rigth value, will have to test it a little

            double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
            double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
            double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
            goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
            double nrot = 0;


            if (Math.abs(goal_rot_ang) > 0.40) { //amount goal angle needs to be off to actually turn in that direction

                nrot = goal_rot_ang * (mag_rot / 2.0); //rotation is the direction of rotation
                //test value of /2.0 just ot
                //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
            } else {

                nrot = goal_rot_ang * (mag_rot) * (Math.abs(goal_rot_ang) + .1); //rotation is the direction of rotation
                // nrot = nrot>0?.1 + nrot:-.1+nrot;
            }

            Robot.resetTime();

            double[] powers = motorPower.calcMotorsFull(0, 0, nrot);

            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                Robot.motors[i].setTargetPosition((int) ((Math.abs(powers[i]) / powers[i]) * max_dist)); //can amke this faster

            }
            Robot.setMotors(0, 0, nrot, true);

        }




    }


    //NOTE: NOT ABLE TO CHANGE WHILE MOVING, HOPE that will help fix this
    public void moveOrient(double vx, double vy, double rot,int rot2, int dist, int max_time,double pant_pow,double in_pow) { //this probalby needs some work


        Robot.resetTime();
        double[] powers = Robot.orientMode(vx, vy, rot,rot2);
        double[] powers_orig = Robot.orientMode(vx, vy, rot,rot2);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotors(powers_orig, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            //powers = Robot.orientMode(vx, vy, rot,rot2);
            /*
            boolean can_change = false; //NOTE: SET TO FALSE FOR NOW
            //boolean can_change = true;
            for (int i = 0; powers.length > i; i++) {
                if ( (powers[i] == 0 || powers_orig[i] == 0) || (Math.abs(powers[i])/powers[i] != Math.abs(powers_orig[i])/powers_orig[i])  ) {
                    can_change = false;
                }
            }
            if(can_change) {
                Robot.setMotors(powers, 1); //This probably don't work properly
                //need to change the target direction probably
            }

             */
            Robot.setMotors(powers_orig, 1);
            Robot.pantagraph.setPower(pant_pow);
            Robot.intakeR.setPower(in_pow);
            Robot.intakeL.setPower(in_pow);

        }
        Robot.reset_pow();
        //sleep(100); //lets get rid of this sleep for now

    }
    public void moveOrient(double vx, double vy, double rot,int rot2, int dist, int max_time) { //this probalby needs some work


       moveOrient(vx,vy,rot,rot2,dist,max_time,0,0);

    }
    public void moveDir(double vx, double vy, double rot,int dist, int max_time) {


        Robot.resetTime();
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotors(powers, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            Robot.setMotors(powers, 1);
        }
        Robot.reset_pow();
        sleep(100);

    }
    public void moveDirMax(double vx, double vy, double rot,int dist, int max_time) {


        Robot.resetTime();
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotorsMax(powers, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            Robot.setMotorsMax(powers, 1);
        }
        Robot.reset_pow();
        sleep(100);
    }
    public void initMode0() {

        //nothign for now
        Robot.resetTime();
        Robot.moveVx(.5,1,10000,true);


    }
    public void initMode1() {
        //nothign for now
        Robot.resetTime();
        Robot.moveVx(-.3,.7,10000,true);


    }

    public boolean checkDone(int maxTime) { //true if done
        //return true when done
        if (Robot.timeElapsed() > maxTime ) { //max amount of time alloted
            telemetry.addData("out of time","time ran out");
            return true;
        }
        if(isStopRequested()) {
            throw new RuntimeException();
            //should force it to stop
            //return true;
        }
        if(Robot.motorsFinished() >= 4 && Robot.timeElapsed() > 150) { //150 milli before it will end
            return true; //expiermental code here
        }
       /*
       if(Robot.motorsFinished() >= 2) {
           return true;
           //then done
       }

        */


        return false;
    }




}
