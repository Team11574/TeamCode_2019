package us.ftcteam11574.teamcode2019;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Blue Block Foundation", group="Autonomous")
public class BlueBlockFoundation extends LinearOpMode {




    //NOTES:
        //need to fix the init mode on the Robot, it still doesn't have the webcam connected
        //call center() to get the center from the camera
        //will want to create a trunacate mode, since we probably won't want to look along the entire y range
    final int line_buffer = 1000;

    final int wall_buffer = 1900;//distance from wall after you back up, applies to all times after
        //was previously 500 when it worked
    //you have grabbed the black, hopefully will allow the robot to avoid the other robot
    final int line_dist_from_start = 1000; //distance from start to the line under the skybridge
    final int block_distance = 1700; //Distance from the middle block to either the rigth or left block
    final int block_distance_x = 780;
    final int move_from_wall = 100; //Distance to move foward orm the wall to not be at risk to get caught on the wall
    final int block_forward_dist = 1600; //Distance to move foward while grabbing the block
    final int foundation_dist = 4650;
    final int foundation_forward =2350-wall_buffer;
    final int foundation_side = 200;
    final int foundation_buff = 550; //buffer for moving it outwards
    final int foundation_back = 450;
    final int foundation_pull = 2300;




    final int[][] ranges = { {0,640/3}, {640/3,(2*640)/3}, {(2*640)/3,640}}; //just some test values, will need to see what it looks like to determine these values

    int[] confidences = new int[3]; //times we have foudn each one to be true
    int most_recent_position = 0; //default is zero
    @Override
    public void runOpMode() {
        try {
            runOpMode2();
        }
        catch (Throwable t) {
            if (t instanceof StopException) {
                Robot.reset_pow();
                return;

            }
            throw t;


        }
    }

    public void runOpMode2() {
        //Intended start: TBD
        Robot.init_autoOp(telemetry, hardwareMap,gamepad1,gamepad2);

        Robot r =new Robot();
        Robot.AUTO auto = r.new AUTO(this);
        Robot.pistonHome();


        while(!isStarted()) { //needs a godo way to exit out of the loop
            //read the camera, and store the position, increase the confidence rating based ont eh consitancy of readings
            auto.recognize_block();



        }
        auto.start();
        most_recent_position = auto.most_recent_position; //because this isn't going to have its most_recent_position updated, only auto will, since it it using the camera
        telemetry.addData("pos guess",most_recent_position);
        telemetry.update();

        auto.grabBlockFastColor(100,1700,1600); //grab block quickly
        placeFoundation();
        moveFoundation();
        //woudl need to test the last part

    }
    public void recognize_block() {
        long start = System.currentTimeMillis();
        int[] center = Robot.center();
        int pos = -1;
        telemetry.addData("pos x",center[0]);
        telemetry.addData("pos y",center[1]);
        telemetry.addData("pos:",most_recent_position);
        for (int i = 0; ranges.length > i; i++) {

            telemetry.addData("range:" +i, "low" + ranges[i][0] + " hi:" + ranges[i][1]);
            if (center[0] >= ranges[i][0] && center[0] <= ranges[i][1]) {
                pos = i;
                telemetry.addData("new position!","new!");
            }

        }
        if(pos != -1) {
            most_recent_position = pos;

            //confiedences not implemented yet
            //will need to read motion sensors or somethingd
            //probably don't program it unless its not working well or something
        }
        telemetry.addData("time taken to calculate (millis)",System.currentTimeMillis()-start);
        telemetry.update();
    }
    public void grabBlock() { //grabs a block, if it is in the center and has already recognized the block
        //grabs a block
        //extra time to move the pantagraph down
        moveDirMax(0,-1,0,move_from_wall,500,-1,0);
        if(most_recent_position == 2) { //then to the right
            moveDirMax(-1,0,0,(int) (block_distance_x/Math.sqrt(2)),2000,-1,0);
            moveDirMax(0,-1,0,(int) (block_distance/Math.sqrt(2)),3000,-1,0); //these don't work
            //maybe coudl slightly improve this, but not by much, sine its limited by the speed of the pantagraph

        }
        if(most_recent_position == 0) {
            moveDirMax(1,0,0,(int) (block_distance_x/Math.sqrt(2)),2000,-1,0);
            moveDirMax(0,-1,0,(int) (block_distance/Math.sqrt(2)),3000,-1,0);
            //moveOrient(1,-1,0,0,block_distance,1000); //these don't work

        }
        if (most_recent_position == 1) {
            //the distance will have to be shorter here
            //need to divide by sqrt(2), since this is a 45,45,90 triangle, and we want ot move
            //and equal amount in terms of y
            moveDirMax(0, -1, 0, (int) (block_distance / Math.sqrt(2)), 1000,-1,0);
        }


        {

            Robot.resetTime();
            double[] powers = motorPower.calcMotorsFull(0, -.6, 0);
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setTargetPosition((int) (Math.abs(powers[i])/powers[i]*block_forward_dist)); //can amke this faster
            }
            Robot.setMotors(powers, 1);
            ElapsedTime pantTime = new ElapsedTime();
            while (!checkDone(3000)) { //need to check if some are done
                Robot.setMotors(powers, 1);
                Robot.pantagraphDown(pantTime);
                Robot.intake(.85); //maybe .7 will be more relaible
            }
            Robot.reset_pow();
        }
        //now we have intaked the block, so lets quickly move back
        //notee: still have ot accoutn for the difference in x position caused by goign for different bloccks
        moveDirMax(0,1,0,(int) (block_forward_dist+(block_distance/Math.sqrt(2))-wall_buffer),4000);
    }
    public void placeFoundation() {
        //words if already grabbed block
        turnOrient(0,1,2050,5500);//doesnt need to be that long
        //turnOrient(0,1,2500,5500); //possibly faster
        //I could gain a lot of time if I was turning while moving I believe
        if (most_recent_position ==2) {
            moveDirMax(0,-1,0,(int) (line_dist_from_start+ (block_distance_x/Math.sqrt(2))),2000);

        }

        if (most_recent_position == 0) {
            moveDirMax(0,-1,0,(int) (line_dist_from_start- (block_distance_x/Math.sqrt(2)) ),2000);
        }
        else {
            moveDirMax(0,-1,0, (line_dist_from_start ),2000);
        }
        //now we are at the line, now we want to be able ot place the block, should be pointed
        turnOrient(0,1,500,3500); //turn to ensure direction
        moveDirMax(0,-1,0,foundation_dist,5000,1,0);
        turnOrient(-1,0,1500,3500);
        //turnOrient(-1,0,1500,3500); //could probably save time here
        //move back a bit to reposition
        //moveDir(0,.8,0,600,1500); //repositioning phase //skip this for now
        moveDirMax(0,-1,0,foundation_forward-300,3000); //move towards foundation
        //moveDirMax(0,-1,0,foundation_buff,3000,0,-1);
        moveDir(0,-.5,0,500,1000); // a little bit more on the slow side, to ensur eit doesn't push it too much
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1000  ) {


                Robot.outtake(.9);
                //if(pantTime.milliseconds() < 500) {
                Robot.runServoDown(); //is this the right amount of time?
                //}
            }
            Robot.resetTime();
            Robot.reset_pow();
        }
    }
    public void moveFoundation() {
        //words if already at foundation
        moveDirMax(0,1,0,foundation_back,3000);

        turnOrient(1,0,2500,4500);
        //moveDirMax(1,0,0,foundation_side,3000); //not sure if this is in the right direction
        //moveDirMax(1,0,0,2000,3000); //2000?

        moveDir(0,.5,0,foundation_back+foundation_buff,3000);
        { //outtake for a second

            Robot.resetTime();

            ElapsedTime pantTime = new ElapsedTime();
            while (pantTime.milliseconds() < 1500  ) { //one second less, since running it earlier
                Robot.runServoDown();
            }
            Robot.reset_pow();
        }
        //should it go max speed, or should it like ramp up, or something like that?
        moveDirMax(0,-1,0,foundation_pull,3000);
        turnOrient(0,-1,3000,3000);
    }


    public void sleep(int maxtime) {
        ElapsedTime time = new ElapsedTime();
        while(time.milliseconds() < maxtime) {
            //nothing
        }

    }
    public void foundation_grabber(int maxtime, double power, boolean down) { //no limit switch yet
        ElapsedTime time = new ElapsedTime();
        Robot.power_foundation = .9;
        while(time.milliseconds() < maxtime) {
            if (down) {
                Robot.runServoDown();
            }
            else {
                Robot.runServoUp();
            }
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

        Robot.resetTime();
        Robot.reset_pow();

        double rot = -gamepady;
        double rot2 = gamepadx;


        {
            double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
            double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
            double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
            double goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
            double nrot = 0;


            if (Math.abs(goal_rot_ang) > 0.20) { //amount goal angle needs to be off to actually turn in that direction

                nrot = goal_rot_ang * (mag_rot); //rotation is the direction of rotation
                //test value of /2.0 just ot
                //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
            } else {

                if (goal_rot_ang != 0) {
                    nrot = (Math.abs(goal_rot_ang) / goal_rot_ang) * (mag_rot) * (Math.abs(goal_rot_ang) + .25);
                }
                // nrot = nrot>0?.1 + nrot:-.1+nrot;
            }

            Robot.resetTime();

            double[] powers = motorPower.calcMotorsFull(0, 0, nrot);

            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //Robot.motors[i].setTargetPosition((int) ((Math.abs(powers[i]) / powers[i]) * max_dist)); //can amke this faster
                //maybe rmeove the get arget position and see what happens, it if doesn't crash, then we just run for time
            }
            Robot.setMotors(0, 0, nrot, false);
        }
        double goal_rot_ang;
        {
            double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
            double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
            //double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
            goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
        }
        boolean change_sign  = false;
        boolean  prev_positive = (goal_rot_ang > 0);
        while( (! (Robot.timeElapsed() > max_time) && !change_sign) || Math.abs(goal_rot_ang) > .1) { //not sure if .05 is the rigth value, will have to test it a little

            double imu_ang = (Robot.imu.getAngularOrientation().firstAngle + 0 * Math.PI / 2.0) % Math.PI;
            double ang_rot = Math.atan2(rot, rot2) - Math.PI / 2.0; //angle your game stick is facing
            double mag_rot = Math.sqrt(rot * rot + rot2 * rot2);
            goal_rot_ang = Robot.distanceTurn(-imu_ang, ang_rot); //
            double nrot = 0;

            if (prev_positive && goal_rot_ang < 0) {
                change_sign = true;
            }
            if (!prev_positive && goal_rot_ang > 0) {
                change_sign = true;
            }


            if (Math.abs(goal_rot_ang) > 0.20) { //amount goal angle needs to be off to actually turn in that direction

                nrot = goal_rot_ang * (mag_rot ); //rotation is the direction of rotation
                //test value of /2.0 just ot
                //maybe do if cur_ang + ang_turn > goal_ang then turn only goal_ang-cur_ang
            } else {

                //nrot = goal_rot_ang * (mag_rot) * (Math.abs(goal_rot_ang) + .4); //rotation is the direction of rotation
                if (goal_rot_ang != 0) {
                    nrot = (Math.abs(goal_rot_ang) / goal_rot_ang) * (mag_rot) * (Math.abs(goal_rot_ang) + .25);
                }
                // nrot = nrot>0?.1 + nrot:-.1+nrot;
            }
            telemetry.addData("ang off",goal_rot_ang);
            //telemetry.update();

            telemetry.addData("time",max_time);
            telemetry.update();


            double[] powers = motorPower.calcMotorsFull(0, 0, nrot);
            /*
            for (int i = 0; Robot.motors.length > i; i++) {
                Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                //Robot.motors[i].setTargetPosition((int) ((Math.abs(powers[i]) / powers[i]) * max_dist)); //can amke this faster
                //ignore target position for now?
                //
            }
             */
            Robot.setMotors(0, 0, nrot, Math.abs(goal_rot_ang) > 0.20);

        }
        Robot.reset_pow();
        Robot.resetTime();//reset these things at the end

        sleep(150);


        //maybe want to turn in place, because it seems time might start ot b ea problem


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
        sleep(100); //lets get rid of this sleep for now

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

        moveDirMax(vx,vy,rot,dist,max_time,0,0);
    }
    public void moveDirMax(double vx, double vy, double rot,int dist, int max_time,double pant_pow,double in_pow) {


        Robot.resetTime();
        Robot.reset_pow();
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Robot.motors[i].setTargetPosition((int) ( (Math.abs(powers[i])/powers[i]) * dist)); //can amke this faster

        }
        Robot.setMotorsMax(powers, 1);
        while (!checkDone(max_time)) { //need to check if some are done
            Robot.setMotorsMax(powers, 1);
            Robot.pantagraph.setPower(pant_pow);
            Robot.intakeR.setPower(in_pow);
            Robot.intakeL.setPower(in_pow);
        }
        Robot.reset_pow();
        sleep(100);
        Robot.resetTime();
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
            Robot.reset_pow();
            throw new StopException();
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
    public class StopException extends RuntimeException {
        public StopException() {super();}
    }





}
