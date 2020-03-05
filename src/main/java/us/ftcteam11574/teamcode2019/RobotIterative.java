package us.ftcteam11574.teamcode2019;


import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

public class RobotIterative {
    //The plan for this is to make a bunch of static functions that I can use when creating this function
    //
    //

    public ElapsedTime runtime = new ElapsedTime();
    boolean first_iteration = true;
    boolean angle_done  = false;
    boolean runner = false;
    boolean control = false;
    boolean button = false;

    double vx;
    double vy;
    double rot; double in_time; double start; double end; int max_dist; double max_time; double angle; double rot2;
    double in_pow;double pant_pow; double servo_pow;
    boolean max_mode = true;
    boolean orientation_mode = false;
    boolean slow_intake = false;
    boolean backwards = false;
    static double angle_error = 0.03;
    static double init_angle = .10;
    boolean ramp_up = false;
    MoveStruct m;
    RobotIterative(double vx, double vy, double rot, double rot2, double in_time, double start, double end, int max_dist, double max_time, double angle, double in_pow,double pant_pow, double servo_pow) {
        constructRunner(vx,vy,rot,rot2,in_time,start,end,max_dist,max_time,angle);
        constructControl( in_pow,pant_pow,servo_pow);
    }
    void constructRunner(double vx, double vy, double rot, double rot2, double in_time, double start, double end, int max_dist, double max_time, double angle) {
        runner = true;
        this.vx=vx;
        this.vy=vy;
        this.rot = rot;
        this.in_time = in_time;
        this.start= start;
        this.end = end;
        this.max_time = max_time;
        this.max_dist = max_dist;
        this.angle = angle;
        this.rot2 = rot2;
        runtime.reset();
    }
    void constructControl(double control0, double control1, double control2,double control3) {
        control = true;
        this.in_pow = control1-control0;
        this.pant_pow = control2;
        this.servo_pow = control3;
    }
    void constructControl(double in_pow,double pant_pow, double servo_pow) {
        control = true;
        this.in_pow = in_pow;
        this.pant_pow = pant_pow;
        this.servo_pow = servo_pow;
    }
    void setModes(boolean max_mode,boolean orientation_mode, boolean slow_intake) {
        this.max_mode = max_mode;
        this.orientation_mode = orientation_mode;
        this.slow_intake = slow_intake;
    }
    RobotIterative(String line, String item_seperator,String line_seperator) {

        line = line.replace(line_seperator,"");
        String[] tokens = line.split(item_seperator); //not splitting correctly

        String first = tokens[0]; //coudl crash here, if there are no items
        if(first.equals("M") ) {

            m = new MoveStruct(line,item_seperator,line_seperator);
            constructRunner(m.vx,m.vy,m.rot,m.rot2,500,0,1,(int)(m.distance),m.distance*2 + 3500,m.angle); //minimum of 500 seconds to do task
            constructControl(m.control[0],m.control[1],m.control[2],m.control[3]); //I don't know if I want to make control work like this, maybe save control in terms of time
            //idk, I can figure it out later
            //
        }
        else if(first.equals( "B") ) {
            button = true;
            boolean[] modes = new boolean[tokens.length-1];
            for (int i = 1; tokens.length > i; i++) {
                modes[i-1] = tokens[i].equals("T")?true:false;
            }
            max_mode = modes[0];
            orientation_mode = modes[1];
            slow_intake = modes[2];
            //update the value of the mdoes
            //orientation_mode =
            //idk, I can figure it out later
            //
        }
        else {
            throw new RuntimeException("Wrong first token   :" + tokens[0] + "   full is::" + line + "second" + tokens[1] + "third" + tokens[2]);
        }

    }
    public void start() {
        if (runner) {
            Robot.reset_pow(); //just testing htis
            Robot.update_position(); //probably want this
            Robot.resetTime();

        }
        if (button) {
            //do nothing
        }
    }
    //use update, not run
    public void update() {
        if(first_iteration) {
            start();
            runtime.reset(); //reset runtime now that it has strated
            first_iteration = false;
            angle_done = (Math.abs(angDiff()) < init_angle);
        }
        run(); //do both on same iteration, so it doesn't look like this is done

        //user should check done themselves
    }
    public void run() {
        if(runner) { //default mode for runner
            if(!angle_done) {
                angle_done = (Math.abs(angDiff()) < angle_error);
                if(angle_done) {
                    Robot.update_position(); //reset the movement that was caused by turning
                    Robot.reset_pow();
                    runtime.reset(); //reset after done with this
                }
            }
            if(!angle_done) {
                Robot.turnIterative(angle);
            }
            else if (backwards) {
                if(max_mode) {
                    moveMaxIterativeRamp(vx, -vy, rot, in_time, start, end);
                }
                else {
                    moveMaxIterativeRamp(vx,-vy,rot,in_time,start,end);
                }
            }
            else {
                if(max_mode) {
                    moveMaxIterativeRamp(-vx, vy, -rot, in_time, start, end);
                }
                else {
                    moveIterativeRamp(-vx,vy,-rot,in_time,start,end);
                }
            }
        }
        if(button) {
            //do nothing, this is just here to update the modes
        }
    }
    public boolean done() {
        if(runner) {
            return ( ((pastDist(max_dist)||timeDone(max_time)) && !first_iteration) && angle_done);
            //return ( (pastDist(max_dist)) && !first_iteration);
        }
        if (button) {
            return true;
        }
        return false;
    }
    public boolean[] return_mode() {
        return new boolean[]{max_mode,orientation_mode,slow_intake};
    }
    public void set_mode(boolean[] mode) {
        if (!button) { //button shouldn't update the modes
            max_mode = mode[0];
            orientation_mode = mode[1];
            slow_intake = mode[2];
        }

    }
    public double angDiff() {
        double cur_ang = Robot.angle();
        double diff = Robot.distanceTurn(cur_ang,angle); //negate angle to make it work?
        return diff;
        //maybe do somethign like a turn orient button, that orients into current angle

    }

    //TODO: implement support for control, such as contorl of patagraph at the same time
    //TODO: I probably want the start and end speeds to match up, coudl see if there nay way to reach that, maybe read based on what the intedned speeds are
        //Note: That might cause some probablems with goign between negative and positive speeds, maybe need an abolsute value int here somewhere, not sure
    public void moveMaxIterativeRamp(double vx, double vy, double rot, double in_time, double start, double end) {
        //move in a direction at max speed iteratively, ie. do one step
        //Return true if done, false if not
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        double len = 0;
        if(ramp_up) {
            len = motorPower.rampUp(runtime.milliseconds(), in_time, start, end); //is this the problem
        }
        else {
            len = 1;
        }
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.setMotorsMax( powers, len);
            Robot.intake(in_pow);
            Robot.pantagraph.setPower(pant_pow);
            Robot.runServo(servo_pow);
            //run all of these things
            //set power of pantagraph and
        }
    }
    //TODO: need to hold angle steady, angle gets off pretty quickly
    public void moveIterativeRamp(double vx, double vy, double rot, double in_time, double start, double end) {
        //Robot.updatePositions coudl potentially be useful
        //ocudl get a function from Robot that cacluates motor power, from either ramp up or not using ramp up
        //then just check whether done
        //actually, move iterative only needs to update motor power, nothing else
        //since it actually relies on done to tell if its done, not on motor power
        double[] powers = motorPower.calcMotorsFull(vx, vy, rot);
        double len = 0;
        if(ramp_up) {
            len = motorPower.rampUp(runtime.milliseconds(), in_time, start, end); //is this the problem
        }
        else {
            len = 1;
        }
        for (int i = 0; Robot.motors.length > i; i++) {
            Robot.setMotors( powers, len);
            Robot.intake(in_pow);
            Robot.pantagraph.setPower(pant_pow);
            Robot.runServo(servo_pow);
        }
        //move iterative naturally goes well with past dist, but the user doesn't hav eto combine those two fucntions
        //

    }
    public static boolean pastDist(int max_dist) {
        double dist = Robot.motorsDist();
        if (dist >= max_dist) {
            return true;
        }
        return false;
    }
    public String dataToJavaFile() {
        //just does this part, will need another thing that creates the header and stuff

        String res = "{\n"; //just to keep the scope seperate
        res += StringConstructor();
        res += StringRun();
        res += "}\n"; //keep scope seperate
        return res;
    }
    public String StringConstructor() {
        String res = "RobotIterative robot_move = new RobotIterative(";
        res += vx + ",";
        res += vy + ",";
        res += rot + ",";
        res += rot2 + ",";
        res += in_time + ",";
        res += start + ",";
        res += end + ",";
        res += max_dist + ",";
        res += max_time + ",";
        res += angle + ",";
        res += in_pow + ",";
        res += pant_pow+ ",";
        res += servo_pow;
        res += ");\n";
        return res;
        //write the constructor for this object
    }
    public String StringRun() {
        //writes the code for making this run in a linear op mode
        String res = "robot_move.start();\n";
        res += "robot_move.set_mode(mode_buttons);\n"; //update mode buttons
        res += "while(!robot_move.done()) {\n";
        res += "\trobot_move.update();\n}\n";
        res += "mode_buttons = robot_move.return_mode();\n";
        return res;
        //then we can go on to the next section,
        //TODO: Dont forget to include teh setup for intilizazing mode buttons at the sstart of the entire program
        //TODO: could test this out by putting this code into sublime text and just reading it with a couple of test lines
    }
    public boolean timeDone(double max_time) {
        return (runtime.milliseconds() >= max_time);
    }



    //double vx, double vy, double rot, double rot2, double in_time, double start, double end, int max_dist, double max_time, double angle







}
