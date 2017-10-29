
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/


dance_routine = [{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0.5400000000000003,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0.5400000000000003,"shoulder_right_yaw":0,"upperarm_right_pitch":-1.1900000000000008,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0.008437500000000004,"shoulder_right_yaw":0,"upperarm_right_pitch":1.271406250000001,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0.008437500000000004,"shoulder_right_yaw":-3.0799999999999783,"upperarm_right_pitch":1.271406250000001,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0},{"clavicle_right_yaw":0,"shoulder_right_yaw":0,"upperarm_right_pitch":0,"forearm_right_yaw":0,"clavicle_left_roll":0}];


fetch_dance = [{"torso_lift_joint":0,"shoulder_pan_joint":0,"shoulder_lift_joint":0,"upperarm_roll_joint":0,"elbow_flex_joint":0,"forearm_roll_joint":0,"wrist_flex_joint":0,"wrist_roll_joint":0,"gripper_axis":0,"head_pan_joint":0,"head_tilt_joint":0,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":6.775513155383438e-10,"shoulder_lift_joint":1.8032785340009941e-9,"upperarm_roll_joint":-2.540818866566349e-9,"elbow_flex_joint":1.6298076607117563e-9,"forearm_roll_joint":-1.969905008734211e-10,"wrist_flex_joint":1.2490653552654183e-9,"wrist_roll_joint":2.4137209702525744e-13,"gripper_axis":0,"head_pan_joint":1.0643370514355754e-13,"head_tilt_joint":-0.4600000726472072,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":1.090000000001314,"shoulder_lift_joint":-0.10999999999650525,"upperarm_roll_joint":-4.9240687402861246e-12,"elbow_flex_joint":0.06000000000315855,"forearm_roll_joint":-3.817646272419907e-13,"wrist_flex_joint":2.4206698680368998e-12,"wrist_roll_joint":4.677754929242781e-16,"gripper_axis":0,"head_pan_joint":1.050000000000001,"head_tilt_joint":-0.009641473220722918,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":-1.1305301637934442,"shoulder_lift_joint":-0.1099464972282288,"upperarm_roll_joint":-4.921711294904631e-12,"elbow_flex_joint":0.059970816675005326,"forearm_roll_joint":-3.815818537461729e-13,"wrist_flex_joint":2.419510949000324e-12,"wrist_roll_joint":4.675515408972993e-16,"gripper_axis":0,"head_pan_joint":-1.0805107082426553,"head_tilt_joint":-0.009636790517301836,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":1.2099999998653497,"shoulder_lift_joint":-0.7000000000131821,"upperarm_roll_joint":0.28,"elbow_flex_joint":-0.9199999999928106,"forearm_roll_joint":-7.094755196506197e-18,"wrist_flex_joint":4.498599110491867e-17,"wrist_roll_joint":8.693190443542839e-21,"gripper_axis":0,"head_pan_joint":-1.286903393348172e-10,"head_tilt_joint":-1.6568283465570482e-8,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":1.2099742366750854,"shoulder_lift_joint":-0.699993504980718,"upperarm_roll_joint":0.27999691789120307,"elbow_flex_joint":-0.9199892129332545,"forearm_roll_joint":-1.1294952458800046e-17,"wrist_flex_joint":7.161834577354809e-17,"wrist_roll_joint":1.3839684394391e-20,"gripper_axis":0,"head_pan_joint":-0.000011893626117867605,"head_tilt_joint":-0.76,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0.8800133188294381,"shoulder_lift_joint":-0.6300077052005019,"upperarm_roll_joint":0.9100030820748726,"elbow_flex_joint":-2.1710000000000016,"forearm_roll_joint":0.23000000000000007,"wrist_flex_joint":7.883277902497227e-22,"wrist_roll_joint":1.5233817115632768e-25,"gripper_axis":0,"head_pan_joint":-1.3091347856437165e-10,"head_tilt_joint":-0.000008365341458845718,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":-0.9501548466218509,"shoulder_lift_joint":-0.8498873143748261,"upperarm_roll_joint":-0.490170814246577,"elbow_flex_joint":-2.1705851344667946,"forearm_roll_joint":0.22995604833135078,"wrist_flex_joint":7.881771453722268e-22,"wrist_roll_joint":1.5230906021362636e-25,"gripper_axis":0,"head_pan_joint":-1.3088846175640824e-10,"head_tilt_joint":-0.000008363742890515556,"torso_fixed_joint":0},{"torso_lift_joint”:0,”shoulder_pan_joint":1.2099742366750854,"shoulder_lift_joint":-0.699993504980718,"upperarm_roll_joint":0.27999691789120307,"elbow_flex_joint":-0.9199892129332545,"forearm_roll_joint":-1.1294952458800046e-17,"wrist_flex_joint":7.161834577354809e-17,"wrist_roll_joint":1.3839684394391e-20,"gripper_axis":0,"head_pan_joint":-0.000011893626117867605,"head_tilt_joint":-0.76,"torso_fixed_joint":0},{"torso_lift_joint":0,"shoulder_pan_joint":0.000055444900299771074,"shoulder_lift_joint":-0.00003209435400311687,"upperarm_roll_joint":0.000012937835705601226,"elbow_flex_joint":-0.000042385406304629,"forearm_roll_joint":3.4998665986155935e-8,"wrist_flex_joint":3.2738570545767896e-21,"wrist_roll_joint":6.326472344245072e-25,"gripper_axis":0,"head_pan_joint":-5.43687952435509e-10,"head_tilt_joint":-0.000034741536328457365,"torso_fixed_joint":0}];

function sleep(milliseconds) {
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > milliseconds){
      break;
    }
  }
}


kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

	// set setpoints (comment for original routine from "Saturday Night Lights"
	kineval.setpoints = fetch_dance; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
	for (joint in robot.joints) {
		pose_idx = kineval.params.dance_sequence_index[kineval.params.dance_pose_index];
		pose = kineval.setpoints[pose_idx][joint];
		kineval.params.setpoint_target[joint] = pose;
	}
	kineval.params.dance_pose_index = (kineval.params.dance_pose_index+1)%(kineval.params.dance_sequence_index.length);
	textbar.innerHTML = "executing dance routine, pose " + kineval.params.dance_pose_index + " of " + kineval.params.dance_sequence_index.length;

	kineval.params.update_pd = true;
	sleep(100);


}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
	for (joint in robot.joints) {
		x = robot.joints[joint].angle;
		xd = kineval.params.setpoint_target[joint];
		e = xd - x;

		kp = robot.joints[joint].servo.p_gain;
		robot.joints[joint].control = kp*e;
	}




}


