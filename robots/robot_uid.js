//   CREATE ROBOT STRUCTURE

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = new Object(); // or just {} will create new object

// give the robot a name
robot.name = "uid";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0.1,0], rpy:[0,0,0]};  

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "torso";  

// specify and create data objects for the links of the robot
robot.links = {"foot1": {}, "foot2": {}, "torso": {}, "arm1": {}, "arm2": {}, "head": {} };

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

// specify and create data objects for the joints of the robot
robot.joints = {};

robot.joints.head_joint = {parent:"torso", child:"head"};
robot.joints.head_joint.origin = {xyz: [0,1.6,0], rpy:[0,0,0]};
robot.joints.head_joint.axis = [0,1,0];

robot.joints.arm1_joint = {parent:"torso", child:"arm1"};
robot.joints.arm1_joint.origin = {xyz: [-0.5,1.3,0], rpy:[0,0,0]};
robot.joints.arm1_joint.axis = [0,0,1];

robot.joints.arm2_joint = {parent:"torso", child:"arm2"};
robot.joints.arm2_joint.origin = {xyz: [0.5,1.3,0], rpy:[0,0,0]};
robot.joints.arm2_joint.axis = [0,0,1];

robot.joints.foot1_joint = {parent:"torso", child:"foot1"};
robot.joints.foot1_joint.origin = {xyz: [-0.2,0.6,0], rpy:[0,0,0]};
robot.joints.foot1_joint.axis = [1,0,0];

robot.joints.foot2_joint = {parent:"torso", child:"foot2"};
robot.joints.foot2_joint.origin = {xyz: [0.2,0.6,0], rpy:[0,0,0]};
robot.joints.foot2_joint.axis = [1,0,0];

// specify name of endeffector frame
robot.endeffector = {};
robot.endeffector.frame = "arm1_joint";
robot.endeffector.position = [[0.5],[0],[0],[1]]

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

// define threejs geometries and associate with robot links 
links_geom = {};

// x = x ?? 
// y = z correct 
// z = y
torso_dim = [1, 1, 0.3];
links_geom["torso"] = new THREE.CubeGeometry( 1, 1, 0.3 );
links_geom["torso"].applyMatrix( new THREE.Matrix4().makeTranslation( 0, 0.6 + 0.5, 0 ) );

links_geom["head"] = new THREE.CubeGeometry( 0.4, 0.4, 0.4);
links_geom["head"].applyMatrix( new THREE.Matrix4().makeTranslation( 0, 0.2, 0  ) );

links_geom["arm1"] = new THREE.CubeGeometry( 0.6, 0.2, 0.2 );
links_geom["arm1"].applyMatrix( new THREE.Matrix4().makeTranslation( -0.3, 0, 0 ) );

links_geom["arm2"] = new THREE.CubeGeometry( 0.6, 0.2, 0.2 );
links_geom["arm2"].applyMatrix( new THREE.Matrix4().makeTranslation( 0.3, 0, 0 ) );

links_geom["foot1"] = new THREE.CubeGeometry( 0.2, 0.6, 0.2 );
links_geom["foot1"].applyMatrix( new THREE.Matrix4().makeTranslation( 0, 0 - 0.3, 0 ) );

links_geom["foot2"] = new THREE.CubeGeometry( 0.2, 0.6, 0.2 );
links_geom["foot2"].applyMatrix( new THREE.Matrix4().makeTranslation( 0, 0 - 0.3, 0 ) );



