#include <tf/transform_listener.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include "vision_manip_pipeline/Conv2DTo3D.h"
#include "vision_manip_pipeline/GetObjLoc.h"
#include "vision_manip_pipeline/GetGrasp.h"
#include "vision_manip_pipeline/PubWorkspace.h"

//TODO: double check order of w,x,y,z! -> since rest of code set up with numbers not as map, do this instead!!!!
//geometry_msgs::PoseStamped getPoseTrans(double x, double y, double z, std::map<std::string, double> ori, const std::string old_frame, const std::string new_frame){
geometry_msgs::PoseStamped getPoseTrans(double x, double y, double z, std::vector<double> ori, const std::string old_frame, const std::string new_frame){
  ros::Time now = ros::Time::now();
  // tf::TransformListener t = new tf::TransformListener(ros::Duration(10.0), true);
  tf::TransformListener t;
  t.waitForTransform(new_frame, old_frame, now, ros::Duration(3.0));

  geometry_msgs::PoseStamped pnt;
  pnt.header.frame_id = old_frame;
  pnt.header.stamp = now;
  pnt.pose.position.x = x;
  pnt.pose.position.y = y;
  pnt.pose.position.z = z;
  pnt.pose.orientation.w = ori[0]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.x = ori[1]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.y = ori[2]; //TODO: double check order of w,x,y,z!
  pnt.pose.orientation.z = ori[3]; //TODO: double check order of w,x,y,z!

  //geometry_msgs::PoseStamped newPnt = t.transformPose(new_frame, pnt);
  geometry_msgs::PoseStamped newPnt;
  t.transformPose(new_frame, pnt, newPnt);
  
  std::cout << "\nTRANSFORM: " << newPnt << '\n';
  return newPnt;
}

geometry_msgs::PointStamped transPoint(double x, double y, double z, const std::string old_frame, const std::string new_frame){
  ros::Time now = ros::Time::now();
  // tf::TransformListener t = new tf::TransformListener(ros::Duration(10.0), true);
  tf::TransformListener t;
  t.waitForTransform(new_frame, old_frame, now, ros::Duration(3.0));

  geometry_msgs::PointStamped pnt;
  pnt.header.frame_id = old_frame;
  pnt.header.stamp = now;
  pnt.point.x = x;
  pnt.point.y = y;
  pnt.point.z = z;

  // geometry_msgs::PoseStamped newPnt = t.transformPose(new_frame, pnt);
  geometry_msgs::PointStamped newPnt;
  t.transformPoint(new_frame, pnt, newPnt);

  std::cout << "\nTRANSFORM: " << newPnt << '\n';
  return newPnt;
}

void moveArm(geometry_msgs::PoseStamped newPnt){
  moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  geometry_msgs::Pose pose_target;
  pose_target.orientation.x = newPnt.pose.orientation.x;
  pose_target.orientation.y = newPnt.pose.orientation.y;
  pose_target.orientation.z = newPnt.pose.orientation.z;
  pose_target.orientation.w = newPnt.pose.orientation.w;
  pose_target.position.x = newPnt.pose.position.x;
  pose_target.position.y = newPnt.pose.position.y;
  pose_target.position.z = newPnt.pose.position.z;

  group.setPoseTarget(pose_target);
  moveit_msgs::OrientationConstraint ocm;
  /*ocm.link_name = "right_arm";
  ocm.header.frame_id = "base_link";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.0075;
  ocm.absolute_y_axis_tolerance = 0.0075;
  ocm.absolute_z_axis_tolerance = 0.0075;
  ocm.weight = 1.0;*/

  // set as the path constraints for the group
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  printf("\tPlanning...");
  sleep(10);

}

std::vector<double> rotationQuat(std::vector<double> approach, std::vector<double> axis,
  std::vector<double> binormal){
    double trace;
    std::vector<double> rotation;

    if(axis[2] < 0){
      if(approach[0] > binormal[1]){
        trace = 1 + approach[0] - binormal[1] - axis[2];
        rotation[0] = trace;
        rotation[1] = binormal[0] + approach[1];
        rotation[2] = approach[2] + axis[0];
        rotation[3] = axis[1] - binormal[2];
      }
      else{
        trace = 1 - approach[0] + binormal[1] - axis[2];
        rotation[0] = binormal[0] + approach[1];
        rotation[1] = trace;
        rotation[2] = axis[1] + binormal[2];
        rotation[3] = approach[2] - axis[0];
      }
    }
    else{
      if(approach[0] < (-binormal[1])){
        trace = 1 - approach[0] - binormal[1] + axis[2];
        rotation[0] = approach[2] + axis[0];
        rotation[1] = axis[1] + binormal[2];
        rotation[2] = trace;
        rotation[3] = binormal[0] - approach[1];
      }
      else{
        trace = 1 + approach[0] + binormal[1] + axis[2];
        rotation[0] = axis[1] - binormal[2];
        rotation[1] = approach[2] - axis[0];
        rotation[2] = binormal[0] - approach[1];
        rotation[3] = trace;
      }
    }

    rotation[0] *= 1.0 / (2.0 * sqrt(trace)); // TODO: x or w? w
    rotation[1] *= 1.0 / (2.0 * sqrt(trace)); // TODO: x or y? x 
    rotation[2] *= 1.0 / (2.0 * sqrt(trace)); // TODO: x or z? y
    rotation[3] *= 1.0 / (2.0 * sqrt(trace)); // TODO: z or w? z
}

int main(int argc, char **argv){
    std::string obj_name;
    if(argc == 2){
      obj_name = argv[1];
    }
    else{
      exit(1);
    }
    std::cout << "Requesting: " << obj_name << '\n';

    ros::init(argc, argv, "vision_manip");
/* TODO: FIGURE OUT ROSLAUNCH STUFF!!!
    std::string uuid = roslaunch.rlutil.get_or_generate_uuid(None, False);
    roslaunch.configure_logging(uuid);
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/janelle/onr_ws/src/gpd/launch/jb_tutorial1.launch"])
*/
    // rosservice call with object to find location of in YOLO
        // So get the bounding box of the image and
        // calculate the center of it as a start for the grasping window?
    //TODO: FIX THESE!
    //vision_manip_pipeline::GetObjLoc::Response resp = get_obj_loc_client(obj_name);
    
    ros::NodeHandle n;
    ros::ServiceClient objLocClient = n.serviceClient<vision_manip_pipeline::GetObjLoc>("get_object_loc");
    vision_manip_pipeline::GetObjLoc objLocSrv;
    
    objLocSrv.request.obj_name = obj_name;
    if(objLocClient.call(objLocSrv)) {
       std::cout << objLocSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call objLoc Service!" );
    }
   
    int x = (objLocSrv.response.xmax - objLocSrv.response.xmin)/2 + objLocSrv.response.xmin;
    int y = (objLocSrv.response.ymax - objLocSrv.response.ymin)/2 + objLocSrv.response.ymin;
    std::cout << "x: " << x << ' ' << "y: " << y << '\n';

    // if object not detected, then exit?
    if(x == 0 && y == 0) {
       ROS_INFO("Error: Object not detected, try again!");
       return -1;
    }

    // calculate the transform of the point in the raw image to
    // the grasping window in the depth point cloud!
    //TODO: FIX THESE!
    //vision_manip_pipeline::Conv2DTo3D::Response resp3 = conv_coord_client(x,y);

    // ros::NodeHandle n;
    ros::ServiceClient conv2DTo3DClient = n.serviceClient<vision_manip_pipeline::Conv2DTo3D>("conv_coord");
    vision_manip_pipeline::Conv2DTo3D conv2DTo3DSrv;
    
    conv2DTo3DSrv.request.x = x;
    conv2DTo3DSrv.request.y = y;
    if(objLocClient.call(conv2DTo3DSrv)) {
       std::cout << conv2DTo3DSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call convCoord Service!" );
    }

    // TODO: JB
    // transform the point from kinect frame to the orthographic frame (static tf projection)
    geometry_msgs::PointStamped newPnt = transPoint(conv2DTo3DSrv.response.newX, conv2DTo3DSrv.response.newY, conv2DTo3DSrv.response.newZ, "/head_mount_kinect_rgb_optical_frame", "/test");

    //TODO: JB
    // generate the cube from the transformed point instead
    //  first set the param for the workspace based on the response?!?
    double eps = 0.1;
    
    
    
//------------------------------    
//FIXXXXXX    

    double cube[] = {newPnt.point.x - eps, newPnt.point.x + eps, newPnt.point.y - eps, newPnt.point.y + eps, newPnt.point.z - eps, newPnt.point.z + eps};    
  
    std::cout << "cube to search for graps: " << cube << '\n'; 

    ros::param::set("/detect_grasps/workspace", cube);
    ros::param::set("/detect_grasps/workspace_grasps", cube);

    // TODO: JB
    // transform the point cloud to the orthographic frame (static tf projection) ->  will probs need to change frames in launch to get cloud in correct frame!!!
    // transPointCloud(cloud, "/local/depth_registered/points", "/local/depth_registered/trans_points"):


/* TODO: FIGURE OUT ROSLAUNCH STUFF!!!
    // relaunch the grasp stuffsssss
    launch.start()
*/
    // // rosservice call to gpd with the calculated grasping window in the
    // // point cloud to get the top grasp
     //TODO: FIX THESE!
     //vision_manip_pipeline::GetGrasp::Response resp2 = get_grasp_client(resp3.newX, resp3.newY, resp3.newZ);

    // ros::NodeHandle n;
    ros::ServiceClient getGraspClient = n.serviceClient<vision_manip_pipeline::GetGrasp>("conv_coord");
    vision_manip_pipeline::GetGrasp getGraspSrv;
    
    getGraspSrv.request.x = conv2DTo3DSrv.response.newX;
    getGraspSrv.request.y = conv2DTo3DSrv.response.newY;
    getGraspSrv.request.y = conv2DTo3DSrv.response.newZ;
    if(objLocClient.call(getGraspSrv)) {
       std::cout << getGraspSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call getGrasp Service!" );
    }


/* TODO: FIGURE OUT ROSLAUNCH STUFF!!!
    launch.shutdown()
*/
    // TODO: FIGURE OUT HOW TO RETURN 0 instead of NONE upon fail becuase no C equiv???
    // IF GRASP NOT FOUND, return 0?
    //if( resp2 == "None") {
    if( getGraspSrv.response.grasp.score.data == 0 ){
        std::cout << "Error: No grasp found, will now return\n";
        return -1;
    }

  // convert from geometry_msgs/Vector3 to std::vector<float>
  std::vector<double> axis(3);
  axis = {getGraspSrv.response.grasp.axis.x, getGraspSrv.response.grasp.axis.y, getGraspSrv.response.grasp.axis.z}; 
  std::vector<double> binormal(3);
  binormal = {getGraspSrv.response.grasp.binormal.x, getGraspSrv.response.grasp.binormal.y, getGraspSrv.response.grasp.binormal.z}; 
  std::vector<double> approach(3);
  approach = {getGraspSrv.response.grasp.approach.x, getGraspSrv.response.grasp.approach.y, getGraspSrv.response.grasp.approach.z}; 


//------------------------------    


    // // convert the top grasp format to a move-it useable format
    // // then use moveit to move the arm of the Baxter/PR2? in rviz/eal world?
    std::vector<double> ori = rotationQuat(axis, binormal, approach);
    std::cout << "ori: " << ori[0] << ',' << ori[1] << ',' << ori[2] << ',' << ori[3] << '\n';

    // visualize the workspace and grasp.......
    std::vector<double> pos(3);
    pos = {getGraspSrv.response.grasp.surface.x, getGraspSrv.response.grasp.surface.y, getGraspSrv.response.grasp.surface.z};
    std::vector<double> tilt(4);
    tilt = {ori[0], ori[1], ori[2], ori[3]}; //TODO: double check order of w,x,y,z!
    std::cout << "pos: " << pos[0] << ',' << pos[1] << ',' << pos[2] << '\n';
    std::cout << "tilt: " << tilt[0] << ',' << tilt[1] << ',' << tilt[2] << ',' << tilt[3] << '\n';
    
    //TODO: FIX THESE!
    //pub_workspace_corners_client(pos,tilt)

    // ros::NodeHandle n;
    ros::ServiceClient pubWorkspaceClient = n.serviceClient<vision_manip_pipeline::PubWorkspace>("pub_workspace_corners");
    vision_manip_pipeline::PubWorkspace pubWorkspaceSrv;
    
    pubWorkspaceSrv.request.pos = pos;
    pubWorkspaceSrv.request.ori = tilt;
    if(objLocClient.call(pubWorkspaceSrv)) {
       std::cout << pubWorkspaceSrv.response << '\n';
    }
    else{
       ROS_ERROR("ERROR: Failed to call pubWorkspace Service!" );
    }



    // TODO: JB
    //  WILL need to transform from other frame here nowwwwwww!!!!!
    // Transform point into correct PR2 frame for motion planning etc...
    geometry_msgs::PoseStamped newPose = getPoseTrans(conv2DTo3DSrv.response.newX, conv2DTo3DSrv.response.newY, conv2DTo3DSrv.response.newZ, ori, "/test", "/torso_lift_link");

//------------------------------    
//FIXXXXXX    
    // TODO: use moveit to plan to this position and orientation!
    moveArm(newPose);
//------------------------------    
}


