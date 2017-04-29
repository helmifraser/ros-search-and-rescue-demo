#include "sar/ObDet.h"

// CONSTRUCTOR
// The method which gets called when the object is created.
// -----------------------------------------------------------------------------------------------------------------------------
//  Input | Type          | Name    | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  1   | NodeHandle      | nh_       | Links the method to the specified node, making it accessible
//  2   | ImageTranpsort  | it_       | Allows the constructor to acces images taken by the kinect from the Turtlebot

// Output | Type  | Name    | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  N/A   | N/A   | N/A     | N/A

ObDet::ObDet(ros::NodeHandle nh_, image_transport::ImageTransport it_)
{
  odom_sub = nh_.subscribe<nav_msgs::Odometry>("/odom", 1 , &ObDet::odomcallback, this);
  image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, &ObDet::imagecallback, this, image_transport::TransportHints("compressed"));
  laser_sub = nh_.subscribe<sensor_msgs::LaserScan>("/scan_laser", 1, &ObDet::lasercallback, this);
  pub_vel = nh_.advertise<geometry_msgs::Twist>("cmd_vel_mux/input/teleop", 1);
  pub_pcl = nh_.advertise<sensor_msgs::PointCloud2>("/mycloud", 1);
  pub_mark = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
  count = count_substage = 1;
  Colour = amount_objects_hit = lastx = lasty = start_angle = current_angle = target_angle = check 
         = stage = substage = sub_sub_stage = body_count = object_count = object_distance = 0;

  metre = 0.965452;
  metreback = 0.960004;
  degree = 0.0055556;
  error = 2*degree;
  turn_angle = 0.703022;
  ratio = 2.3333;
  turn_check = travel_check = clear_path = clear_turn = axis = sub_clear_path = true;
  body_detected = object_detected = stop = false;
  angular_speed_90 = -0.5;
  angular_speed_360 = -0.3;
  linear_speed = 0.3;
  size_of_array = 100;
  morphsize = 7;

  OPENCV_WINDOW1 = "Live Feed";
  OPENCV_WINDOW2 = "Body Detection";
  OPENCV_WINDOW3 = "Obstacle Detection";
  cv::namedWindow(OPENCV_WINDOW1, CV_WINDOW_NORMAL);
  cv::namedWindow(OPENCV_WINDOW2, CV_WINDOW_NORMAL);
  cv::namedWindow(OPENCV_WINDOW3, CV_WINDOW_NORMAL);

  for (int i = 0; i < size_of_array; ++i)
    bodies_angles[i] = bodies_xcoords[i] = bodies_ycoords[i] = objects_angles[i] = objects_xcoords[i] 
                     = objects_ycoords[i] = dist_saved_objects[i] = dist_saved_bodies[i]=0;
}

// DECONSTRUCTOR
// The method which gets called when the object is stopped. Destroys the opencv windows and runs a method which processes 
//    information gathered during the run.
// -----------------------------------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A     | N/A

// Output | Type  | Name    | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  N/A   | N/A   | N/A     | N/A

ObDet::~ObDet()
{
  cv::destroyWindow(OPENCV_WINDOW1);
  cv::destroyWindow(OPENCV_WINDOW2);
  cv::destroyWindow(OPENCV_WINDOW3);
  process(); 
}

// METHOD - process
// This method is called in the deconstructor, cycling through the arrays it uses trigonometric functions to calculate the 
//    positions of the objects and bodies that were detected during the run, and stores them in new arrays.
// -----------------------------------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A     | N/A
//

// Output | Type  | Name    | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A     | N/A

void ObDet::process()
{
    for (int i = 0; i < size_of_array; ++i)
    { 
      if(bodies_angles[i] > 0 & bodies_angles[i] < 0.7 | bodies_angles[i] < 0 & bodies_angles[i] > -0.7)
        bodies_angles[i] = bodies_angles[i]*(3.1415926536/1.4);
        else if(bodies_angles[i] > 0.7 & bodies_angles[i] < 1)
          bodies_angles[i] = 3.1415926536/2 + (bodies_angles[i]-0.7)*(3.1415926536/0.6);
        else if(bodies_angles[i] < -0.7 & bodies_angles[i] > -1)
          bodies_angles[i] = -3.1415926536/2 + (bodies_angles[i]+0.7)*(3.1415926536/0.6);
      

      if(objects_angles[i] > 0 & objects_angles[i] < 0.7 | objects_angles[i] < 0 & objects_angles[i] > -0.7)
        objects_angles[i] = objects_angles[i]*(3.1415926536/1.4);
        else if(objects_angles[i] > 0.7 & objects_angles[i] < 1)
          objects_angles[i] = 3.1415926536/2 + (objects_angles[i]-0.7)*(3.1415926536/0.6);
        else if(objects_angles[i] < -0.7 & objects_angles[i] > -1)
          objects_angles[i] = -3.1415926536/2 + (objects_angles[i]+0.7)*(3.1415926536/0.6);
      
      bodies_xcoords[i] = bodies_xcoords[i] + dist_saved_bodies[i]*cos(bodies_angles[i]);
      bodies_ycoords[i] = bodies_ycoords[i] + dist_saved_bodies[i]*sin(bodies_angles[i]);

      objects_xcoords[i] = objects_xcoords[i] + dist_saved_objects[i]*cos(objects_angles[i]);
      objects_ycoords[i] = objects_ycoords[i] + dist_saved_objects[i]*sin(objects_angles[i]);

      bodies_angles[i] = bodies_angles[i]*(180/3.1415926536);
      objects_angles[i] = objects_angles[i]*(180/3.1415926536);
    }
}

// METHOD - lasercallback
// This method retrieves the laser scan points transported over the ros node, stores them in an array, then checks the distances
//    for various scenarios. The data had to be filtered for inf's and nan's, and convert these to 100 and 0 respectively, in 
//    order for the processing to work.
// -----------------------------------------------------------------------------------------------------------------------------
//  Input | Type                              | Name      | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  1     | sensor_msgs::LaserScan::ConstPtr& | scan      | this is the link to the message sent over the ros node from the bot
//                                                        |   containing the laser scan points taken by the hokuyo laser
//
// All Outputs are linked to global variables
//
// Output | Type    | Name            | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  1     | bool    | clear_path      | checks whether the space ahead of the turtlebot is clear, used in method traveldecision
//  2     | bool    | clear_turn      | checks whether after a turn there is nothing obstructing the avoidance path, used in
//                                    |   method traveldecision
//  3     | float   | object_distance | takes the lowest value (ie closest point) detected by laser, and stores it, used 
//                                    |   in method turn360


void ObDet::lasercallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  float dist_vals[scan->ranges.size()];

  for(int i = 0; i<scan->ranges.size(); ++i)
  {
    if(scan->ranges[i] > 0.001 && scan->ranges[i]<100)
      dist_vals[i] = scan->ranges[i];
      else
        dist_vals[i] = 100;
  }

  for(int j = 200; j<300;++j)
  {
    if(dist_vals[j] > 0.2)
      clear_path = true;
      else
      {
        clear_path = false;
        j=300;
      }
  }

  for(int k = 200; k<300;++k)
  {
    if(dist_vals[k] > 0.4)
      clear_turn = true;
      else
      {
        clear_turn = false;
        k=300;
      }
  }

  int range_min = 150;
  int range_max = 350;
  int range_diff = range_max - range_min;
  float dist_av[range_diff];

  for(int l = range_min; l<range_max; ++l)
    dist_av[l-range_min] = dist_vals[l];

  object_distance = dist_av[0];
    
  for(int l = 0; l<range_diff;++l)
  {
    if(dist_av[l]<object_distance)
      object_distance = dist_av[l];
      else if(dist_av[l]>=object_distance)
        object_distance = object_distance;
  }


  sensor_msgs::PointCloud2 cloud;
  projector.transformLaserScanToPointCloud("base_laser_link",*scan, cloud, listener);
  pub_vel.publish(cloud);
}

// METHOD - odomcallback
// This method retrieves the odometry values published by the turtlebot. The odometry values are relative to the start point.
//  This method also executes the search pattern, and executes the methods that visualize the path taken by the turtlebot
// -----------------------------------------------------------------------------------------------------------------------------
//  Input | Type                              | Name      | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  1     | nav_msgs::Odometry::ConstPtr& | odom_msg      | this is the link to the message sent over the ros node from the bot
//                                                        |   containing the odometry locations of the turtlebot
//
// All Outputs are linked to global variables
//
// Output | Type    | Name  | Description
// -----------------------------------------------------------------------------------------------------------------------------
//  None  | float   | x     | x-position relative to start point
//  None  | float   | y     | y-position relative to start point
//  None  | float   | z     | z-angle relative to start direction

void ObDet::odomcallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
  x = odom_msg->pose.pose.position.x;
  y = odom_msg->pose.pose.position.y;
  z = odom_msg->pose.pose.orientation.z;
  
  createline();
  createmodel();
  search(); 

  std::cout << "\n";  
  std::cout << "stage: " << stage << "\n";
  std::cout << "substage:  " <<substage<< "\n";
  std::cout << "sub_sub_stage:  " <<sub_sub_stage<< "\n";
  std::cout << "\n";
}
// METHOD - imagecallback
// This method retrieves the image transported over the ros node, processes it, and displays it.
// ----------------------------------------------------------------------------------------------------
//  Input | Type                        | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1     | sensor_msgs::ImageConstPtr& | msg     | this is the link to message sent over the node from the bot
//        |                             |         |   containing the image taken by kinect
//
// All Outputs are linked to global variables
//
// Output | Type  | Name             | Description
// ----------------------------------------------------------------------------------------------------
//  1     | bool  | object_detected  | set tot true when an object is detected, used in method turn360
//  2     | bool  | body_detected    | set tot true when a body is detected, used in method turn360

void ObDet::imagecallback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr; 

  //define the Mat structures that will hold the HSV and processed images.
  cv::Mat imgHSV_body, imgHSV_object, threshold_Image_body,threshold_Image_object; 

  bool centre_check_body = false;
  bool centre_check_object = false;

  try
  {
    //allows the image to be copied in to a variable and modified in this method
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);  
  }
  
  //Any errors that causes the above try to fail will be printed
  catch (cv_bridge::Exception& e) 
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  iLowH_object = 104;
  iHighH_object = 113;
  iLowS_object = 198;
  iHighS_object = 242;
  iLowV_object = 0;
  iHighV_object = 255;


  iLowH_body = 19;
  iHighH_body = 31;
  iLowS_body = 174;
  iHighS_body = 209;
  iLowV_body = 0;
  iHighV_body = 255;

  //all processes will be run twice for the seperate colors of the objects and bodies
  //converts the image's BGR encoding to HSV encoding for suitable processing
  cv::cvtColor(cv_ptr->image, imgHSV_body, cv::COLOR_BGR2HSV); 

  cv::cvtColor(cv_ptr->image, imgHSV_object, cv::COLOR_BGR2HSV);


  //filters the image to show only the desired colour
  cv::inRange(imgHSV_body, cv::Scalar(iLowH_body, iLowS_body, iLowV_body), cv::Scalar(iHighH_body, iHighS_body, iHighV_body), threshold_Image_body); 

  cv::inRange(imgHSV_object, cv::Scalar(iLowH_object, iLowS_object, iLowV_object), cv::Scalar(iHighH_object, iHighS_object, iHighV_object), threshold_Image_object); 


  // perform morphology operations, in this case opening
  cv::erode(threshold_Image_body, threshold_Image_body, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  cv::dilate(threshold_Image_body, threshold_Image_body, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) ); 

  cv::erode(threshold_Image_object, threshold_Image_object, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
  cv::dilate(threshold_Image_object, threshold_Image_object, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );


  // more morphology, to reduce noise in the image, can be user customisable via trackbar "Morph Size"
  cv::morphologyEx(threshold_Image_body, threshold_Image_body, 2, cv::getStructuringElement( 2, cv::Size(morphsize+1, morphsize+1)));

  cv::morphologyEx(threshold_Image_object, threshold_Image_object, 2, cv::getStructuringElement( 2, cv::Size(morphsize+1, morphsize+1)));


  //instantiates a Moments object to calculate moments from the image
  cv::Moments oMoments_body = cv::moments(threshold_Image_body);

  cv::Moments oMoments_object = cv::moments(threshold_Image_object);


  // define zeroth, first and second order moments
  double dM01_body = oMoments_body.m01;
  double dM10_body = oMoments_body.m10;
  dArea_body = oMoments_body.m00;

  double dM01_object = oMoments_object.m01;
  double dM10_object = oMoments_object.m10;
  dArea_object = oMoments_object.m00;

  // determine centroid of object
  posX_body = dM10_body / dArea_body;
  posY_body = dM01_body / dArea_body;

  posX_object = dM10_object / dArea_object;
  posY_object = dM01_object / dArea_object;


  //Store centre of image for reference to movement of centroid of object
  center = 331;

  if (posX_body < center + 20 & posX_body > center - 20)
    centre_check_body = true;


  if (posX_object < center + 20 & posX_object > center - 20)
    centre_check_object = true;


  //overlay circle over "Live Feed" with the coordinates of the object centroid
  cv::circle(cv_ptr->image, cv::Point(posX_body, posY_body), 10, CV_RGB(255,0,0)); 

  cv::circle(cv_ptr->image, cv::Point(posX_object, posY_object), 10, CV_RGB(0,255,0)); 

  
  //display processed image
  cv::imshow(OPENCV_WINDOW2, threshold_Image_body); 

  cv::imshow(OPENCV_WINDOW3, threshold_Image_object); 


  //display live feed with circle overlay
  cv::imshow(OPENCV_WINDOW1, cv_ptr->image); 
      
  cv::waitKey(1);
  

  if(dArea_object>10000 & centre_check_object == true)
    object_detected = true;
    else
      object_detected = false;

  if(dArea_body>10000 & centre_check_body == true)
    body_detected = true;
    else
      body_detected = false;
}

// METHOD - traveldecision
// This method determines what to do when an object is encountered. It has a check in place for when 
//    the first obstacle is detected, after which it will turn 90 degrees. It will then check if 
//    anything else is blocking its path, if not, it will avoid the object, if there is another obstacle
//    it will turn around 180 degrees and try to avoid it on the other side of the object.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name        | Description
// ----------------------------------------------------------------------------------------------------
//  1     | float | distance_in | The amount the turtlebot will move forward, in which obstacles need to 
//                              |   checked for.
//
// Output | Type  | Name | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A

void ObDet::traveldecision(float distance_in)
{
  if(clear_path == true & amount_objects_hit == 0)
  {
    travel(distance_in);

    if(travel_check == true)
      ++stage;
  }

  if(clear_path == false | amount_objects_hit>0)
  {
    if(count_substage==1)
    {
      amount_objects_hit = 1;
      ++count_substage;
    }
   
    std::cout << "substage:  " <<substage<< "\n";
    std::cout << "amount_objects_hit: " <<amount_objects_hit<<"\n";
    std::cout << "sub_sub_stage:  " <<sub_sub_stage<< "\n";
    std::cout << "avoid again boolean: "<<sub_clear_path << "\n";

    if(substage==0)
    {
      turn90(false);
      if(turn_check == true)
        ++substage;
    }
    else if(substage==1)
    {
      if(clear_turn == false)
      {
        ++amount_objects_hit;
        ++substage;
      }
        else if(travel_check==false)
          ++substage;
        else
          travel(0.5);
    }
    //When only one obstacle is detected
    else if(substage == 2 & amount_objects_hit ==1 & sub_clear_path == true)  
    {
        switch(sub_sub_stage)
        {
          case 0:
            travel(0.5); 
            if(travel_check==true)
              ++sub_sub_stage; 
            break;
          
          case 1:
            turn90(true);
            if(turn_check==true)
              ++sub_sub_stage;
            break;

          case 2:
            travel(0.7);
            if(clear_path == false)
            {
              ++amount_objects_hit;
              sub_clear_path = false;  
            }
              else if(travel_check==true)
              {
                ++sub_sub_stage;
                ++stage;
              }
            break;
          
          case 3:
            turn90(true);
            if(turn_check==true)
                ++sub_sub_stage;
            break;
          
          case 4:
            travel(0.4);
            if(travel_check==true)
              ++sub_sub_stage;           
            break;

          case 5:
            turn90(false);
            if(turn_check==true)
                ++stage;
            break;        
               
        }
    }
    // when something is blocking movement forward to avoid initial obstacle after turn
    else if(substage == 2 & amount_objects_hit ==2) 
    {
      switch(sub_sub_stage)
      {
        case 0:
          turn90(true);
          if(turn_check==true)
            ++sub_sub_stage;
          break;

        case 1:
          turn90(true);
          if(turn_check==true)
            ++sub_sub_stage;
          break;

        case 2:
          travel(0.5); 
          if(travel_check==true)
            ++sub_sub_stage; 
          break;

        case 3:
          turn90(true);
          if(turn_check==true)
            ++sub_sub_stage;
          break;

        case 4:
          travel(0.7);
          if(travel_check == true)
          {
            ++sub_sub_stage;
            ++stage;
          }
          break;
      }
    }
  }
}

// METHOD - search
// This method executes the search pattern, which simply takes the turtlebot along the middle of the x-axis
//    in the y-direction. It constantly checks for obstacles when moving forward.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name        | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A
//
// Output | Type  | Name | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A
void ObDet::search()
{
  std::cout << "x: " << x << " | "<< "y: " << y << " | "<< "z: " << z << " \n ";
  switch(stage)
  {
    case 0:
      traveldecision(1.1);
      break;
  
    case 1:
      turn90(false);
      if(turn_check==true)
        ++stage;
      break;
    
    case 2:
      traveldecision(1);
      break;      
    
    case 3:
      turn360();
      break;
    
    case 4:
      traveldecision(1);
      break;      
    
    case 5:
      turn360();
      break;

    case 6:
      traveldecision(1);
      break;      
      
    case 7:
      turn360();
      break;
  }
}

// METHOD - createline
// This method publishes a line to RViz which is created by displaying points with the stored x,y values
//    published by odometry, and linking them with a line.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name        | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A
//
// Output | Type  | Name | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A

void ObDet::createline()
{
  points.header.frame_id = line.header.frame_id = "/base_laser_link";
  points.header.stamp = line.header.stamp = ros::Time::now();

  points.ns = line.ns = "track_turtlebot";
  points.action = line.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line.pose.orientation.w  = 1.0; 
  points.type = visualization_msgs::Marker::POINTS;
  line.type = visualization_msgs::Marker::LINE_STRIP;

  points.scale.x = points.scale.y = 0.02;
  line.scale.x = line.scale.y = 0.01;

  points.color.g = points.color.a = 1.0;
  line.color.b = line.color.a = 1.0;
  ros::Rate r(30);
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0;

  points.points.push_back(p);
  line.points.push_back(p);
  pub_mark.publish(points);
  pub_mark.publish(line);
  r.sleep();
}

// METHOD - createmodel
// This method publishes a cylindrical shape to RViz to the current x,y odometry value of the turtlebot
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name        | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A
//
// Output | Type  | Name | Description
// ----------------------------------------------------------------------------------------------------
//  None  | N/A   | N/A  | N/A

void ObDet::createmodel()
{
  marker.header.frame_id = "/base_laser_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "cube";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  
  ros::Rate r(30);
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0;
  pub_mark.publish(marker);
  r.sleep();
}


// METHOD - travel
// This method executes linear movement in any axis for a given distance.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1   | float   | distance      | Desired distance in metres 

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | void  | N/A     | N/A
void ObDet::travel(float distance)
{
  if(count == 1)
  {
    travel_check = false;
    lastx = x;
    lasty = y;
    std::cout << lastx << "|" << lasty << "\n";
    ++count;
  }
    else
    {
      float desired = distance*metre;
      float travelled;

      switch(axis)
      {
        case true:
          travelled = x - lastx;
          std::cout << "Travelling in the x-axis\n";
          break;

        case false:
          travelled = y - lasty;
          std::cout << "Travelling in the y-axis\n";
          break;
      }

      if (travelled < 0)
        travelled = travelled*-1;

      std::cout << "desired: "<< desired << "  travelled: " << travelled << "\n";

      if (travelled < desired)
      {
        ObDet::straight(true);
        travel_check = false;
      }
   
      std::cout << "now I'm at: " << x << "|" << y << "\n";

      if (travelled > desired)
      {
        cmd_stored.linear.x = cmd_stored.linear.y = cmd_stored.linear.z = cmd_stored.angular.x = cmd_stored.angular.y = cmd_stored.angular.z = 0;
        std::cout << "reached the distance fam innit" << "\n";

        ++check;
        count = 1;
        pub_vel.publish(cmd_stored);
        travel_check =true;
      }
    }
  
  cmd_stored.linear.x = cmd_stored.linear.y = cmd_stored.linear.z = cmd_stored.angular.x = cmd_stored.angular.y = cmd_stored.angular.z = 0;
}  

// METHOD - straight
// This method publishes a linear velocity command in the x-axis for a given direction.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1   | bool   | direction      | Desired direction of movement, true for forward, false for backward. 

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | void  | N/A     | N/A
void ObDet::straight(bool direction)
{
  cmd_stored.linear.x = cmd_stored.linear.y = cmd_stored.linear.z = 
  cmd_stored.angular.x = cmd_stored.angular.y = cmd_stored.angular.z = 0;

  if(direction == true)
    cmd_stored.linear.x = linear_speed;

  if(direction == false)
    cmd_stored.linear.x = -linear_speed;

    pub_vel.publish(cmd_stored);
}

// METHOD - turn360
// This method executes a stationary 360 degree turn while simultaneously checking whether an object or a body
// has been detected and saving the angular pose of the robot at the point of detection.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
// None  | void  | N/A     | N/A

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | void  | N/A     | N/A
void ObDet::turn360()
{
  bool halt;
  int decide;

  if(count == 1)
  {
    current_angle = start_angle = target_angle = z;

    std::cout << "Starting angle: " << start_angle << "\n";
    ++count;
    past = false;
    halt = false;
  }  
    else
    {
      std::cout << "Starting angle: " << start_angle << "\n";
      std::cout << "Target angle: " << target_angle << "\n";


      current_angle = z;

      if(start_angle < 0)
        decide = 0;
          

      if(start_angle - turn_angle < -1)
        decide = 1;

      if(start_angle > 0 & start_angle < 1)
        decide = 2;
          
      switch(decide)
      {
        case 0:
          target_angle = start_angle - 10*error;
          std::cout << "Case 0 " << target_angle << "\n";
          if (current_angle < target_angle)
          {
            past = true;   
            std::cout << "past: " <<past<<"\n";   
          }

          if (past == true & (current_angle < start_angle + error & current_angle > start_angle - error))
            halt = true;
            else
              halt = false;
            break;

        case 1:
          target_angle = -1*target_angle;
          std::cout << "Case 1 " << target_angle << "\n";

          if (current_angle < start_angle - 10*error & current_angle > 0)
          {
            past = true;
            std::cout << "past: " <<past<<"\n";
          }
            
          if (past == true & (current_angle < start_angle + error & current_angle > start_angle - error))
            halt = true;
            else
              halt = false;
            break;

        case 2:
          target_angle = start_angle - 10*error;
          std::cout << "Case 2 " << target_angle << "\n";

          if (current_angle < target_angle)
          {
            past = true;
            std::cout << "past: " <<past<<"\n";
          }

          if (past == true & (current_angle < start_angle + error & current_angle > start_angle - error))
            halt = true;
            else
            halt = false;
          break;
        }

        if (object_detected == true | body_detected == true)
        {
          std::cout << "OBSTACLE DETECTED!!\n";

          if (body_detected == true)
          {
            bodies_angles[body_count] = z; 
            std::cout << "Bodies angle: " << bodies_angles[body_count] << "\n";
            std::cout << "Body Detected, Turtlebot Position: (" << x << "," << y << "," << z <<")\n"; 

            dist_saved_bodies[body_count] = object_distance;

            bodies_xcoords[body_count] = x;
            bodies_ycoords[body_count] = y;
            std::cout << "Body co-ordinates: " << bodies_xcoords[body_count] << " | " << bodies_ycoords[body_count] << "\n";
            ++body_count;
          }

          if (object_detected == true)
          {
            objects_angles[object_count] = z; 
            std::cout << "Object angles: " << objects_angles[body_count] << "\n";
            std::cout << "Object Dectected, Turtlebot Position: (" << x << "," << y << "," << z <<")\n"; 

            dist_saved_objects[object_count] = object_distance;

            objects_xcoords[object_count] = x;
            objects_ycoords[object_count] = y;
            std::cout << "Object co-ordinates: " << objects_xcoords[body_count]<< " | " << objects_ycoords[body_count]  << "\n";  
            ++object_count;   
          }
        }

        switch(halt)
        {
          case true:
            cmd_stored.angular.z = 0;
            pub_vel.publish(cmd_stored);
            ++check;
            count = 1;
            decide = 3;
            std::cout << "Halt conditions met. Stopped. \n";
            ++stage;
            break;

          case false:
            cmd_stored.angular.z = angular_speed_360;
            pub_vel.publish(cmd_stored);
            std::cout << "Turning...\n";
            std::cout << "Past boolean " << past << "\n";
            break;
        } 

    }
}
// METHOD - turn90
// This method executes a stationary 90 degree turn in a given direction.
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
// 1      | bool  | right   | Desired direction of 90 degree turn. True for right, false for left.

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | void  | N/A     | N/A
void ObDet::turn90(bool right)
{
  if(count == 1)
  {
    current_angle = start_angle = z;
    std::cout << "Starting angle: " << start_angle << "\n";
    ++count;
  }

  desired = turn_angle*degree;
  current_angle = z;

  int decide = 0;

  switch(right)
  {
    case true:
      if (start_angle < 0 & start_angle > -1*turn_angle || start_angle == 0)
          decide = 1;

      if (start_angle < -1*turn_angle & start_angle > -1)
          decide = 2;

      if (start_angle > turn_angle & start_angle < 1)
          decide = 3;

      if (start_angle > 0 & start_angle < turn_angle)
          decide = 4;

      // 0 to -1
      switch(decide)
      {
        case 1:
          target = start_angle/ratio + -1*turn_angle;      
          std::cout << "Case 1. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          } 
            else
              stop = false;
          break;

        // -1 to 1
        case 2:
          target = 1 + start_angle + turn_angle;
          std::cout << "Case 2. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          }
            else
              stop = false;
          break;

        // 0 to -1, 1 to 0
        case 3:
          target = turn_angle - ratio*(1 - start_angle);
          std::cout << "Case 3. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error & current_angle > 0)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          } 
            else
              stop = false;
          break;

        // 1 to 0, 0 to -1
        case 4:
          target = start_angle - turn_angle;
          std::cout << "Case 4. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          }
            else
              stop = false;
          break;
      }
    break;

  case false:
      std::cout << "reached left turn\n";
      if (start_angle > 0 & start_angle < turn_angle | start_angle == 0)
          decide = 1;

      if (start_angle > turn_angle & start_angle < 1)
          decide = 2;

      if (start_angle < -1*turn_angle & start_angle < -1)
          decide = 3;

      if (start_angle < 0 & start_angle > -1*turn_angle)
          decide = 4;  

      switch(decide)
      {
        case 1:
          target = turn_angle + start_angle/ratio;
          std::cout << "Case 1. Left. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          }
            else
              stop = false;
          break;

        case 2:
          target = (start_angle - turn_angle) - 1;
          std::cout << "Case 2. Left. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          }
            else
              stop = false;
        break;

        case 3:
          target = (1 + start_angle)*ratio - turn_angle;;
          std::cout << "Case 3. Left. Target odom:  " << target << "\n";

          if (current_angle < target + error & current_angle > target - error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          }
            else
              stop = false;
          break;

        case 4:
          target = (turn_angle + start_angle);
          std::cout << "Case 4. Left. Target odom:  " << target << "\n";

          if (current_angle < target - error & current_angle > target + error)
          {
            stop = true;
            std::cout << "I'm at the target, give or take a few. \n";
          }
            else
              stop = false;
          break;
      }
  }

  switch(stop)
  {
    case true:
      cmd_stored.angular.z = cmd_stored.linear.x = 0;
      pub_vel.publish(cmd_stored);
      current_angle = z;
      std::cout << "Finished. I turned 90 degrees. My position is " << current_angle << ".\n";
      ++check;
      count = 1;
      axis = !axis;
      turn_check = true;
      break;

    case false:
      if (right == false)
        cmd_stored.angular.z = -1*angular_speed_90;
        else
          cmd_stored.angular.z = angular_speed_90;

      turn_check = false;
      pub_vel.publish(cmd_stored);
      std::cout << "Hold on, I'll try to turn 90 degrees \n";
      break;
  }
}

// MAIN METHOD
// Runs the class
// ----------------------------------------------------------------------------------------------------
//  Input | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  1   | int   | argc      | N/A
//  2   | char  | argv      | N/A

// Output | Type  | Name    | Description
// ----------------------------------------------------------------------------------------------------
//  None  | void  | N/A     | N/A

int main(int argc, char** argv)
{
  //initialize ros
  ros::init(argc, argv, "ObDet");

  //define ros nodehandle
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  
  //as long as a roscore or minimal launch is running, continue to loop
  while(ros::ok)
  {
    //initialize class, calls constructor, using image transporter and nodehandle
    ObDet od(nh, it);
    //loop unless Ctrl-C is used in terminal
    ros::spin();
    break;
  }
  return 0;
}