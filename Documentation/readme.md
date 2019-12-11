Overview
The Firefighter Indoor Navigation project is an ongoing project to develop a set of distributed applications that will allow real time 3d mapping of an interior space. The long term goal is to collect and collate data from multiple remote units into a cohesive 3d map usable from a base unit.
Because this is an ongoing project, this readme will focus more on providing an introduction for future developers than it will on describing how to use the project in its current state. Nonetheless, a brief overview of the current functionality will be provided.
For a detailed set of directions on how to set up a build environment to begin work on the project, see the file Firefighter_Indoor_Navigation_Environment_Guide.txt


Current State
As of 2019-12-11, this project has the following functionality completed:
        -Remote unit code capable of capturing and transmitting environment data
-Base unit code to receive data from one remote unit and creating both a sparse point cloud model and dense point cloud model from that data


Future Plans
Future goals that remain include:
        -Ability for the base unit to collect data simultaneously from multiple remote units
        -Ability for the base unit to collate these data into a single, combined model
        -Ability for the base unit to perform object recognition based mapping on the model
-Ability for the base unit to create a simplified view of the model to transmit to the remote unit
        -Ability for the remote unit to receive and display simplified map from the base unit


Application Overview
Remote Unit - The remote unit application captures and transmits data from an Intel RealSense d435i camera. The application ui is a single window with 3 buttons:
        -Test: Activates a window showing the current output from the attached camera                
        -Send: Begins sending data from the camera
        -Config: Allows for the editing of application settings
Base Unit - The base unit application receives data from a single remote unit and creates a set of point clouds from it. The application ui has the following components:
        -A fixed portion showing the current video frame being processed
        -A tab showing a sparse point cloud with a track of the camera’s position history
        -A tab showing a sparse point cloud, rendered from above
        -A tab showing a dense point cloud
        -A tab with application settings


Technical Overview
This section provides a brief, high level overview of the operation of each application, for more information on a specific portion, please refer to the comments in that section of the source code.
Remote Unit - Initialization
-Program enters main, which creates and initializes a Qt based ui and passes execution to mainWindow
-Settings are loaded from config.ini file
-A ROS node is created(ROS is a middleware layer that abstracts the data transmission used by the application)
-The main ui is entered and waits until a button is selected
Remote Unit - Test
        -The application polls for attached Intel RealSense devices
        -If exactly one is found, the device’s input system is activated
-This then feeds into a simple window that will output live data from the device until it is closed
Remote Unit - Send
-Control is passed to rosNodeWidget, and the init() function is called
-This initializes the ROS node created in main during startup with our current config settings
        -If these settings are valid, and the ROS master is reachable, the run() function is called
        -This function polls for attached RealSense devices
        -If exactly one is found, it is activated
-The data from the device is then captured, processed, and sent over three different ROS topics(one for color video, one for depth data, and one for IMU data)
-The application continues transmitting data until the Stop button is pressed, whereupon the application is returned to the main menu
Remote Unit - Config
        -The application settings are made visible and can be edited
        -A brief overview of the settings:
                -The ROS IPs are used to determine the ROS network setup
                -The topic names are the names of the ROS topics used for each data type
-Publish rate is the number of times per second data is published over those topics
-The remaining values are filter values for the depth sensor, for more information, please see: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md 
        -Done saves the settings and returns to the main menu


Base Unit - Initialization
-Program enters main, which creates and initializes a Qt based ui and passes execution to mainWindow
-Settings are loaded from config.ini file
-A ROS node is created
        -Control is passed to rosNodeWidget, and the init() function is called
-This initializes the ROS node created in main during startup with our current config settings
        -If these settings are valid, and the ROS master is reachable, the run() function is called
-An ORB_SLAM2 object is created, which will create four output windows(two sparse point cloud viewers, a dense point cloud viewer, and a video frame viewer)
-These windows are then captured and integrated into the main ui
-A subscribe loop is entered, where a color image and depth image are received, and then passed to a pointCloudWidget’s processFrames function
-processFrames does some minor frame processing and then passes the frame pair to the ORB_SLAM2 library
-The frame data is then processed by ORB_SLAM2 into a sparse point cloud and a dense point cloud


ROS Setup
This project uses ROS as a middleware layer to handle transmission of information. The remote unit application has a ROS node publishing data on a set of ROS topics, and the base unit application has a ROS node subscribing to those topics. In order to run the applications successfully, the base unit must be running the ROS Core, which can be done by executing the command:
* roscore
The base unit and remote unit must also have the ROS IP values in their settings pointed correctly. If both are running on the same machine, such as for testing or development, they can be set as follows:
        -Base application
                -ROS Master IP: http://{MACHINE-NAME}:11311/ 
                -ROS Local IP: {MACHINE-NAME}
        -Remote application
                -ROS Master IP: http://{MACHINE-NAME}:11311/ 
                -ROS Local IP: {MACHINE-NAME}
If the two are connected over a network, they can be set as follows:
        -Base application
                -ROS Master IP: http://{MACHINE-NAME}:11311/ 
                -ROS Local IP: {MACHINE-NAME}
        -Remote application
                -ROS Master IP: http://{BASE-UNIT-IP}:11311/ 
                -ROS Local IP: {REMOTE-UNIT-IP}


Miscellaneous
This is a collection of notes and thoughts on the project that do not necessarily fit into any other category.
-The ROS Topic name settings are currently unneeded, though they may be useful in the future if additional remote units are added, as a way to differentiate the data streams from one another
-One major future improvement would be to improve the quality of the depth data being sent. There are several possibilities for this:
        -Improving the filter selection and settings used
        -Changing the conversion to a CV Mat, which currently reduces bit depth
        -Adding additional post-processing steps
-While the software is able to function on any network where the two machines can communicate, bandwidth is a major issue at this time. No success has been had in getting the system to operate at a successful speed over Wi-Fi. Changing the message type sent from the remote unit from ROS imageMessages to ROS compressedImageMessages may help, as may reducing either the frame size or the frame rate being sent. Due to development issues, however, none of these options has been sufficiently explored.
-Each Intel RealSense camera has a software accessible ID. This may be useful as a method of distinguishing remote units when multiple units are connected.
-The Intel RealSense cameras are factory calibrated, but may still have slight variances in the intrinsic values. A small console program project is included that will output the camera intrinsics of an attached camera to the console. These can be used to modify the values in the base unit’s camera.yaml file to improve accuracy.
-It is recommended to run the base unit once without roscore running to allow you to change the settings without it automatically trying to connect using the default values
-The reset options and loop closing systems of ORB_SLAM2 are currently disabled due to unknown underlying issues, these issues should be rectified and the systems enabled at some point.