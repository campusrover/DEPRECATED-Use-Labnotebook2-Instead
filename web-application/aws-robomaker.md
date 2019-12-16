# AWS RoboMaker

## Introduction:

Amazon Web Services RoboMaker is an online service which allows users to develop and test robotic applications online. Robomaker has a number of advanced features, but this notebook page will focus on developing nodes and simulating them in Gazebo. To use RoboMaker, you will also need to use AWS S3 and Cloud9.

## Getting started:

Create an AWS root account if you do not already have one. A free tier account will suffice for getting started, though make note that under a free tier membership you will be limited to 25 Simulation Units \(hours\) for the first twelve months.

Once you have an AWS account, you should create an IAM for your account. AWS recommends not using your root user account when using services like RoboMaker. [To learn how to set up IAM, click here](https://docs.aws.amazon.com/IAM/latest/UserGuide/getting-started_create-admin-group.html). Remember the username and password of the account you create. Additionally, save the link to where you can log in with those credentials.

Going forward, you should be logged in with your IAM account. Log into AWS with your IAM, then proceed.

### Amazon Documentation and Mini Tutorial:

RoboMaker has a limited documentation set that can help you use the software. The “getting Started” section can help familiarize yourself with the software by working with a sample application. [You can find this tutorial by clicking here](https://docs.aws.amazon.com/robomaker/latest/dg/getting-started.html).

### Creating an S3 bucket:

From the AWS Management Console, type “S3” into the “find services” field and click on S3 in the autofill list below the entry box. From the S3 Management Console, click “Create Bucket”

* On the first page, enter a name for your bucket. Under region, make sure that it is US East \(N Virginia\), NOT US East \(Ohio\), as RoboMaker does not work in the Ohio region.
* Skip step 2, “configure options”
* In step 3, “Set Permissions”, uncheck all four boxes
* Click “Create Bucket”

This bucket is where your bundled robotic applications will be stored.

## Creating a Development Environment:

Beck at the AWS Management Console, type “robomaker” in to the same entry field as S3 in the last part to go to the RoboMaker Management Console. In the left hand menu, under Development, select “Development environments” then click “Create Environment”

* Give your environment a name
* Keep the instance type as its default, m4 large
* Under networking, select the default VPC and any subnet, then click “create” to finish creating your environment

In your Cloud9 environment, use the bash command line at the bottom of the screen and follow these instructions: [how to create a new RoboMaker application](https://docs.aws.amazon.com/robomaker/latest/dg/application-create-new.html) to create the directories needed to work with ROS.

At the end, you will have both a robot workspace and a simulation workspace. The robot workspace \(`robot_ws`\) contains source files which are to be run on the robot. The simulation workspace \(`simulation_ws`\) contains the launch files and the world files needed to run gazebo simulations. Going forward this guide assumes that you have directories set up exactly as described in the walkthrough linked above, especially that the folders `robot_app` and `simulation_app` exist inside the `src` folders of `robot_ws` and `simulation_ws`, respectively.

### Adding new scripts to `robot_ws`:

Python ROS node files should be stored in the scripts folder inside robot\_app. When you add a new node, you must also add it to the `CMakeLists.txt` inside `robot_app`. In the section labelled “install” you will see `scripts/rotate.py`, below that line is where you should list all file names that you add \(with scripts/ preceding the name\).

### Modifying the Simulation:

When creating your directories, two files were put inside `simulation_app/launch`: `example.launch` and `spawn_turtlebot.launch`

* Inside `spawn_turtlebot.launch`, you will see a line that looks like this \(it should be on line 3\): `<arg name="model" default="$(optenv TURTLEBOT3_MODEL waffle_pi)" doc="model type [burger, waffle, waffle_pi]"/>` In this section: `default="$(optenv TURTLEBOT3_MODEL waffle_pi)"` you can replace `waffle_pi` with `burger` or `waffle` to change the model of turtlebot3 used in the simulation
* Inside `example.launch` you will see this line \(it should be on line 8\): `<arg name="world_name" value="$(find simulation_app)/worlds/example.world"/>` You can replace `example.world` with the name of another world file to change the world used for the simulation. Note that the world file must be present in the folder `simulation_app/worlds`. You can copy world files from the folder `catkin_ws/src/turtlebot3_simulations/turtlebot3_gazebo/worlds` \(which is on your personal machine if you have installed ROS Kinetic\)

## Building and Bundling your applications:

In order to use your applications in simulation, they must first be built and bundled to a format that RoboMaker likes to use. At the top of the IDE, click: RoboMaker Run → Add or Edit configurations, then click Colcon Build.

Give your configuration a name. I suggest it be  robot or  simulation&gt;. For working directory, select the path to either `robot_ws` or `simulation_ws` \(you will have to do this twice, once for each workspace\).

Do the same, but this time for Colcon Bundle.

You now have shortcuts to build and bundle your robot and simulation applications.

Next, in the configuration window that you have been using, select workflow to create a shortcut which will build and bundle both applications with one click. Give your workflow a name, then put your actions in order. It is important that the builds go before the bundles.

Once you’ve made your workflow, go to: RoboMaker Run → workflow → your workflow to build and bundle your applications. This will create a bundle folder inside both `robot_ws` and `simulation_ws`. Inside `bundle`, there is a file called `output.tar.gz`. You can rename it if you like, but remember where it is.

Finally, we will go back to the configuration window to configure a simulation launcher.

* Give it a name
* Give it a duration \(in seconds\)
* Select “fail” for failure behavior
* Select an IAM role - it doesn’t necessarily matter which one, but AWSServiceRoleForRoboMaker is recommended
* Skip to the robot application section
* Give it a name
* Bundle path is the path to `output.tar.gz` \(or whatever you renamed it\) inside `robot_ws/bundle`
* S3 bucket should be the bucket you created at the beginning of this guide
* Launch package name is `robot_app`
* Launch file is the launch file you wish to use

NOTE: the name of the robot application and the launch file should related in some way The simulation application section is much of the same, except everything that was “robot” should be replaced with “simulation.”

OPTIONAL: Once your simulation launch configuration has been saved, you can add it as the final action of the workflow you made earlier.

## Running a simulation

These are the steps that have been found to work when you want to run a simulation in RoboMaker. They are Kludgy, and perhaps a more elegant solution exists, but for now this is what has been found:

1. Make sure your applications have been built and bundled. Then from the IDE, go to RoboMaker Run → Simulation Launch → your simulation config. This will upload your application bundles to the S3 bucket you specified, then try to start the simulation. **IT WILL PROBABLY FAIL**. This is okay, the main goal of this step was to upload the bundles.
2. Go back to the RoboMaker Management Console, and in the left menu Select Simulations → Simulation Jobs, then click “Create simulation job”
3. Now we will configure the simulation job again:
   * Set the failure behavior to fail
   * For IAM role, select “create new role”, then give your role a name. Each simulation job will have its own IAM role, so make the name related to the simulation job.
   * Click next
   * For robot application, select the name you gave when you configured the simulation in the IDE. The launch package name and launch file will be the same too, but you must type those in manually.
   * Click next, the next page is for configuring the simulation application, the same rules apply here as the robot application
   * Click next, review the configuration then click create.
4. RoboMaker will begin preparing to launch your simulation, in a few minutes it will be ready and start automatically. Once it is running, you will be able to monitor it through gazebo, rqt, rviz and the terminal.
5. Be sure that once you are done with your simulation \(if it is before the duration expires\) to cancel the simulation job under the action menu in the upper right of the simulation job management screen.

