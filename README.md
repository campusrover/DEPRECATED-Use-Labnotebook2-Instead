# Introduction

## To the students of Gen 4:

We started our iteration with the the code from Gen 2. This consisted of several nodes which were compatible with the Turtlebot2. We immediately eliminated some nodes which did not exist on the Mutant \(such as package\_handler\), as we did not have such a sensor. Other nodes required reworking, such as pickup\_detector, as the Turtlebot2 had sensors to see whether it was on the ground or not but the Mutant did not have this functionality built in. In addition, during our time in the class, namespacing was introduced, so multiple robots could run on the same roscore. This introduced some difficulties, specifically with certain nodes that were not being namespaced correctly.

It seems that many of the problems we were having resulted from the Turtlebot2 running off of a lot of proprietary software, while the Mutant \(based off of a Turtlebot3\) is much simpler. Using ROS, we did run into many hiccups and there was a significant amount of troubleshooting involved. A lot of information about the troubleshooting can be found in the lab notebook, on the Mutant page. As ROS is very complicated, without a doubt any future generation will also be faced with a significant amount of troubleshooting. We found the rqt\_graph tool to be one of the most helpful -- it allows the user to see which nodes are subscribing/publishing, etc. Much of the time problems stem from this, and being able to see this visually can be very helpful.

We participated in the development of the campus rover similar to Gen 2, where we would brainstorm projects and Pito would lead the adoption by each group of a given project. While everyone was \(for the most part\) able to complete their projects, we left integration of the different parts \(computer vision, hand gestures, navigation\) until the last day, which created some difficulties. We highly recommend teams to work on integration throughout the process so that integration is as smooth as possible for the demo at the end of the semester!

In future generations, we recommend further improving the robustness of features that were implemented, such as hand gestures and CV. In addition, part of Gen 3’s goal was to allow the project to run on any Turtlebot. While the code should be able to run on any Turtlebot3, a speaker of some sort would need to be attached for the talk services. These talk services allow the robot to update the user with its progress throughout its use. With these new features, we tried to give direction to further development of the project. We think that these features will guide future generations of the class in development of the campus rover, ultimately leading to a campus rover that will actually be able to deliver packages on campus.

Best of luck, Gen 3

### Eli Cohn, 5/17/2019

## To the students of Gen 3:

We, the Gen 2 team, hope in this letter \(and in the ancillary documentation contained in the `labnotebook` repo\) to pass on the experience and knowledge we have gathered over the past semester to the Gen 3 team and beyond. We hope that future teams will not only build upon our accomplishments, but also that they will learn from and avoid our mistakes.

We began Campus Rover as a first-come-first-serve brainstorming of projects and possible features of the final Campus Rover product, where students would create teams \(1-3 student sized teams for 6 total students\) and pick from the list of projects for that current iteration \(typically the week from the current class to the next week's meeting\). Often, a project would take longer than a single iteration, in which case a team would usually remain on the project for the next iteration.

While this system allowed us to initially take on virtually any project we liked, and gave us a great deal of control over the Campus Rover's final functionality, the protocol gave us expected headaches as we approached the final demo date. By the third or fourth to last iteration \(3-4 weeks before the final demo\), we were forced as a team to change gears towards combining and debugging the individual projects from our previous iterations. This involved the implementation of states \(`states.py` and `all_states.py`\), controller nodes \(`rover_controller.py`\), and other nodes solely for structurally organizational purposes. While these nodes perform satisfactorily for the extent of our Campus Rover \(demo and features-wise\), they could have put in much more work if implemented chronologically.

In future generations of the Campus Rover, we recommend starting where we finished. With a robust, scaleable node framework, implementing new features would not only be faster and easier, but the nodes will also be able to be implemented with customized standardized guidelines that will gurantee consistency throughout the architecture. For example, with the introduction of states, we had hoped to better organize the robot by its current objective \(i.e. navigating, teleoping, etc\). However, since many of our existing functionalities were already built, we ran into problems refactoring them to reflect and operate under states.

Implementing high level node control proved its worth again when developing the web application. In order for the application to provide rich feedback including the robot's position, navigation goal, etc, it must be able to access this information. With some exhaustive high-level state and node handling, this information would already be available throughout the architecture, and easily integrated into the web app \(see `Flask & ROS.md`\).

For an ideal Gen 3 implementation of Campus Rover, we think that full syncronization and managing of nodes is imperative. Most \(if not all\) robot functionality should come second to a standardized and organized framework. Consider sorting messages coming from sensors, the web app, and other nodes, so that any subscribing node can easily select and sort through incoming messages. Also, though our implementation was useful to us in the end, a deeper and more easily managed state mechanism would greatly help organization of the robot's features and tasks.

Good luck!

Gen 2

### _Ben Alberxt 12/16/18_

