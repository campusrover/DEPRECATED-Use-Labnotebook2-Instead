 #!/bin/bash
#Jacob smith COSI 119A 11/7/2019 ASrm Interface

#update Topic Subscriber on robot
echo "updating Local File on Robot"
scp topic_subscriber.py mutant@mutant.dyn.brandeis.edu:Documents/topic_subscriber.py

#run topic echo node to print serial output
#run publisher node with a delay so robot can start up
gnome-terminal -e 'sh -c "echo \"This Window will publish output from Arm, please look at main window for now\";sleep 40;rostopic list;sleep 5;rostopic echo /armresponse"'

#run publisher node with a delay so robot can start up
gnome-terminal -e 'sh -c "echo \"This Window will publish input to Arm, please look at main window for now\";sleep 40;cd ~/catkin_ws/src/arminterface/scripts;python topic_publisher.py"'
#start robot and run subscriber node
ssh mutant@mutant.dyn.brandeis.edu 'bash' < '/home/robot/catkin_ws/src/arminterface/scripts/mutantStartup.sh'