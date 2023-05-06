<img width="189129387" alt="Screen Shot 2023-05-05 at 2 04 40 AM" src="https://user-images.githubusercontent.com/89604161/236387225-d58a367c-4a97-4f7c-b819-a044ff157efd.png">

<h1 align=”center”> 🤖🤖 Guard Robot (by Liulu Yue, Rongzi Xie, and Karen Mai) 🤖🤖🤖 </h1> 



<img width="301923" alt="Screen Shot 2023-05-05 at 9 08 15 AM" src="https://user-images.githubusercontent.com/89604161/236465896-5bd5f586-b3f5-465c-9f20-6f35d838da74.png">

<p>
For our team's COSI119a Autonomous Robotics Term Project, we wanted to tackle a challenge that was not too easy or not hard. All of our members have only recently learned ROS, so to create a fully demo-able project in 8 weeks sounded impossible. We started off having a few ideas: walking robot that follows an owner, robot that plays tag, and a pet robot that accompanies one. There were a lot of blockers for those ideas from limitation of what we know, tech equipments, time constraints, and mentor's concern that it will not be able to. We had to address issues like: 

In the end, we decided to work on a robot that guards an object and stop an intruder from invading a specified region. We pivoted from having one robot to having multiple robots so that they are boxing out the object that we are protecting. We tested a lot of design decisions where we wanted to see which method would give us the desired result with the most simple, least error prone, and high perfomring result.  

There were a lot of learnings throughout this project, and while even during the last days we were working to deploy code and continue to test as there is so much that we want to add.  
</p>

<img width="1234546576" alt="Screen Shot 2023-05-05 at 9 10 19 AM" src="https://user-images.githubusercontent.com/89604161/236466349-af0ddfdf-cf64-44f7-a5fe-d863c0cf6812.png">

<p>
As we will be approaching this project with an agile scrum methodology, we decided to take the time in developing our user stories, so that at the end of the week during our sprint review we know if we are keeping us on track with our project. After taking a review of the assignment due dates for other assignments of the class, we thought that having our own group based deadlines on Friday allows us to make these assignments achievable on time. Below is a brief outline of our weekly goals.
</p>

Here is a ⏰ timeline ⏳ , we thought were were going to be able to follow:
- March 3: Finish the Movie Script That Explains Story of Waht we want to do and Proposal Submission
- March 10: Figure out if tf can be done in gazebo
- March 17: Run multiple real robots
- March 24: Detect the object
- March 31: Aligning to be protecting it - circle or square like formation
- April 7: Detecting if something is coming at it
- April 14: Making the robot move to protect at the direction that the person is coming at it from 
- April 21: Testing
- April 28: Testing
- May 3: Stop writing code and work on creating a poster
- May 10 Continuing to prepare for the presentation

Here is a timeline of what we actually did: 
- March 3: Finished all Drafting and Proposal - Submitted to get Feedback and Confirmation That it is Good To Go (Karen, LiuLu, Rongzi)
- March 10: Figure out if tf can be done in gazebo (Rongzi), Creating Github Repo (Karen), Drawing Diagrams (Liulu), Explore Gazebo Worlds - Get Assests and Understand Structure (Karen)
- March 17: Run multiple real robots on Gazebo (Karen), Created multi robot gazebo launch files (Karen), Wrote Code on Patroling Line (Liulu), Create box world (Rongzi), Make Behavior Diagram (Karen)
- March 24: Continue to write code on patroling the line for multirobot (Liulu), Explored fiducials to integrate (Karen), Made better Gazebo world (Rongzi)
- March 31: Aligning to be protecting it - circle or square like formation
- April 7: Detecting if something is coming at it
- April 14: Making the robot move to protect at the direction that the person is coming at it from 
- April 21: Testing
- April 28: Testing
- May 3: Stop writing code and work on creating a poster
- May 10 Continuing to prepare for the presentation


Before            |  After
:-------------------------:|:-------------------------:
<img width="866" alt="Screen Shot 2023-05-05 at 2 09 55 AM" src="https://user-images.githubusercontent.com/89604161/236387884-c02bf777-ca33-4b76-8e44-d82aee0286e1.png"> | <img width="876" alt="Screen Shot 2023-05-05 at 2 11 27 AM" src="https://user-images.githubusercontent.com/89604161/236388088-48d3e45f-4730-4905-a425-b9b0c724456b.png">

Here is another breakdown of the timeline: 
![7131683301088_ pic](https://user-images.githubusercontent.com/89604161/236645060-07185b9e-8c21-450b-be6f-c79ff4f87719.jpg)



<img width="1245" alt="Screen Shot 2023-05-05 at 9 11 38 AM" src="https://user-images.githubusercontent.com/89604161/236466638-6ff7e061-705e-4a79-927e-84642f39c9ef.png">


<h3 align="left-center ">
Preliminary ❓ Questions ❓ Answered 
</h3> 

<p>
To address blockers, we had to start asking ourselves questions and addressing issues like: 
</p>

  1. What are the major risky parts of your idea so you can test those out first?
  2. What robot(s) will you want to use?
  3. What props (blocks, walls, lights) will it need? 
  4. What extra capabilities, sensors, will your robot need? How you have verified that this is possibler?
  5. And many other questions.


Here was the original plan of what we wanted to display: 
Seeing object they want to protect             |  Going towards the object and boxing it out
:-------------------------:|:-------------------------:
<img width="869" alt="Screen Shot 2023-05-05 at 2 07 59 AM" src="https://user-images.githubusercontent.com/89604161/236387641-64ae5517-e94b-458e-96f5-45cf3b1a56e1.png">  |  <img width="871" alt="Screen Shot 2023-05-05 at 2 08 23 AM" src="https://user-images.githubusercontent.com/89604161/236387687-3db63abd-7a1f-4b99-bf9b-e0eb7919179b.png">

Blocking the intruder when one robot detects it           |  Notifies robot to go over to intruder
:-------------------------:|:-------------------------:
<img width="867" alt="Screen Shot 2023-05-05 at 2 08 38 AM" src="https://user-images.githubusercontent.com/89604161/236387702-5f45fd65-3292-486a-a432-63d087863e19.png"> | <img width="860" alt="Screen Shot 2023-05-05 at 2 08 55 AM" src="https://user-images.githubusercontent.com/89604161/236387738-02f57a87-d7fc-4e17-86da-de20153979b9.png">

Though things do not often go to plan as we did not realize that step 2 to step 3 was going to be so much harder as there was all these edge cases and blockers that we discovered (concurrency, network latency, camera specs). This is why we did more of this instead: 





<img width="14325490" alt="Screen Shot 2023-05-05 at 9 13 14 AM" src="https://user-images.githubusercontent.com/89604161/236467016-b3e1da86-f765-4b75-abd1-05ae222b173a.png">

There are many states to our project. There is even states withhin states that need to be broken down. Whenever we were off to coding, we had to make sure that we were going back to this table to check off all the marks about control of logic. We were more interested in getting all the states working, and so we did not use any of the packages of state management. Another reason why we did not proceed with using those state pacakges was because we needed to have multiple files as one python file represented one node so we were not able to be running multiple robot through one file. 

<img width="891283" alt="Screen Shot 2023-05-05 at 9 14 15 AM" src="https://user-images.githubusercontent.com/89604161/236467210-25347e43-1d0e-406f-afaa-81e3bd5d835e.png">


<img width="234546" alt="Screen Shot 2023-05-05 at 9 14 42 AM" src="https://user-images.githubusercontent.com/89604161/236467313-4e6ecde6-ea64-45af-92e0-b5d12fba5a1d.png">
<img width="201" alt="Screen Shot 2023-05-05 at 1 59 55 AM" src="https://user-images.githubusercontent.com/89604161/236386699-5cd0786c-072b-4923-a3b7-9c81ec59f60d.png">

<img width="201" alt="Screen Shot 2023-05-05 at 2 00 16 AM" src="https://user-images.githubusercontent.com/89604161/236386744-9d3376ba-ccf9-47d9-aa38-146439e3abc5.png">

<img width="201" alt="Screen Shot 2023-05-05 at 2 01 08 AM" src="https://user-images.githubusercontent.com/89604161/236386843-1e1213ea-7d07-4572-9e42-21d70f66b5e5.png">

<img width="201" alt="Screen Shot 2023-05-05 at 2 01 36 AM" src="https://user-images.githubusercontent.com/89604161/236386897-4cd7db0a-76ce-4aaf-807e-4d2b731fed24.png">


<img width="2465735" alt="Screen Shot 2023-05-05 at 9 15 16 AM" src="https://user-images.githubusercontent.com/89604161/236467419-9112d8db-08a1-473c-903f-05a96d491371.png">

<img width="235465" alt="Screen Shot 2023-05-05 at 9 15 51 AM" src="https://user-images.githubusercontent.com/89604161/236467533-66784370-ce6c-409d-b3ca-cbd7d5053dd6.png">

<img width="1245" alt="Screen Shot 2023-05-05 at 9 20 33 AM" src="https://user-images.githubusercontent.com/89604161/236468664-10761f2c-95e1-4046-827b-c442a57fad98.png">
A new message is created for our robots to communicate about intruder detection. The message contains four boolean values each associated with one of our guardbot. When a guardbot detects the intruder, a new message will be created with that associated boolean value set to True, and the new message will be published to a topic called see_intruder. Each robot also has a subscriber that subscribes to this topic and a callback function that will check the passed-in message and get the information about which robot is seeing the intruder.  

The CMakeLists.txt and package.xml are also modified to recognize this newly created message.


<img width="2463547" alt="Screen Shot 2023-05-05 at 9 16 50 AM" src="https://user-images.githubusercontent.com/89604161/236467763-ea3e7515-7f1e-4810-a378-e259e5067946.png">

<img width="23546" alt="Screen Shot 2023-05-05 at 9 17 26 AM" src="https://user-images.githubusercontent.com/89604161/236467866-d5f71598-5fda-4141-a3c3-5e1d50989107.png">



👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻👩‍💻
