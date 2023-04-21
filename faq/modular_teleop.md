# Use teleop in your code
Chris Minkwon Choi

## Introduction

Among multiple ways to move a robot, teleop is one of the most intuitive methods. In your project, you can add teleop feature easily for various purposes. For example, this code is used in the Robotag Project. Robotag Project is a game where robots play game of tag. In Robotag Project, this code is used to let the user take over the control and play as either cop or rubber. 

## How to use
The control is very intuitive, and there is short instruction built in to the system. w,a,d,x is for each directions, and s stops the robot. You can also play around with the original teleop and familiarize with the control using the original teleop.

## What to add

Add the following code to the main class (not in a loop or if statement). These declares constants and mothods for teleop. 

``` python 
BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82
LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your TurtleBot3!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Burger : ~ 0.22, Waffle and Waffle Pi : ~ 0.26)
a/d : increase/decrease angular velocity (Burger : ~ 2.84, Waffle and Waffle Pi : ~ 1.82)

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():  
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input
    return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def checkLinearLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return vel

def checkAngularLimitVelocity(vel):
    vel = constrain(vel, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return vel

```

Then when you want to use teleop, add following code. These are the actual moving parts. Change 'use-teleop' to any other state name you need. If you want teleop to work in all times, take this code outside the if statement. 


```python
if state=='use-teleop':
    if rospy.Time.now().to_sec()-time_switch.to_sec()>10:
        inc_x = posex2 -posex1
        inc_y = posey2 -posey1
        angle_to_goal = atan2(inc_y, inc_x)
        z=math.sqrt((inc_x*inc_x)+(inc_y*inc_y))
        if z > .05:
            if z < .3:
                twist.linear.x=0
                twist.angular.z=0
                state="cop"
                time_switch=rospy.Time.now()



    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    #rospy.init_node('turtlebot3_teleop')
    cmd_vel_msg = '/cmd_vel'
    cmd_vel_pub = rospy.Publisher(cmd_vel_msg, Twist, queue_size=10)
    turtlebot3_model = rospy.get_param("model", "burger")
    cmd_vel_pub.publish(twist)
    inc_x = posex2 -posex1
    inc_y = posey2 -posey1

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if key == 'w' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'x' :
                target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'a' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == 'd' :
                target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
                status = status + 1
                print(vels(target_linear_vel,target_angular_vel))
            elif key == ' ' or key == 's' :
                target_linear_vel   = 0.0
                control_linear_vel  = 0.0
                target_angular_vel  = 0.0
                control_angular_vel = 0.0
                print(vels(target_linear_vel, target_angular_vel))
            elif key == 'r':
                state = 'robber'
                break
            else:
                if (key == '\x03'):
                    break

            if status == 20 :
                print(msg)
                status = 0

            twist = Twist()

            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

            cmd_vel_pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        cmd_vel_pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
```

If you want to enter this 'use-teleop' using 'z' key,

```python
if os.name != 'nt':
    settings = termios.tcgetattr(sys.stdin)
key = getKey()
if key == 'z': #h for human
    state = 'use-teleop'
```


## Customization

This code can be edited for customization. Edit the 'msg' String at the top of the code for GUI, and edit constants for maximum speed. 

