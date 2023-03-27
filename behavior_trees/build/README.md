# Building a Behavior Tree in JSON

The mr_bt project allows users to define behavior trees as JSON files. Each node in the behavior tree is a JSON dictionary object, for example:

```
{
    "name":"goal_pid",
    "type":"Multitasker",
    "children":[
        
        {
            "name":"Calculate angular velocity necessary to face towards the position",
            "type":"PathCorrection",
            "correction_var_name":"angular_goal_pid",
            "goal_value_var_name":"goal_rotation",
            "current_value_var_name":"rotation",
            "max_vel":1.5
          },
          {
            "name":"Calculate linear velocity to get to position",
            "type":"PathCorrection",
            "correction_var_name":"linear_goal_pid",
            "goal_value_var_name":"dist",
            "current_value_var_name":"dummy_current_value",
            "max_vel":0.1,
            "offset":0
          }
    ],
    "blackboard":{
        "angular_goal_pid":null,
        "linear_goal_pid":null,
        "dummy_current_value":0,
        "dist":null
    },
    "blackbox":true
}
```

## Folder structure
When building a new project create a new folder with the name of your project in `mr_bt/src/tree_jsons`.

Each behavior tree project must have a root JSON file named `root.json`. This file will define the root node of your behavior tree and must be unique in your project folder. For example if you want to make a new project named `my_new_bt_project`, your folder structure should look something like this:
```
mr_bt/src/tree_jsons/
  my_new_bt_project/
    root.json
    action1.json
    action2.json
```