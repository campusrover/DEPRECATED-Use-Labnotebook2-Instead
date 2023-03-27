
# Working with Tree References

Due to the recursively defined nature of the behavior tree, the JSON definitions can get messy and unreadable for larger trees. The solution for splitting up different parts of your tree is using a reference to another JSON within your workspace instead of an entire node or tree definition. 

Anywhere you could include a node or tree definition, you could instead reference another JSON using the `"ref"` keyword. The root directory of the tree parser is set to `mr_bt/src/tree_jsons/`, so your reference provided must be relative to that root directory. 

For example, let's say that you are creating a new behavior tree in the folder `mr_bt/src/tree_jsons/my_new_tree/` and your folder contains the following:
```
/mr_bt/src/tree_jsons/my_new_tree/
	root.json
	move.json
	calculate_dist.json
```
You want to include `move.json` and `calculate_dist.json` as children for a parent node in `root.json`. This is what that would look like
```
{
    "name":"my_new_tree_root",
    "type":"Sequencer",
    "children":[
        {
            "ref":"my_new_tree/calculate_dist.json"
        },
        {
            "ref":"my_new_tree/move.json"
        }
    ]
}
```
You can also make references to other behavior tree JSONs as long as they are in the directory `mr_bt/src/tree_jsons` by providing their path within that directory.
