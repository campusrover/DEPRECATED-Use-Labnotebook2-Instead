# Nodes
A behavior tree is a tree-shaped data structure consisting of nodes, each of which contain a logical method which executes code. The behavor tree is evaluated recursively starting 
at the root node. Each node has the abilility to execute code which will either run a script, or execute all of its children. Each node will also return one of 3 outputs to its 
parent: "success", "failure", or "running". There are two main types of nodes: the control-flow (parent nodes) nodes and the leaf nodes.
