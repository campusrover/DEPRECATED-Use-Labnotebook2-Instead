# Behavior Trees (in progress)

## Author: Chris Tam

### Introduction

The campus rover behavior tree project was conceived of as a potential avenue of exploration for campus rover’s planning framework. Behavior trees are a relatively new development in robotics, having been adapted from non-player character AI in the video game industry around 2012. They offer an alternative to finite state machine control that is both straightforward and highly extensible, utilizing a parent-child hierarchy of actions and control nodes to perform complex actions. The base control nodes - sequence, selector, and parallel - are used to dictate the overarching logic and runtime order of action nodes and other behavior subtrees (Collendachise). Because of their modular nature, behavior trees can be combined and procedurally generated, even used as evolving species for genetic algorithms (Collendachise). Numerous graphical user interfaces make behavior trees yet more accessible to the practicing technician (Game assets, other paper). My aim for this project, then, was to implement these trees to be fully integrable with Python and ROS, discover the most user-friendly way to interact with them for future generations of campus rover, and integrate my library with the current campus rover codebase at either a surface level (communicating with web server and sequencing navigation) or a deep level (replacing the finite state machine).

