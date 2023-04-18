# Creating a custom Conditional node

To create a custom conditional node, you must define your node class by inheriting from the superclass that defines a generic conditional node.

You also must include the `condition` function as the "tick" function that returns a `bool` type depending on the state of the relavent blackboard

Here is an example of a custom conditional node:

```
from ...nodes.conditional import Conditional

class MyConditionalNode(Conditional):
    def __init__(self):
        super().__init__()
    
    def condition(self, blackboard: dict) -> bool:
        ...
```

This type of node will return a boolean value.