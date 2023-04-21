
## Control-Flow Nodes

- Selector
  - The Selector executes its children sequentially from left to right. 
  - If one of its children returns either "success" or "running", it will halt execution of its children and it will return the result of the child it stopped on.
  - If all of its children return "failure", the Selector will also return "failure".
- Sequencer
  - The Sequencer executes its children sequentially from left to right.
  - The Sequencer will not halt execution of its children unless one of them returns "failure" or "running", in which case it will also return "failure" or "running".
  - If all children return "success" the Sequencer will return "success"
- Multitasker
  - The Multitasker runs all of its children concurrently, each in a separate thread.
  - The Multitasker will return "success" only if all of it's children return "success".
  - If any of its children return "running" but none return "failure", the Multitasker will return "running".
  - If any of its children return "failure", the Multitasker will return "failure".
