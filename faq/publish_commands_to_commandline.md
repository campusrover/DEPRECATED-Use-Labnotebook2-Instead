#### Adam Rogers

### How to publish commands to a commandline in Python

First, import the `subprocess` module. This module has functions that allow Python to interact with a commandline.

`import subprocess`

Use the Popen class from subprocess to publish commands. Keep in mind, these commands will be published to the terminal that you are currently running the python script in. The variable `args` should be a list that represents the command being published with each index being a word in the command separated by spaces. For example, if the command I wanted to run was `ls -l`, args would be `['ls', '-l']`.

`p = subprocess.Popen(args, stdout=subprocess.PIPE)`

In some cases, you will want to terminate certain commands that you have previously published. If so, the best way is to add each Popen object to a dictionary, and when you want to terminate a command, find it in the dictionary and call the `terminate()` function on that object. For the dictionary, I suggest using the command as a String separated by spaces for the key and the Popen object as the value. For example:

```
p = command_dictionary['ls -l']
p.terminate()
```
