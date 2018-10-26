# Creating a New Flask App

###### Create the project's directory
`mkdir MyNewFlaskApp`  
`cd MyNewFlaskApp`

###### Install Flask
`pip install Flask`

###### Create & activate the project's virtual environment
`python3 -m venv venv`  
`. venv/bin/activate`

###### Create the directory & file that will hold the app's main Python code
`mkdir flaskr`  
`touch __init__.py`  

* Adding an `__init__.py` file to a directory tells Python to treat it as a *package* rather than a *module*. Though it can be left empty, it usually contains code used to initialize a package. We can use it to house our main application code.

###### Basic layout in Python file (e.g. `__init__.py`)

	import flask from Flask
	
	def create_app( test_config=None ): 
		app = Flask( __name__, instance_relative_config=True )
		...
		return app
* See full tutorial [here](http://flask.pocoo.org/docs/1.0/tutorial/factory/).

###### Set environment variables & run the application\

	export FLASK_APP=flaskr
	export FLASK_ENV=development
	flask run

* Make these are run from the project's directory, *not* within the flaskr directory.

---
# About Flask

* A Flask app is an instance of the Flask class. However, instead of using just one, global Flask object, a Flask app is best implemented by defining a function (e.g. `create_app()`) that creates and returns an instance of the Flask class whenever called. This function is often referred to as the "Application Factory." See the [Flask tutorial](http://flask.pocoo.org/docs/1.0/tutorial/factory/) for further details.

* Note that the `create_app()` function takes the name of a configuration file, which contains the names and values of environmental variables to be used by the Flask application.

---
# About Virtual Environments

* Using a python virtual environment in a project is a way to ensure all of project's dependencies (e.g. python version, python packages, etc.) "accompany" it and are met wherever it happens to be run.

* [Here's](http://flask.pocoo.org/docs/1.0/installation/) the Flask documentation on virtual environments.

* To add a virtual environment to a project, cd into the project's directory and run `python3 -m venv venv`

* Whenever you work on your project, activate its virtual environment first, by running `. venv/bin/activate`

---
# About SQLite

* SQLite is a serverless relational database. Simply put, it allows you to implement a database in your project without having to run/connect to a separate database server.

* It's intended for light use, ideal for a development environment (or in production with light activity).

* Python also has built-in support for SQLite3, so there's no need to install it. Adding it to a project is a simple as `import sqlite3`. Further Flask-specific documentation is available [here](http://flask.pocoo.org/docs/1.0/tutorial/database/).

* A tutorial on using Flask with SQLAlchemy to interface with a SQLite database in a more object-oriented way can be found [here](https://www.youtube.com/embed/cYWiDiIUxQc).

---
# Adding a UI

###### Bootstrap

* Bootstrap provides a large [selection of pre-made UI components](https://getbootstrap.com/docs/4.1/components/alerts/). 

* To enable using Bootstrap via CDN: 
	1. Paste this into your HTML document's header, *before* any other links to css: 
`<link rel="stylesheet" href="https://stackpath.bootstrapcdn.com/bootstrap/4.1.3/css/bootstrap.min.css" integrity="sha384-MCw98/SFnGE8fJT3GXwEOngsV7Zt27NXFoaoApmYm81iuXoPkFOJwJ8ERdknLPMO" crossorigin="anonymous">` 

	2. Paste these *in order* near the very of your HTML document, right bofore the `</body>` closing tag: 

```	
	<script src="https://code.jquery.com/jquery-3.3.1.slim.min.js" integrity="sha384-q8i/X+965DzO0rT7abK41JStQIAqVgRVzpbzo5smXKp4YfRvH+8abtTE1Pi6jizo" crossorigin="anonymous"></script>
	<script src="https://cdnjs.cloudflare.com/ajax/libs/popper.js/1.14.3/umd/popper.min.js" integrity="sha384-ZMP7rVo3mIykV+2+9J3UJ46jBk0WLaUAdn689aCwoqbBJiSnjAK/l8WvCWPIPm49" crossorigin="anonymous"></script>
	<script src="https://stackpath.bootstrapcdn.com/bootstrap/4.1.3/js/bootstrap.min.js" integrity="sha384-ChfqqxuZUCnJSK3+MXmPNIyE6ZbWh2IMqE241rYiqJxyMiZ6OW/JmZQ5stwEULTy" crossorigin="anonymous"></script>
```

* The full set-up instructions can also be found [here](https://getbootstrap.com/docs/4.1/getting-started/introduction/).

* Most Bootstrap elements are added by creating a `<div>` with the `class` attribute set to one of Bootstrap's predefined classes. For example, adding a Boostrap alert element consists of the following: 
`<div class="alert alert-primary" role="alert">A simple primary alert—check it out!</div>`
	* Bootstrap uses the `role` attribute to ensure [accessability](https://v4-alpha.getbootstrap.com/getting-started/accessibility/).

###### Icons

* To enable use of free FontAwesome icons via CDN, add the following link tag to the header of your HTML document: 
`<link rel="stylesheet" href="https://use.fontawesome.com/releases/v5.4.1/css/all.css" integrity="sha384-5sAR7xN1Nv6T6+dT2mhtzEpVJvfS3NScPQTrOxhwjIuvcA67KV2R5Jz6kr4abQsz" crossorigin="anonymous">`

* To add a specific icon, pick the one you want from the [FontAwesome gallery](https://fontawesome.com/icons?d=gallery), then simply copy its html tag (e.g. `<i class="fas fa-arrow-alt-circle-up"></i>`) and paste it into the desired section of your HTML document.
