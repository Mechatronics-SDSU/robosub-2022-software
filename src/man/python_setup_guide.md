## Python Setup Guide
##### by Ian Reichard

### Introduction
This guide assumes you know either a bit of programming basics and have taken/are taking CS 107. You need not know any Python before using this guide, but you need to read through until we reach interactive mode if you know Python to know what our setup is.

Python is a dynamically-typed general-purpose programming language built on C. It abstracts much of the memory management in C and as a result lacks the direct memory management C has. It trades this for code readability and ease of development, with quicker development times than other programmming languages. Python thus has many features and can be written for different paradigms and supports features from Objects to lambda functions. Python is thread locked with the GIL, and runs in only 1 thread.

We use Python primarily in Mechatronics for a variety of reasons such as faster development time and library support, but are not dedicated to only using Python for code. It's helpful to think of programming languages as tools, where some can be used for many applications, but some can be more specific.

### Pycharm
Pycharm is an IDE (Integrated Development Environment) for Windows, MacOS, and Linux that helps programmers write Python code. It is not required that you install Pycharm to write code for Mechatronics, but is highly recommended by us for new Python programmers, especially if you don't want to work with the command line.

[You can download Pycharm here](https://www.jetbrains.com/pycharm/download/#section=windows)

Follow the setup instructions and install using the latest Python version if prompted. Install virtual enviornment or venv as well if prompted.

### Without Pycharm
You can also install your IDE of choice, but you may have to look around online for instructions on how to use features like venv and VCS. If you want to download Python and use an editor like notepad++, vim, or emacs, you can download python [from python.org](https://www.python.org/downloads/) or use your package manager of choice to install python. When you're done installing Python, be sure to also [install pip](https://pip.pypa.io/en/stable/installation/) with get-pip.py or using your package manager of choice.

### Virtual Environments

We recommend using a separate virtual environment for each project. A common use of Python is to import libraries with [pip](https://pip.pypa.io/en/stable/#). 

If you installed Pycharm, it comes with this functionality already when you make a new project.  Go to File > New Project, then select "new environment using Virtualenv". This is like a separate python install where your libraries can be isolated between projects.

If you didn't install Pycharm, virtual environment is included in the standard Python library, but you have to do a bit more work, check out the [python documentation](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/#creating-a-virtual-environment) for how to activate and use a virtual environment for the command line on your system.

### (Optional) Conda

As an alternative to standard virtual environments, you can also use Conda to manage different versions of python. Conda is my perffered method of separate environments and is supported in Pycharm. [You can download it here.](https://anaconda.org/anaconda/conda)

You're done!
