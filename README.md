# KinectPython
![Open Issues](https://img.shields.io/github/issues/KonstantinosAng/KinectPython.svg)(https://github.com/KonstantinosAng/KinectPython/issues)

## Table of Contents
1. [Description](##Description))
2. [Installation](##Installation)

***
## Description

## Installation

#### Python Environment Installation

The constructed architecture works only on Windows and is tested with Python 3.6. 
First create a fresh conda virtual environment with anaconda that uses Python 3.6 with
the following steps:

1. Download and Install Anaconda for Windows [using this link](https://www.anaconda.com/products/individual#windows).

2. Create a new virtual env with Python 3.6. Open the Anaconda Prompt and type the following command.

    ```
    conda create -n name_of_your_environment python=3.6
    ```

3. Activate the constructed environment.
    ```
    conda activate name_of_your_environment
    ```
4. Install all **requirements** from requirements.txt using the following command.
    ```
    pip install -r requirements.txt
    ```
5. Download all files using git clone or the .zip option and place them all in a 
folder wherever you want.

6. Open the Anaconda Prompt and type the following commands to find the directory
of the installed python in the conda environment.
    ```
    conda activate name_of_your_environment
    
    where python
    ```
7. Navigate to the Python's displayed directory, for example
   ```
   C:\Users\UserName\.conda\envs\name_of_your_environment
   ```
8. Navigate inside the pykinect2 installed Library of the Python.
    ```
    C:\Users\UserName\.conda\envs\name_of_your_environment\Lib\site-packages\pykinect2
    ```
9. Replace all the files inside the pykinect2 installed Library with the files located in the
pykinect2_original folder inside the repository's downloaded files.

10. Add Python Directory to the systems Environment Variables PATH.
    - Search for _Edit the system environment variables_.
    - From the Advanced Tab click on the environment variables.
    - From the System variables scroll down, select Path and click on Edit.
    - By clicking on New add the following paths.
        ```
        C:\Users\UserName\.conda\envs\name_of_your_environment
        C:\Users\UserName\.conda\envs\name_of_your_environment\python.exe
        C:\Users\UserName\.conda\envs\name_of_your_environment\Library\bin
        C:\Users\UserName\.conda\envs\name_of_your_environment\Scripts
        ```
 
#### RoboDK Installation 

1. Download and Install [RoboDK](https://robodk.com/download).
