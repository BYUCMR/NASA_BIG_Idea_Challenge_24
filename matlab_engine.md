# Installing matlab engine

This process can be difficult and filled with frustration if not done properly. Hopefully this page will alleviate some stress and direct you down the smoothes path to instalation.

## DO NOT PIP INSTALL ANYTHING

Some of the main issues I have faced while installing is python expecting the files to be in a location becasue I installed other matlab libraries like "matlab" or "matlabengine"\
These will not work.

1. Verify your python version. Matlab engine only works with python version 3.8, 3.9, and 3.10.\
Check your version by opening up a terminal window and typing

``` {bash}
python --version
```

2. Navigate to your matlab directory on your computer. 
You can find this by opening matlab and in the script enter

``` {matlab}
matlabroot
```

This will most likely be in\
Windows : C:\Program Files\MATLAB or C:\Users\"your user"\AppData\MATLAB\
mac/linux : user/Applications/MATLAB_'version'

3. Open up a terminal and go to your matlab root\
navigate to the python engine\
matlabroot\extern\engine\python\

Once in this directory run the command

```{bash}
python setup.py install
```

You should see some installation text and then a success message.

4. Verify installation\
In the same terminal window run

```{bash}
pip list
```

You should see "matlabengineforpython" followed by its version number\
Next test is to run a python script using the engine.

```{python}
eng = matlab.engine.start_matlab()

# Call a simple MATLAB function
result = eng.sqrt(4.0)
print(result)  # Output should be 2.0

# Stop the MATLAB engine
eng.quit()
```
