This troubeshooting guide is for a windows computer.

Step 1: Ensure you have git installed on your computer. If you aren't sure about whether you have it, hit the windows key, type in 'powershell' and press enter to open the powershell terminal. Then type in 'git --version' to ask git what version you have installed. If you don't see something like 'git version 2.47.1.windows.2' and instead see something like 'the command git is not recognized as a cmdlet...' then you will need to install it.

Step 2: You've probably already cloned this repository, and if you don't know what that is, then you can find plenty of guides on GitHub to describe how to do that. You need this repository so you can run the code and create your own branch of the code when you want to make changes. Please do not make changes to the main branch.

Step 3: In order to run this code, you must have the PlatformIO extension installed on your machine. Click the 'Extensions' toolbar on the left of the screen, and install 'PlatformIO'. 

If you run into an error resembling "the command 'pio' was not recognized as a cmdlet..." then the pio executable is likely not added to your windows path. On windows, this typically has a path like 'C:\Users\<YourUsername>\.platformio\penv\Scripts' if you installed the pio extension using VS code. 

We recommend finding this path via the file explorer or terminal and then adding that path to the system environment variables.

To add this path to your Windows environment variables:

- Open the Start menu and search for "Environment Variables"
- Click on "Edit the system environment variables"
- In the System Properties window, click on the "Environment Variables" button
- Under "System variables", find the "Path" variable and click "Edit"
- Click "New" and add the path mentioned above
- Click "OK" to close all windows