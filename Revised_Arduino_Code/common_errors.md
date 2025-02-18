# Troubleshooting guide for uploading and editing the Revised Arduino Code

> This troubeshooting guide is for a windows computer.

## Step 1: Ensure you have git installed on your computer

If you aren't sure about whether you have it, hit the windows key, type in 'powershell' and press enter to open the powershell terminal. Then type in `git --version` to ask git what version you have installed. If you don't see something like `git version 2.47.1.windows.2` and instead see something like `the command git is not recognized as a cmdlet...` then you will need to [install it](https://git-scm.com/downloads).

## Step 2: Clone repository

You've probably already cloned this repository, and if you don't know what that is, then you can find plenty of guides on GitHub to describe how to do that. You need this repository so you can run the code and create your own branch of the code when you want to make changes. Please do not make changes to the main branch.

## Step 3: Install Required and Recommended VScode Extensions

In order to run this code, you must have the PlatformIO extension installed on your machine. Click the 'Extensions' toolbar on the left of the screen (Ctrl+Shift+x), and install 'PlatformIO'.

### Possible Terminal Errors

If you run into an error resembling `the command 'pio' was not recognized as a cmdlet...` then the pio executable is likely not added to your windows path. On Windows, this typically has a path like `C:\Users\<YourUsername>\.platformio\penv\Scripts` if you installed the pio extension using VS code.

We recommend finding this path via the file explorer or terminal and then adding that path to the system environment variables. > If you're using the file explorer, ensure that you have enabled the "view hidden files" setting.

### How to enable viewing "Hidden Items" in the windows file explorer

1. Open File Explorer by pressing Win + E or clicking the File Explorer icon in the taskbar.
2. Click on the "View" button in the top toolbar.
3. Select "Show" from the dropdown menu.
4. Click on "Hidden items" to enable viewing hidden files and folders.

### How to add a path to your Windows environment variables

1. Open the Start menu and search for "Environment Variables".
2. Click on "Edit the system environment variables".
3. In the System Properties window, click on the "Environment Variables" button.
4. Under "System variables", find the "Path" variable and click "Edit".
5. Click "New" and add the path mentioned above.
6. Click "OK" to close all windows.
