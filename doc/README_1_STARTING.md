# Setting up a VSCode Development Environment
This guide will walk you through how to set up your computer to make code changes and push them to the RoboRio. You should make sure all of these instructions work before you spend a lot of time changing any code to know that your end-to-end testing loop with a robot will work.

## Get Set Up With Git
Git can be a little hard.
<img src="readme_img/git_comic.webp">

Throughout these docs we will sometimes tell you to run a Git command. It's better if you understand what it does, but it's not the end of the world if you don't. You should probably go through most of these steps to get somewhat familiar with the basic Git commands, then take the Coding at Team100 Class (TODO!).

Let's start by getting the `Team100/all25` Git repo checked out.

## Setting Up the Team100/all25 Repository with GitHub Desktop

### Prerequisites
Before we start, make sure you have:
- A computer with internet access
- A GitHub account (if you don't have one, go to [github.com](https://github.com) and sign up)
- Basic familiarity with using a computer (opening programs, clicking buttons, etc.)

### Step 1: Install GitHub Desktop
GitHub Desktop is a user-friendly way to work with Git repositories. It's much easier than using command line Git!

**On Windows:**
1. Go to [desktop.github.com](https://desktop.github.com)
2. Click "Download for Windows"
3. Run the installer and follow the prompts
4. GitHub Desktop will open automatically after installation

**On Mac:**
1. Go to [desktop.github.com](https://desktop.github.com)
2. Click "Download for Mac"
3. Open the downloaded file and drag GitHub Desktop to your Applications folder
4. Open GitHub Desktop from your Applications folder

### Step 2: Sign In to GitHub Desktop
1. Open GitHub Desktop
2. Click "Sign in to GitHub.com"
3. Enter your GitHub username and password
4. GitHub Desktop will ask for permission - click "Authorize desktop"
5. You should now see the GitHub Desktop main screen

### Step 3: Fork the Team100/all25 Repository
A "fork" creates your own copy of the repository that you can modify without affecting the original.

1. Open your web browser and go to [github.com/Team100/all25](https://github.com/Team100/all25)
2. Click the "Fork" button in the top-right corner
3. GitHub will ask where to fork it - choose your personal account
4. Wait for the fork to complete (this might take a minute)
5. You now have your own copy at `github.com/YourUsername/all25`

### Step 4: Clone Your Fork to Your Computer
Now we'll download your forked copy to your computer.

1. In GitHub Desktop, click "Clone a repository from the Internet"
2. Click the "GitHub.com" tab
3. Find "all25" in the list (it should show "YourUsername/all25")
4. Click on it to select it
5. Choose where to save it on your computer:
   - **Recommended location:** `C:\Users\YourUsername\Documents\GitHub\` (Windows) or `/Users/YourUsername/Documents/GitHub/` (Mac)
   - GitHub Desktop will create the folder automatically
6. Click "Clone"
7. Wait for the download to complete (this might take a few minutes)

### Step 5: Verify Everything Worked
Let's make sure everything downloaded correctly:

1. In GitHub Desktop, you should see "all25" in your repository list
2. Click on "all25" to open it
3. You should see the repository contents
4. Open File Explorer (Windows) or Finder (Mac) and navigate to where you saved the repository
5. You should see folders like `comp`, `lib`, `studies`, `console`, `raspberry_pi`, and `doc`

### Step 6: Understanding What You Just Downloaded
You now have your own copy of Team100's 2025 robot code! Here's what each folder contains:

- **`comp/`** - The main competition robot code
- **`lib/`** - Shared library code used by all projects
- **`studies/`** - Small experimental projects and learning exercises (this is where you'll work!)
- **`console/`** - Operator console hardware and software
- **`raspberry_pi/`** - Vision processing and other coprocessor tasks
- **`doc/`** - Documentation (like this file!)

### Step 7: Set Up Your Development Branch
Before you start making changes, it's good practice to create your own branch (a separate version of the code where you can make changes without affecting the main code):

1. In GitHub Desktop, make sure you're on the "main" branch (you should see this at the top)
2. Click "Current branch: main" and then "New branch"
3. Name your branch something like "your-name-dev" (replace with your actual name)
4. Click "Create branch"
5. You should now see "Current branch: your-name-dev" at the top

### Step 8: Understanding the Fork Workflow
Here's how you'll work with your code:

1. **Make changes** in your local repository
2. **Commit changes** using GitHub Desktop (we'll show you how later)
3. **Push changes** to your fork on GitHub
4. **Create a Pull Request** to contribute your changes back to Team100/all25

### Troubleshooting Common Issues

**"Repository not found"**
- Make sure you forked the repository first (Step 3)
- Check that you're signed in to GitHub Desktop
- Verify you're looking at the right repository

**"Clone failed"**
- Check your internet connection
- Make sure you have permission to access the repository
- Try closing and reopening GitHub Desktop

**"Can't find the repository"**
- Make sure you forked it to your personal account
- Check that you're signed in with the correct GitHub account

**"Permission denied"**
- Make sure you're signed in to GitHub Desktop
- Verify you have write access to the folder where you're trying to save

### What's Next?
Great job! You now have your own copy of the Team100/all25 repository on your computer. The next step is to set up VSCode and WPILib so you can actually work with the code.

## Get VSCode + WPILib Set Up
TODO

1. You might need to install [Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack) manually to get autocomplete etc?


## Getting VSCode Workspace Working
Now that you have the repository on your computer, let's set up VSCode to work with it.

1. Open VSCode
2. Go to `File > Open Folder`
3. Navigate to where you saved your repository (probably `Documents/GitHub/all25/`)
4. Select the `comp` folder inside your repository
5. Click "Select Folder"

When you open the comp project, VS Code should show a message in the bottom-right corner suggesting that you open the "workspace" instead, which includes lib:

<img src="readme_img/openworkspace.png" width=350/>

**Click "Open Workspace"** - this will give you access to both the competition code and the shared library code.

When you have VS Code correctly set up, the Explorer view should contain two top-level folders: "comp" and "lib":

<img src="readme_img/comp_workspace.png" width=350/>

**Important:** You're now working in your own fork of the repository, so any changes you make won't affect the main Team100/all25 repository until you create a Pull Request.

## Building the RoboRIO Code
Use the WPI extensions to run the `gradlew` build:
1. Press `CTRL + SHIFT + P` to get the VSCode Command Mode.
    1. `CMD + SHIFT + P` on a Mac
1. Search for `Build Robot Code`
1. Select `comp` when it asks.
1. A terminal should pop up that runs the build. If successful, you will see this:
    ```
    BUILD SUCCESSFUL in 2s``
    6 actionable tasks: 1 executed, 5 up-to-date
    Watched directory hierarchies: [C:\Users\Engineering Student\src\all24\comp\swerve100]
    ```

TODO: what if the build is not successful?

# Next Steps
You are done setting up your code! It builds successfully so you could start making changes now, if you'd like. If you have a RoboRIO handy, you might want to try to [deploy](README_DEPLOY_TO_ROBORIO.md) to it immediately. But before you do, I recommend learning a bit more about how to work with the FIRST Robotics interfaces, WPILib. There are two paths you can take:

1. [Learn to control a single motor](README_2_MOTOR.md): recommended, this walks you step-by-step on how to get a single motor running through the RoboRIO.
2. [Learn to use the Simulator](README_5_SIMULATOR.md): alternate path, learn to deploy the code to a simulator and do your testing that way.
