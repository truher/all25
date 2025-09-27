# Setting up a VSCode Development Environment
This guide will walk you through how to set up your computer to make code changes and push them to the RoboRio. You should make sure all of these instructions work before you spend a lot of time changing any code to know that your end-to-end testing loop with a robot will work.

## Get Set Up With Git
Git can be a little hard.
<img src="readme_img/git_comic.webp">

Throughout these docs we will sometimes tell you to run a Git command. It's better if you understand what it does, but it's not the end of the world if you don't. You should probably go through most of these steps to get somewhat familiar with the basic Git commands, then take the Coding at Team100 Class (TODO!).

Let's start by getting the `Team100/all25` Git repo checked out.

## Setting Up the Team100/all25 Repository

### Prerequisites
Before we start, make sure you have:
- A computer with internet access
- A GitHub account (if you don't have one, go to [github.com](https://github.com) and sign up)
- Basic familiarity with using a computer (opening programs, typing commands, etc.)

### Step 1: Install Git
Git is a tool that helps us manage code versions and collaborate with others. You need to install it first.

**On Windows:**
1. Go to [git-scm.com](https://git-scm.com/download/win)
2. Download the installer
3. Run the installer and follow the prompts (you can use all the default settings)
4. Open Command Prompt or PowerShell to test: type `git --version` and press Enter
5. You should see something like `git version 2.x.x`

**On Mac:**
1. Open Terminal (press Cmd+Space, type "Terminal", press Enter)
2. Type `git --version` and press Enter
3. If Git is not installed, you'll be prompted to install it. Follow the instructions.
4. If Git is already installed, you'll see the version number

**On Linux:**
1. Open Terminal
2. Type `sudo apt install git` (Ubuntu/Debian) or `sudo yum install git` (CentOS/RHEL)
3. Press Enter and follow the prompts
4. Test with `git --version`

### Step 2: Configure Git (First Time Only)
You need to tell Git who you are. Replace "Your Name" and "your.email@example.com" with your actual information:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"
```

### Step 3: Choose Where to Put Your Code
Decide where you want to keep your robot code. We recommend creating a folder like `src` or `projects` in your home directory.

**On Windows:**
- Open File Explorer
- Navigate to `C:\Users\YourUsername\`
- Create a new folder called `src`

**On Mac/Linux:**
- Open Terminal
- Type `mkdir ~/src` and press Enter (this creates a folder called "src" in your home directory)

### Step 4: Clone the Repository
Now we'll download the Team100/all25 code to your computer.

1. Open Terminal/Command Prompt
2. Navigate to your src folder:
   - **Windows:** `cd C:\Users\YourUsername\src`
   - **Mac/Linux:** `cd ~/src`
3. Clone the repository:
   ```bash
   git clone https://github.com/Team100/all25.git
   ```
4. Wait for the download to complete (this might take a few minutes)
5. Navigate into the new folder:
   ```bash
   cd all25
   ```

### Step 5: Verify Everything Worked
Let's make sure everything downloaded correctly:

1. Go into your `src` or `project` folder where you have the code.
1. List the contents of the folder:
   ```bash
   ls
   ```
   (On Windows, use `dir` instead of `ls`)

2. You should see folders like `comp`, `lib`, `studies`, `console`, `raspberry_pi`, and `doc`

### Step 6: Understanding What You Just Downloaded
You now have a complete copy of Team100's 2025 robot code! Here's what each folder contains:

- **`comp/`** - The main competition robot code
- **`lib/`** - Shared library code used by all projects
- **`studies/`** - Small experimental projects and learning exercises
- **`console/`** - Operator console hardware and software
- **`raspberry_pi/`** - Vision processing and other coprocessor tasks
- **`doc/`** - Documentation (like this file!)

### Step 7: Set Up Your Development Branch (Optional but Recommended)
Before you start making changes, it's good practice to create your own branch (a separate version of the code where you can make changes without affecting the main code):

1. Create a new branch with your name:
   ```bash
   git checkout -b your-name-dev/your-project-name
   ```
   1. Replace "your-name-dev" with something like "alex-dev" or "sarah-dev"
   2. Replace "your-project-name" with what you are trying to accomplish. E.g. for now it can be `learning`

2. Verify you're on your new branch:
   ```bash
   git branch
   ```
   You should see an asterisk (*) next to your branch name

### Troubleshooting Common Issues

**"git: command not found"**
- Git isn't installed or isn't in your PATH. Reinstall Git and make sure to restart your terminal.

**"Permission denied"**
- You might not have permission to write to that directory. Try using a different folder or ask for help.

**"Repository not found"**
- Check that you typed the URL correctly: `https://github.com/Team100/all25.git`
- Make sure you have internet access

**"Authentication failed"**
- This usually happens when GitHub requires authentication. For now, you can work with the code locally. We'll cover authentication later.

### What's Next?
Great job! You now have the Team100/all25 repository on your computer. The next step is to set up VSCode and WPILib so you can actually work with the code.

## Get VSCode + WPILib Set Up
TODO

1. You might need to install [Java](https://marketplace.visualstudio.com/items?itemName=vscjava.vscode-java-pack) manually to get autocomplete etc?


## Getting VSCode Workspace Working
In VSCode, go to `File > Open Folder` and select the `all25/comp` folder. 

When you open the comp project, VS Code should show a message in the bottom-right corner suggesting that you open the "workspace" instead, which includes lib:

<img src="readme_img/openworkspace.png" width=350/>

When you have VS Code correctly set up, the Explorer view should contain two top-level folders: "comp" and "lib":

<img src="readme_img/comp_workspace.png" width=350/>
TODO.

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
You are done setting up your code! It builds successfully so you could start making changes now, if you'd like. But before you do, I recommend learning a bit more about how to work with the FIRST Robotics interfaces, WPILib. There are two paths you can take:

1. [Learn to control a single motor](README_2_MOTOR.md): recommended, this walks you step-by-step on how to get a single motor running through the RoboRIO.
2. [Learn to use the Simulator](README_5_SIMULATOR.md): alternate path, learn to deploy the code to a simulator and do your testing that way.
