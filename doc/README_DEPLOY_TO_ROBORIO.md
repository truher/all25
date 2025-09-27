# Deploying to RoboRIO

## Deploying to the RoboRIO
Once you have your code compiling locally there is one more step before you start changing things: making sure you can deploy to a RoboRIO. 

1. Ask a mentor for a RoboRIO you can use and get their help to power it up. The green light should come on.
1. Connect the RoboRIO to your laptop with a USB cable, like these photos:
    1. ![](readme_img/usb_1.jpg)
    1. ![](readme_img/usb_2.jpg)
1. Click the "W" button (or go to the search) in VSCode and find `WPILIB: Deploy Robot Code`
    1. ![deploy](readme_img/wpilib_deploy.png)
1. Select `comp` (or the name of your study) and then watch the build happen in the terminal. You should get `BUILD SUCCESSFUL` and the RioLog should pop up on the right after the build. 
    1. ![rio_log](readme_img/rio_log.png)

You are (hopefully!) getting a bunch of errors because we just deployed "production" code to a test RoboRIO, so it can't find motors, devices, etc. That's good! Move forward to the next step, [controlling your first motor](README_2_MOTOR.md)