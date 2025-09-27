TODO: we should have a whole README on how debugging works
with robots! It's particularly hard because of timing issues etc. What are the best practices?

1. Make sure you can debug your code in simulation.
    1. If you aren't familiar with debugging in VSCode, read this: TODO.
    1. Set a debug breakpoint on:
        ```java
        public void robotPeriodic() 
        ```
    1. The simulation should trip your debugger since it calls this function every few ms. Then you are ready to go!