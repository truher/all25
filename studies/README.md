# studies
This should contain many small projects, each a "study" of some topic.

# Using lib in your study

To use the lib code in your study, you need to do four things:

1. In VSCode, click File::Add Folder to Workspace... and then navigate to the `lib` directory and click "Add."
   This will add the lib folder in the VSCode "Explorer" pane.
2. Copy the contents of `lib/vendordeps` to the `vendordeps` folder of your study.  To do this, highlight
   the lib/vendordeps files (not the directory) in the Explorer and use ctrl-C.  Then highlight the vendordeps
   in your study and use ctrl-V.
3. Edit `build.gradle` in your study, adding these lines right below the "plugins" section:

```
sourceSets {
    main {
        java {
            srcDir "../../lib/src/main/java"
        }
    }
}
```
4. In VSCode, click File::Save Workspace As... and choose your study directory.

I usually rebuild the whole project at this point: Explorer::Java Projects::three dots::Clean Workspace.

I would also appreciate it if you set your workspace settings so that some of the default VSCode features are turned off:

* inlay
* minimap
* detect indentation (we always use 4)
* also use some "light" theme so I can read it.
* also turn on "reveal if open" so you can use multiple little windows correctly
* and turn off "sticky scrolling"
