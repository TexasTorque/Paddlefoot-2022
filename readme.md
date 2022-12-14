# Paddlefoot-2022

Codebase for Paddlefoot, our 2022 offseason robot. 

Master = Rapid React. [Mineral Mayhem Manual](https://docs.google.com/document/d/14zJiKeNHfTmeQ2ZzQ8RP-kx3hduD3rV0CSx2J5x9dfw/edit)

## New project structure

```txt
.
|-- Main
|-- Robot
|-- Input
|-- Ports
|-- subsystems
    |-- Drivebase extends TorqueSubsystem
    | ..`
|-- auto
    |-- AutoManager extends TorqueAutoManager
    |-- commands
        |-- SampleCommand extends TorqueCommand
        | ...
    |-- sequences
        |-- SampleSequence extends TorqueSequence
        | ...
|-- torquelib (submodule)
```

## Controls

Please publish major control changes to the [Google Drive folder](https://drive.google.com/drive/folders/1bb2S4_d-e_ZlqMYT-pupsMfWWjnk1Fzt), and inform other programmers and driveteam.

## Contributing

Before you do anything, check the project board and issues tab. If you are adding a new feature, create a new issue. Pull requests are enforced to write to master. Create a branch, named as a description of the changes made in the branch, and push your changes to it. From GitHub, create a pull request to merge into master. In your pull request, write a description of changes, and link the issue(s) you are addressing using "Closes #1477" (with the actual number of the issue). After creating the PR, request review. The PR will either be approved or rejected, deleting the branch, or changes will be requested from the review.

## Formatting

Remember the order of modifiers (from left to right):

- `private`, `protected`, or `public`
- `static`
- `final`
- `volatile` or `synchronized`

~~To acquire the formatter in VSCode, download [this extension](https://marketplace.visualstudio.com/items?itemName=redhat.java).~~

~~Then, go to `Settings -> Text Editor -> Formatting` and enable "Format on Paste" and "Fromat on Save". Additionally, search for "default formatter" and select "Language Support for Java(TM) by Red Hat".~~
