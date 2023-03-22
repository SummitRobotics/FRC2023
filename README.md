# FRC 2023

[![Build](https://github.com/SummitRobotics/FRC2023/actions/workflows/build.yml/badge.svg?branch=main)](https://github.com/SummitRobotics/FRC2023/actions/workflows/build.yml)

FRC code from team Chaos Theory (5468, Summit High School) for Apophis, our 2023 robot. This code is written in Java and based off of WPILib's Java control system. It utilizes a command-based architecture.

The code is divided into several packages, each responsible for a different aspect of the robot function. This README explains setup instructions, the function of each package, and some of the variable naming conventions used. Additional information about each specific class can be found in their respective Java files.

## Setup Instructions

### General
1. Clone this repository
1. Run `./gradlew` to download gradle and needed FRC/Vendor libraries
1. Run `./gradlew tasks` to see available options
1. Enjoy!

### Visual Studio Code (Official IDE)
1. Get the WPILib extension for easiest use from the VSCode Marketplace - Requires Java 11 or greater
1. In [`.vscode/settings.json`](.vscode/settings.json), set the User Setting, `java.home`, to the correct directory pointing to your JDK 11 directory

### IntelliJ
1. Run `./gradlew idea`
1. Open the `FRC-2023-Public.ipr` file with IntelliJ

### Eclipse
1. Run `./gradlew eclipse`
1. Open Eclipse and go to File > Open Projects from File System...
1. Set the import source to the `FRC-2023-Public` folder then click finish

### Basic Gradle Commands
- Run `./gradlew deploy` to deploy to the robot in Terminal (*nix) or Powershell (Windows)
- Run `./gradlew build` to build the code.  Use the `--info` flag for more details
- Run `./gradlew test` to run all of the JUnit tests

### List of IP Addresses on Robot Network
- RoboRIO: 10.54.68.2
- Limelight: 10.54.68.16:5801
- PhotonVision: 10.54.68.202:5800

## Conventions

### Variable Naming Conventions
- CONSTANT_VARIABLE - Constant variables are all caps with underscores seperating names.
- normalVariable - Most everything else uses cammelCase.
- ClassName - All classes use Title Case per normal java convention.

### Branch Naming Conventions
- a_branch_name - Generic branches. Mainly feature branches
- clean_##### or clean_NAME - A branch for cleaning up code
- dev_##### or dev_NAME - A branch for rapid development (Ex. At comps)
- fix_##### or fix_NAME - A branch for making thoughout fixes
- hotfix_##### or hotfix_NAME - A branch for hotfixes
- \#\#\#\#\# - Number in the format MonthDayIncrement (Ex. for the second cleanup branch on Jan 20 (clean_01202))
