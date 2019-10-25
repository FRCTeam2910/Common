# FRC Team 2910 Common code repository

This repository contains code that is shared between applications and robots
that are created by FRC Team 2910, Jack in the Bot. The repository itself is
currently split into two parts: common and robot. The common library contains
code that does not depend on FIRST's WPILib, meaning it can run as a part of
desktop applications. The robot library contains code that depends on WPILib
and should only be ran on robots.

## How to use in another project

The common library is imported into a project by using
[Git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). In
order to add this project as a submodule run `git submodule add
https://github.com/FRCTeam2910/Common-Public.git common` in your project's root
directory. This will clone this repository into the directory `common`.

### Using the Common library

A dependency on the common library can be created by adding the following to
your `build.gradle` file:
```gradle
dependencies {
    ...
    compile project(':common')
    ...
}
```
The following also needs to be added to your `settings.gradle` file:
```gradle
include ':common'
```

### Using the Robot library

A dependency on the robot library can be created by adding the following to
your `build.gradle` file:
```gradle
plugins {
    ...
    id 'edu.wpi.first.GradleRIO' version '<GRADLERIO_VERSION>'
    ...
}
...
dependencies {
    ...
    compile project(':common:robot')
    ...
}
```
Replace `<GRADLERIO_VERSION>` with the version of GradleRIO you want to use.

> NOTE: The common library does not need to be an explicit dependency because
> it is a dependency of the robot library.

The following also needs to be added to your `settings.gradle` file:
```gradle
include ':common'
include ':common:robot'
```
