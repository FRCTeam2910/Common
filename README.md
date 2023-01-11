# FRC Team 2910 Common code repository

This repository contains code that is shared between applications and robots
that are created by FRC Team 2910, Jack in the Bot. The common library contains
code that depends on FIRST's WPILib's math and util packages, and it can run as a part of
a desktop applications. 

## How to use in another project

The common library is imported into a project by using
[Git submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules). In
order to add this project as a submodule run `git submodule add
https://github.com/FRCTeam2910/Common.git common` in your project's root
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