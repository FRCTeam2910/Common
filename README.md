# FRC Team 2910 Common code repository

This repository contains code that is shared between applications and robots
that are created by FRC Team 2910, Jack in the Bot. The repository itself is
currently split into two parts: common and robot. The common library contains
code that does not depend on FIRST's wpilib, meaning it can run as a part of
desktop applications. The robot library contains code that depends on wpilib
and should only be ran on robots.

## How to use in another project

Currently, FRC Team 2910 does not have a system for hosting maven dependencies,
this means to use the libraries, the libraries must be built and installed to
the local maven repository.

To install locally on Windows run `gradlew publishToMavenLocal`.

While in on Linux or macOS run `./gradlew publishToMavenLocal`.

In order to use the locally-installed libraries, add `mavenLocal()` to the
repositories block in your `build.gradle` file.

```gradle
repositories {
    ...
    mavenLocal()
    ...
}
```

### Using the Common library

A dependency on the common library can be created by adding the following to your `build.gradle` file.
```gradle
dependencies {
    ...
    compile group: 'org.frcteam2910.common', name: 'common', version: '0.1.0'
    ...
}
```

### Using the Robot library

Using the robot library is a little more complicated. In order for the library
to not crash at runtime, the following dependencies need to be added to your
`build.gradle` file.
```gradle
dependencies {
    ...
    compile wpilib()
    compile ctre()
    compile navx()
    ...
}
```

> NOTE: `ctreLegacy()` cannot be used

After that, the robot library can be added.
```gradle
dependencies {
    ...
    compile group: 'org.frcteam2910.common', name: 'robot', version: '0.1.0
    ...
}
```

> NOTE: The common library does not need to be explicitly used because it is a
> dependency of the robot library.
