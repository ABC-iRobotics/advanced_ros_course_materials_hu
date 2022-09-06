---
title: 01. Bevezetés
author: Tamas D. Nagy
tags: Lectures, ROS course
---

# 01. Introduction

---

## Robot Operating System (ROS) introduction

---

### The definition of robot

- **Joseph Engelberger, pioneer in industrial robotics:** *"I can't define a robot, but I know one when I see one."*
- **Wikipedia:** *"A robot is a machine—especially one programmable by a computer— capable of carrying out a complex series of actions automatically. Robots can be guided by an external control device or the control may be embedded within. Robots may be constructed on the lines of human form, but most robots are machines designed to perform a task with no regard to their aesthetics."*
- **ISO 8373:2012 Robots and robotic devices – Vocabulary, FDIS 2012:** *"A robot is an actuated mechanism programmable in two or more axes with a degree of autonomy, moving within its environment, to perform intended tasks."*
- **Rodney Brooks, Founder and CTO, Rethink Robotics:** *"A robot is some sort of device, wich has sensors those sensors the world, does some sort of computation, decides on an action, and then does that action based on the sensory input, which makes some change out in the world, outside its body. Comment: the part "make some change outside its body" discriminates a washing machine from e.g. a Roomba."*
- **Tamás Haidegger, Encyclopedia of Robotics**: *"A robot is a complex mechatronic system enabled with electronics, sensors, actuators and software, executing tasks with a certain degree of autonomy. It may be pre-programmed, teleoperated or carrying out computations to make decisions."*

---

### What is ROS?

![](https://moveit.ros.org/assets/images/logo/ROS_logo.png){:style="width:300px" align=right}

- Open-source, robotics themed middleware
- Modularity, reusability (drivers, algorithms, libraries, ...)
- Hardware abstraction, ROS API
- C++ és Python support
- Ubuntu Linux (except ROS 2)
- Great community

![](https://upload.wikimedia.org/wikipedia/commons/4/43/Ros_Equation.png)

---

### History

- Mid 2000s, Stanford: robotics themed, flexible, dynamic framework for prototype development
- 2007, Willow Garage: incubation, the core of ROS under BSD license
- Spread in robotics reserach, PR2
- 2012: Industrial robotics, ROS-Industrial
- 2017: ROS 2

<iframe width="560" height="315" src="https://www.youtube.com/embed/mDwZ21Zia8s" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

---

## Development system build -- homework

---

Recommended environment:
    
- Ubuntu 20.04
- ROS Noetic
- *IDE: QtCreator*

---

1. ROS



    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    source /opt/ros/noetic/setup.bash
    ```


    A `source` parancs a környezeti változók beállításáért felelős, ezt minden új terminálablak megnyitásakor meg kell(ene) adni. Ez a parancs beilleszthető a `~/.bashrc` fájl végére, amely minden terminálablak megnyitásakor lefut, így nem kell mindig beírnunk:


    ```bash
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

    ---

2. ROS dependencies




    ```bash
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    ```

    Ha ezzel megvagyunk, a következő parancssal tesztelhetjük a ROS telepítésünket:


    ```bash
    roscore
    ```

    ---
    

3. További csomagok


    Az alábbi csomagokra is szükség lesz a félév során, így ezeket is érdemes feltelepíteni:


    ```bash
    sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-catkin-tools python3-osrf-pycommon libasound2-dev libgl1-mesa-dev xorg-dev
    ```

    ---


4. QtCreator

    ROS csomagok fejlesztéséhez jelenleg a leginkább használható IDE a QtCreator, melyhez ROS plugin is készült. Az installer az alábbi linken elérhető. A "18.04 **offline** installer"-t érdemes használni, ez működik Ubunutu 20.04-en is.

    [https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html)

    Ha letöltöttük, az IDE az alábbi paranccsal telepíthető (fontos, hogy `cd`zzünk be a letöltés helyére):


    ```bash
    sudo ./qtcreator-ros-bionic-latest-online-installer.run
    ```

    Amikor a telepítő kérdezi, hova telepítse, módosítsuk pl. `/home/<USER>/QtCreator` mappára. Ha a root-ba teléepítjük, nem fogjuk tudni futtatni. A telepítés után "Qt Creator (4.9.2)" néven keressük.

---

!!! tip "Suggestion"
    Install **Terminator** terminal emulator:
    ```bash
    sudo apt update
    sudo apt install terminator
    ```


---

## Useful links

- [https://www.ros.org/](https://www.ros.org/)
- [https://www.ros.org/install/](https://www.ros.org/install/)
- [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)
- [Online MD editor: HackMD](https://hackmd.io/)
- [QtCreator + ROS plugin](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html)
- [IROB virtual tour](https://www.youtube.com/watch?v=8XmKGWBV5Nw)
- [ROS 10 years montage](https://www.youtube.com/watch?v=mDwZ21Zia8s)


























