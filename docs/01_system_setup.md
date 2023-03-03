---
title: Bevezetés
author: Nagy Tamás
---

# 01. Bevezetés

---

## Robot Operating System (ROS) bevezetés

---

### A robot fogalma

![](img/what_is_a_robot_1.png){:style="width:300px" align=right}

- **Joseph Engelberger, pioneer in industrial robotics:** *"I can't define a robot, but I know one when I see one."*
- **Wikipedia:** *"A robot is a machine—especially one programmable by a computer— capable of carrying out a complex series of actions automatically. Robots can be guided by an external control device or the control may be embedded within. Robots may be constructed on the lines of human form, but most robots are machines designed to perform a task with no regard to their aesthetics."*
- **ISO 8373:2012 Robots and robotic devices – Vocabulary, FDIS 2012:** *"A robot is an actuated mechanism programmable in two or more axes with a degree of autonomy, moving within its environment, to perform intended tasks."*
- **Rodney Brooks, Founder and CTO, Rethink Robotics:** *"A robot is some sort of device, wich has sensors those sensors the world, does some sort of computation, decides on an action, and then does that action based on the sensory input, which makes some change out in the world, outside its body. Comment: the part "make some change outside its body" discriminates a washing machine from e.g. a Roomba."*
- **Tamás Haidegger, Encyclopedia of Robotics**: *"A robot is a complex mechatronic system enabled with electronics, sensors, actuators and software, executing tasks with a certain degree of autonomy. It may be pre-programmed, teleoperated or carrying out computations to make decisions."*

---

### Mi a ROS?

![](https://moveit.ros.org/assets/images/logo/ROS_logo.png){:style="width:300px" align=right}

- Open-source, robotikai célú middleware
- Modularitás, újra-felhasználhatóság (driverek, algoritmusok, library-k, ...)
- Hardware absztrakció, ROS API
- C++ és Python támogatás
- Ubuntu Linux (kivéve ROS 2)
- Népes közösség

<iframe title="vimeo-player" src="https://player.vimeo.com/video/639236696?h=740f412ce5" width="640" height="360" frameborder="0"    allowfullscreen></iframe>


---


### Történet

![](https://www.freshconsulting.com/wp-content/uploads/2022/06/path-planning-1024x693.jpg){:style="width:300px" align=right}

- 2000-es évek közepe, Stanford: robotikai célú rugalmas, dinamikus szoftverrendszer prototípusok fejlesztése
- 2007, Willow Garage: inkubáció, kialakult a ROS alapja BSD open-source licensz alatt
- Robotikai kutatások területén egyre inkább elterjedt, PR2
- 2012: Ipari robotika, ROS-Industrial
- 2017: ROS 2



---

## Fejlesztőkörnyezet felállítása - Házi feladat

---

![](https://brandslogos.com/wp-content/uploads/thumbs/ubuntu-logo-vector.svg){:style="width:400px" align=right}

Ajánlott környezet:
    
- Ubuntu 20.04
- ROS1 Noetic
- ROS2 Foxy
- *IDE: QtCreator/CLion/VSCode*



---

## ROS 1 Noetic 


![](http://wiki.ros.org/noetic?action=AttachFile&do=get&target=noetic.png){:style="width:300px" align=right}

1. ROS Noetic telepítése

    ```bash
    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    sudo apt install curl
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt update
    sudo apt install ros-noetic-desktop-full
    source /opt/ros/noetic/setup.bash
    ```

    ---
    
2. ROS 1 dependencies
    
    ```bash
    sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
    sudo rosdep init
    rosdep update
    ```
   
    ---

3. Ha ezzel megvagyunk, a következő parancssal tesztelhetjük a ROS 1 telepítésünket:


    ```bash
    source /opt/ros/noetic/setup.bash
    roscore
    ```

---    

### ROS 2 Foxy

![](https://global.discourse-cdn.com/business7/uploads/ros/optimized/2X/d/d6fd5322bd2ddc06530d8352fcab20f0bca08c06_2_861x1024.png){:style="width:300px" align=right}


1. Locale beállítása.

    ```bash
    locale  # check for UTF-8

    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8

    locale  # verify settings
    ```
   
    ---

2. ROS 2 Foxy telepítése


    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-foxy-desktop python3-argcomplete ros-dev-tools ros-foxy-moveit* ros-foxy-control*
    ```

    ---

3. Ha ezzel megvagyunk, a következő parancssal tesztelhetjük a ROS 2 telepítésünket:


    ```bash
    source /opt/ros/foxy/setup.bash
    ros2 run demo_nodes_py talker
    ```

    ---
    
4. A `source` parancs a környezeti változók beállításáért felelős, ezt minden új terminálablak megnyitásakor meg kell(ene) adni. Ez a parancs beilleszthető a `~/.bashrc` fájl végére, amely minden terminálablak megnyitásakor lefut, így nem kell mindig beírnunk (ROS 2 lesz az alapértelmezett):


    ```bash
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    ```
    
---

### További csomagok


1. Az alábbi csomagokra is szükség lesz a félév során, így ezeket is érdemes feltelepíteni:


    ```bash
    sudo apt install libxml2-dev libraw1394-dev libncurses5-dev qtcreator swig sox espeak cmake-curses-gui cmake-qt-gui git subversion gfortran libcppunit-dev libqt5xmlpatterns5-dev python3-catkin-tools python3-osrf-pycommon libasound2-dev libgl1-mesa-dev xorg-dev ros-foxy-turtlebot3*
    ```

---


### IDE


1. QtCreator

    ROS csomagok fejlesztéséhez jelenleg az egyik leginkább használható IDE a QtCreator, melyhez ROS plugin is készült. Az installer az alábbi linken elérhető. A "18.04 **offline** installer"-t érdemes használni, ez működik Ubunutu 20.04-en is.

    [https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html)

    Ha letöltöttük, az IDE az alábbi paranccsal telepíthető (fontos, hogy `cd`zzünk be a letöltés helyére):


    ```bash
    chmod +x qtcreator-ros-bionic-latest-offline-installer.run
    sudo ./qtcreator-ros-bionic-latest-offline-installer.run
    ```

    Amikor a telepítő kérdezi, hova telepítse, módosítsuk pl. `/home/<USER>/QtCreator` mappára. Ha a root-ba teléepítjük, nem fogjuk tudni futtatni. A telepítés után "Qt Creator (4.9.2)" néven keressük.
   
    ---

2. CLion

    A CLion magasfokú ROS integrációval rendelkezik, a kurzus során ennek a használata a leginkább ajánlott. Ingyenes hallgatói licensz az alábbi linken igényelhető: [https://www.jetbrains.com/community/education/#students](https://www.jetbrains.com/community/education/#students)

    Telepítés után keressük meg a `/var/lib/snapd/desktop/applications/clion_clion.desktop` fájlt. A megfelelő sort írjuk át erre, így a terminál által beállított környezetet fogja használni az IDE:

    ```bash
    Exec=bash -i -c "/snap/bin/clion" %f
    ```

    ---

3. Visual Studio
    
    A Microsoft Visual Studio szintén támogatja a ROS-hoz készült forráskódokat, ez az IDE is használható a félév során.
    
    
---

!!! tip "Suggestion"
    Install **Terminator** terminal emulator:
    ```bash
    sudo apt update
    sudo apt install terminator
    ```


---

## Hasznos linkek

- [https://www.ros.org/](https://www.ros.org/)
- [ROS 1 Noetic installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
- [ROS 2 Foxy installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- [ROS Distributions](http://wiki.ros.org/Distributions)
- [http://wiki.ros.org/ROS/Tutorials](http://wiki.ros.org/ROS/Tutorials)
- [CLion hallgatói licensz](https://www.jetbrains.com/community/education/#students)
- [QtCreator + ROS plugin](https://ros-qtc-plugin.readthedocs.io/en/latest/_source/How-to-Install-Users.html)
- [IROB virtual tour](https://www.youtube.com/watch?v=8XmKGWBV5Nw)





















