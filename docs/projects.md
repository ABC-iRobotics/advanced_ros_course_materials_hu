---
title: Kötelező program
author: Levendovics Tamás
---

# Kötelező Program

---

## Ütemezés

---

| Okt. hét | Dátum    | Számonkérés                                      |
|:--------:|----------|--------------------------------------------------|
|    6.    | márc. 28 | Kötelező programok ismertetése. Projekt labor I. |
|   14.    | máj. 23  | Projekt labor II.                                |
|   13.    | máj. 30  | Kötelező programok bemutatása.                   |

---


## Nehézségi fokozatok és érdemjegyek

---

A kötelező programok három *nehézségi fokozatban* teljesíthetők. A *nehézségi fok* meghatározza a **legjobb** érdemjegyet, amely a teljesítéséért kapható! 

| Nehézségi fok | Legjobb megszerezhető érdemjegy |
| -------- | :-------: |
| Basic    |     3 |
| Advanced |     4 |
| Epic     |     5 |

!!! tip
	A feladatok úgy vannak megadva, hogy érdemes a **Basic** szinttel kezdeni, és onnan fokozatosan építkezni az **Epic** szintig.

A kötelező programok a következő szempontok szerint kerülnek értékelésre: 

- Bizonyítottan saját munka
- Értékelhető eredményeket produkáljon
- Verziókövetés használata, feltöltés GitHub/GitLab/egyéb repoba
- Launch fájlok
- Megoldás teljessége
- Megfelelő ROS kommunikáció alkalmazása
- Program célszerű ROS struktúrája
- Implementáció minősége
- Kód dokumentálása

!!! tip
    ChatGPT és egyéb MI eszközök használata megengedett.

---



## Évközi jegy

---

A félév elfogadásának feltétele, hogy mind a két ZH, mind a kötelező program értékelése legalább elégséges. A két ZH közül az egyik az utolsó óra alkalmával pótolható.

!!! abstract "Félév végi jegy"
	$Jegy = (ZH1 + ZH2 + 2 \times KötProg) / 4$ 

---


## Kötelező program témák

---

### 1. Mobil robot

#### A. Playground Robot

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)

![playground_robot.png](img%2Fplayground_robot.png){:style="width:600px"}


#### B. TurtleBot4

- [TurtleBot4 Simulator Tutorial](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- [TurtleBot4 GUI Docs](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_common.html#menu-control)  

![turtlebot.png](img%2Fturtlebot.png){:style="width:600px"}

#### C. PlatypOUs (ROS 1)

- [PlatypOUs GitHub](https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform)

![](https://www.mdpi.com/sensors/sensors-22-02284/article_deploy/html/images/sensors-22-02284-g001.png){:style="width:600px"}

#### D. Bármilyen mobil robot

---

#### 1.1. Mobil robot akadály elkerülés


- **Basic:** Szimulátor élesztése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása akadály felismerésére és az akadályt kikerülő trajektória tervezésére és megvalósítására szimulált környezetben bármely szenzor felhasználásával.
- **Epic:** Nyűgözz le!

---

#### 1.2. Mobil robot pályakövetés



- **Basic:** Szimulátor élesztése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása pályakövetésre szimulált környezetben bármely szenzor felhasználásával(pl. fal mellett haladás adott távolságra LIDAR segítségével).
- **Epic:** Nyűgözz le!



---

#### 1.3. Mobil robot objektum követés/visual servoing

- **Basic:** Szimulátor élesztése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása objektum megkeresésére/felismerésére és követésére/megközelítésére szimulált környezetben bármely szenzor felhasználásával (pl. visual servoing).
- **Epic:** Nyűgözz le!

---

#### 1.4. Mobil robot action library

- **Basic:** Szimulátor élesztése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** Egyszerű műveleteket tartalmazó, ROS action alapú könyvtár és ezeket végrehajtó rendszer implementálása (pl. push object, move to object, turn around).
- **Epic:** Nyűgözz le!

---

### 2. Quadcopter

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)  
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)  



    ```bash
    ign gazebo -v 4 -r quadcopter.sdf
    ```


![quadcopter.png](img%2Fquadcopter.png){:style="width:600px"}

- **Basic:** Szimulátor élesztése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása magasság/sebesség szabályozására.
- **Epic:** Nyűgözz le!

---


### 3. Szabadon választott Gazebo szimuláció

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)  
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)  
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/gz-sim7/examples/worlds)  

![pendulum.png](img%2Fpendulum.png){:style="width:600px"}


Megegyezés alapján.

---


### 4. Gazebo szimuláció összeállítása

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)  
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)  
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/gz-sim7/examples/worlds)  

![velocity.png](img%2Fvelocity.png){:style="width:600px"}

Megegyezés alapján.

---

### 5. TurtleSim

- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)  
- [Koch Görbe](https://en.wikipedia.org/wiki/Koch_snowflake)

![turtle_xmas_fast.gif](img%2Fturtle_xmas_fast.gif){:style="width:600px"}

#### 5.1 Turtlesim Fraktál/Szöveg

- **Basic:** Arányos szabályozó implementálása.
- **Advanced:** Fraktál/szöveg rajzolása.
- **Epic:** Nyűgözz le!

---

### 6. DVRK 

- [Download and compile dVRK 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2)
- [Marker examples](https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker)

![PSM_coordinates.png](img%2FPSM_coordinates.png){:style="width:600px"}

#### 6.1 DVRK Interaktív Marker

Megfogható, mozgatható marker implementálása a DVRK szimulátorához.

---

### 7. YouBot (Windows)

<iframe width="560" height="315" src="https://www.youtube.com/embed/qvBEQsGvC3M" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


- [YouBot controller GitHub](https://github.com/ABC-iRobotics/YoubotDriver/tree/ROS)

---


#### 7.1. YouBot ROS integráció

- **Basic:** YouBot repo build-elése, megismerése
- **Advanced:** Szimulált robot mozgatása csuklótérben ROS környezetben
- **Epic:** Tesztelés valós roboton és/vagy nyűgözz le!


---


### X. Saját téma

---

Megegyezés alapján.

---

## Hasznos linkek

- [Gazebo install](https://gazebosim.org/docs/harmonic/install_ubuntu)
- [Setting up a robot simulation (Gazebo)](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [Gazebo World Examples](https://github.com/gazebosim/gz-sim/tree/gz-sim7/examples/worlds)
- [YouBot controller GitHub](https://github.com/ABC-iRobotics/YoubotDriver/tree/ROS)
- [Download and compile dVRK 2](https://github.com/jhu-dvrk/sawIntuitiveResearchKit/wiki/BuildROS2)
- [Marker examples](https://www.programcreek.com/python/example/88812/visualization_msgs.msg.Marker)
- [Turtlesim Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html)
- [Koch Görbe](https://en.wikipedia.org/wiki/Koch_snowflake)  









