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
|    8.    | márc. 31 | Kötelező programok ismertetése. Projekt labor I. |
|   13.    | máj. 26  | Projekt labor II.                                |
|   14.    | jún. 2   | Kötelező programok bemutatása.                   |

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

### 1. TurtleBot3 


[TurtleBot3 ROS tutorial](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)



---

#### 1.1. TurtleBot akadály elkerülés


![turtlebot_world.png](img%2Fturtlebot_world.png){:style="width:380px" align=right}

- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása akadály felismerésére és az akadályt kikerülő trajektória tervezésére és megvalósítására szimulált környezetben bármely szenzor felhasználásával.
- **Epic:** Nyűgözz le!

---

#### 1.2. TurtleBot pályakövetés

![](https://robots.ros.org/assets/img/robots/turtlebot3/turtlebot3.png){:style="width:380px" align=right}



- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása pályakövetésre szimulált környezetben bármely szenzor felhasználásával(pl. fal mellett haladás adott távolságra LIDAR segítségével).
- **Epic:** Nyűgözz le!
<!--suppress XmlDeprecatedElement -->
<font size="1"> Image source: https://robots.ros.org/turtlebot3/ </font>


---

#### 1.3. TurtleBot objektum követés/visual servoing

- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása objektum megkeresésére/felismerésére és követésére/megközelítésére szimulált környezetben bármely szenzor felhasználásával (pl. visual servoing).
- **Epic:** Nyűgözz le!

---

#### 1.4. TurtleBot action library

- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** Egyszerű műveleteket tartalmazó, ROS action alapú könyvtár és ezeket végrehajtó rendszer implementálása (pl. push object, move to object, turn around).
- **Epic:** Nyűgözz le!



---

### 2. YouBot

<iframe width="560" height="315" align="right" src="https://www.youtube.com/embed/qvBEQsGvC3M" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


[YouBot controller GitHub](https://github.com/ABC-iRobotics/YoubotDriver/tree/ROS)

---


#### 2.1. YouBot ROS integráció

- **Basic:** YouBot repo build-elése, megismerése
- **Advanced:** Szimulált robot mozgatása csuklótérben ROS környezetben
- **Epic:** Tesztelés valós roboton és/vagy nyűgözz le!


---

### 3. AMBF

[AMBF GitHub](https://github.com/WPI-AIM/ambf)

!!! tip "AMBF build-elése"
	Fork-oljuk az AMBF csomagot, majd a fork-ot clone-ozzuk:
	```bash
	cd ~/ros2_ws/src
	git clone <MY_AMBF_FORK.git>
	```
    Ne az AMBF dokumentációjában javasolt make-et használjuk, hanem colcon-t:
	```bash
    cd ~/ros2_ws
    colcon build --symlink-install
	```    
	Így indíthatjukl el a szimulátort:
	```bash    
    cd ~/ros2_ws/src/ambf/bin/lin-x86_64
	./ambf_simulator -l 4
	```

---


#### 3.1. da Vinci sebészrobot ROS integrációja AMBF szimulátorban

![ambf_psm.png](img%2Fambf_psm.png){:style="width:200px" align=right}

- **Basic:** Szimulátor élesztése, robot vezérlése joint space-ben és task space-ben (IK már implementálva AMBF-ben) ROS-ból CRTK szerinti topic-okon keresztül
- **Advanced:** Objektumok detektálása *Peg transfer puzzle*-ben
- **Epic:** Autonóm manipuláció *Peg transfer*-en és/vagy nyűgözz le!

#### 3.2. KUKA robotkar ROS integrációja AMBF szimulátorban

![ambf_kuka.png](img%2Fambf_kuka.png){:style="width:200px" align=right}


- **Basic:** Szimulátor élesztése, robot vezérlése joint space-ben ROS-ból
- **Advanced:** Trajektóriák generálása joint space-ben
- **Epic:** Inverz kinematika implementálása és/vagy nyűgözz le!

#### 3.3. PR2 humanoid robot ROS integrációja AMBF szimulátorban

![ambf_pr2.png](img%2Fambf_pr2.png){:style="width:200px" align=right}

- **Basic:** Szimulátor élesztése, robot vezérlése joint space-ben ROS-ból
- **Advanced:** Robot vezérlése task space-ben, IK?
- **Epic:** Trajektóriatervezés/Navigáció/Manipuláció és/vagy nyűgözz le!

---

### X. Saját téma

---

Megegyezés alapján.

---

## Hasznos linkek

- [TurtleBot4 Simulation](https://turtlebot.github.io/turtlebot4-user-manual/software/turtlebot4_simulator.html)
- [TurtleBot3 Tutorial](https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)
- [AMBF](https://github.com/WPI-AIM/ambf)
- [My fork of AMBF](https://github.com/TamasDNagy/ambf)
- [CRTK topics](https://github.com/jhu-cisst/cisst/blob/devel/utils/crtk-port/crtk-ros-commands.dict)
- [Navigation stack](http://wiki.ros.org/navigation)
- [Paper on LiDAR SLAM](https://www.hindawi.com/journals/jat/2020/8867937/)
- [Paper on vSLAM](https://ipsjcva.springeropen.com/articles/10.1186/s41074-017-0027-2)
- [Paper on Visual Servoing Mobile Robot](https://www.researchgate.net/publication/252057005_An_image_based_visual_servoing_scheme_for_wheeled_mobile_robots)









