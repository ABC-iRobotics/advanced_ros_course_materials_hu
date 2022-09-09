---
title: Követelmények
author: Tamas D. Nagy
---

# Követelmények

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
- Értékelési szempontok:
    - a megoldás teljessége
    - megfelelő ROS kommunikáció alkalmazása
    - program célszerű szerkezete
    - az implementáció minősége
    - a kód dokumentálása

---

## Ütemezés

---

| Okt. hét | Dátum      | Számonkérés |
|:--------:| ---------- | ----------- |
|2.| február 14  | Kötelező programok ismertetése. |
|4.| február 28 | Kötelező programok választása. |
|9| április 4  | Kötelező program mérföldkő.|
|14.| május 9 | Kötelező programok bemutatása. |

---

## Évközi jegy

---

A félév elfogadásának feltétele, hogy mind a két ZH, mind a kötelező program értékelése legalább elégséges. A két ZH közül az egyik az utolsó óra alkalmával pótolható.

!!! abstract "Félév végi jegy"
	$Jegy = (ZH1 + ZH2 + 2 \times KötProg) / 4$ 

---

## Témaválasztás

---

- A 4. heti órán (február 28.) konzultálunk a kötelező programokról

!!! warning
	A választott témáját mindenki küldje el emailben **február 28. 19:00-ig** erre az email címre: `tamas.daniel.nagy@irob.uni-obuda.hu`. Az email tárgya legyen **ROS_kötprog_2022_tavasz**.

---

## Kötelező program témák

---

### 1. PlatypOUs 

---


![](https://i.imgur.com/mCuxG54.png)

#### 1.1. PlatypOUs pályakövetés



- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása pályakövetésre szimulált környezetben bármely szenzor felhasználásával(pl. fal mellett haladás adott távolságra LIDAR segítségével).
- **Epic:** Implementáció és tesztelés a valós hardware-en és/vagy nyűgözz le!

#### 1.2. PlatypOUs akadály elkerülés

- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása akadály felismerésére és az akadályt kikerülő trajektória tervezésére és megvalósítására szimulált környezetben bármely szenzor felhasználásával.
- **Epic:** Implementáció és tesztelés a valós hardware-en és/vagy nyűgözz le!

#### 1.3. PlatypOUs objektum követés

- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** ROS rendszer implementálása objektum megkeresésére/felismerésére és követésére/megközelítésére szimulált környezetben bármely szenzor felhasználásával (pl. visual servoing).
- **Epic:** Implementáció és tesztelés a valós hardware-en és/vagy nyűgözz le!

#### 1.4. PlatypOUs action library

- **Basic:** Szimulátor élesztése, SLAM tesztelése. ROS node/node-ok implementálása szenzorok adatainak beolvasására és a a robot mozgatására.
- **Advanced:** Egyszerű műveleteket tartalmazó, ROS action alapú könyvtár és ezeket végrehajtó rendszer implementálása (pl. push object, move to object, turn around).
- **Epic:** Implementáció és tesztelés a valós hardware-en és/vagy nyűgözz le!


---

### 2. AMBF

---


#### 2.1. da Vinci sebészrobot ROS integrációja AMBF szimulátorban

![](https://i.imgur.com/tmhAkwg.png)

- **Basic:** Szimulátor élesztése, robot vezérlése joint space-ben és task space-ben (IK már implementálva AMBF-ben) ROS-ból CRTK szerinti topic-okon keresztül
- **Advanced:** Objektumok detektálása *Peg transfer puzzle*-ben
- **Epic:** Autonóm manipuláció *Peg transfer*-en és/vagy nyűgözz le!

#### 2.2. KUKA robotkar ROS integrációja AMBF szimulátorban

![](https://i.imgur.com/4FyvHM5.png)

- **Basic:** Szimulátor élesztése, robot vezérlése joint space-ben ROS-ból
- **Advanced:** Robot vezérlése task space-ben, IK?
- **Epic:** Trajektóriatervezés

#### 2.3. PR2 humanoid robot ROS integrációja AMBF szimulátorban

![](https://i.imgur.com/tGCClwQ.png)

- **Basic:** Szimulátor élesztése, robot vezérlése joint space-ben ROS-ból
- **Advanced:** Robot vezérlése task space-ben, IK?
- **Epic:** Trajektóriatervezés/Navigáció/Manipuláció

---

### X. Saját téma

---

Megegyezés alapján.

---

## Hasznos linkek

- [Gazebo ROS packages](http://wiki.ros.org/gazebo_ros_pkgs)
- [PlatypOUs](https://github.com/ABC-iRobotics/PlatypOUs-Mobile-Robot-Platform)
- [AMBF](https://github.com/WPI-AIM/ambf)
- [My fork of AMBF](https://github.com/TamasDNagy/ambf)
- [CRTK topics](https://github.com/jhu-cisst/cisst/blob/devel/utils/crtk-port/crtk-ros-commands.dict)
- [Navigation stack](http://wiki.ros.org/navigation)
- [Paper on LiDAR SLAM](https://www.hindawi.com/journals/jat/2020/8867937/)
- [Paper on vSLAM](https://ipsjcva.springeropen.com/articles/10.1186/s41074-017-0027-2)
- [Paper on Visual Servoing Mobile Robot](https://www.researchgate.net/publication/252057005_An_image_based_visual_servoing_scheme_for_wheeled_mobile_robots)









