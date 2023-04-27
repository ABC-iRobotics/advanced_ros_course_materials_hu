---
title: Verziókövetés, Git
author: Nagy Tamás
---

# 05. Verziókövetés, Git

---

## Elmélet

--- 


### Version control, Git

---

![](https://cdn.freebiesupply.com/logos/thumbs/2x/git-logo.png){:style="width:300px" align=right}



- Track changes in a set of files
- Coordinating work among developers
- Who made what changes and when
- Revert back at any time
- Local and remote repos
- Take snapshots of files by making a *commit*
 
---


#### Install

---

```bash
sudo apt install git
```

---

#### Basic commands

---

```bash
git init          # Initialize local git repo
git add <file>    # Add file/files to staging area
git status        # Check status of working tree and staging area
git commit -m "What I've done"    # Commit changes in index
git push          # Push to remote repository
git pull          # Pull latest changes from remote repo
git branch <new_branch_name>
git checkout <branch_name>
git merge <branch_name>   # Merge the branch into the current branch
git config --global user.name "Istvan Szabo"
git config --global user.email "istvan.szabo@gmail.com"

```




---

#### GitHub

---

![](https://github.githubassets.com/images/modules/logos_page/GitHub-Mark.png){:style="width:300px" align=right}


```bash
git remote
git clone <link>   # Copy repo into a new directory

# Add remote to repository:
git remote add origin <link>
git push -u origin master
```


!!! note "Some alternatives to GitHub"
    GitLab, BitBucket, Launchpad, Phabricator 

---

## Markdown

---

- Markup language, easy to read
- Text file &rarr; Formatted document
- Widespread usag, e.g., blogs, forums, documentations, readme files, GitHub 
- [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)

---

## Gyakorlat

---

### 0: GitHub repo létrehozása

1. Regisztráljunk GitHub-ra, készítsünk egy tokent.

    ---

2. Hozzunk létre egy private repo-t GitHub-on a `ros2_course` package számára.


!!! tip
    **Personal token megjegyzése:** `git config --global credential.helper store`


    ---


3. Hozzuk létre a local repo-t, állítsuk be a remote-ot, majd push-oljuk a package
tartalmát GitHUb-ra (a GitHub is segít a repo létrehozása után):

    ```bash
    cd ~/ros2_ws/src/ros2_course
    git init
    git add .
    git commit -m "Initial commit"
    git branch -M main
    git remote add origin <REPO_GITHUB_ADDRESS>.git
    git push -u origin main
    ```



    ---


4. Adjunk hozzá egy README.md-t a ros2_course csomaghoz az alábbi tartalommal:
 
    ```markdown
    # ros2_course

    ## About
    
    Something about the package.

    ## Usage
   
    How to *build* and use the package.
   
        cd ~/ros2_ws
        colcon build --symlink-install
    ```
    
    ---

5. Commit-oljunk és push-oljuk a változtatásokat:

    ```bash
    git add .
    git commit -m "Add README"
    git push
    ```

    !!! tip "VCS Clion-ban"
        A GitHub használata CLion-ban is beállítható, így grafikus felületen kezelhetjük a verziókat.

---

!!! tip
    **Windows és Linux óra probléma megoldása:** `timedatectl set-local-rtc 1 --adjust-system-clock`



## Hasznos linkek

- [Markdown Cheatsheet](https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet)


