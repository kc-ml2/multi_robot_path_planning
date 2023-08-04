# multi_robot_path_planning

This branch is for producing tree architecture file output and documentation
Notion documentation site is bellow
https://dog-ferry-0ea.notion.site/OMPL-Documentation-8e67b036da4c477ba80c19bdef9e972e?pvs=4

# Installing and configuring OMPL

### Installation OMPL

```bash

#install wget 
sudo apt install wget

# download ompl shell script
wget https://ompl.kavrakilab.org/install-ompl-ubuntu.sh

#change file authority
chmod u+x install-ompl-ubuntu.sh

#run shell script with OMPL.app and python binding (maybe full packages)
./install-ompl-ubuntu.sh --app
```

### Build from source (to modify source code)

- Clone OMPL git repo
    - git clone https://github.com/ompl/ompl.git
- Dependencies install
    - Boost (version 1.58 or higher)
    - CMake (version 3.5 or higher)
    - Eigen (verison 3.3 or higher)
- Build OMPL source

```bash
mkdir -p build/Release
cd build/Release
cmake ../..

make -j 4 update_bindings # if 
```

---

# OMPL dev

## Installation

```bash
sudo apt install imagemagick
```

## Structure of ompl_test

```
ompl_test/       # root directory
├─ algorithm/    # Modified algorithm files based on OMPL package
│	├─ include/
│	│	└─ RRTstar.h
│	├─ RRTstar.cpp
│
├─ build/        # Package build
│
├─ resources/    # Resources like map or else.
│	└─ ppm/
│		└─ floor.ppm
│
├─ src/          # Main file of this packages
│	└─ Point2DPlanning.cpp
```

## How to build and run ompl_app

```bash
cd ./build

#build with cmake & make
cmake ..
make

./ompl_app    #run ompl_app
./ompl_app ../resources/ppm/floor.ppm    #run ompl_app with resources/ppm/floor.ppm file

pnmtopng result_demo.ppm > result.png
```

---

## Text format of tree architecture

Node json type

- to change parent architecture

```json
{
	"index" : 0   # 0 : root node, index = 1, 2, ...
	"parent": {
		"value":[ 0.0, 0.0, 0.0] #x, y, z
		"index": 0
	}, 
 	"state": [0.0, 0.0, 0.0], #x, y, z
	"costs": 0.0,
	"timestamp": 0, # Increase in the order in which it was created -> 1, 2, ...
	... (add required variables)
}
```

Node array of each robot

- Erase trees → add next time…

```json
{
	"robot_id": 0,
	"history":[
		{Node Json},
		{Node Json},
		...
	],
}
```

## Full json code

- erase map_file → cannot get filename in RRTStar.cpp

```json
{
	"data":[
		{
			"robot_id": 0,
			"history":[
				{Node Json},
				{Node Json},
				...
			],
		}
		{
			"robot_id": 1,
			"history":[
				{Node Json},
				{Node Json},
				...
			],
		},
		...
	],
}
```
