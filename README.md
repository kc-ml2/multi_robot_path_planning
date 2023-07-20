# multi_robot_path_planning

This branch is for producing tree architecture file output and documentation

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

## Text format of tree architecture

Node json type 1

```json
{
	"index" : 0   # 0 : root node, index = 1, 2, ...
	"parent": 0, 
 	"state": {
		"x": 0,
		"y": 0,
		"z": 0
	},
	"costs": 0.0,
	"timestamp": 0, # Increase in the order in which it was created -> 1, 2, ...
	... (add required variables)
}
```

Node json type 2

```json
{
	"index" : 0   # 0 : root node, index = 1, 2, ...
	"parent": 0, 
 	"state": [0.0, 0.0, 0.0], #x, y, z
	"costs": 0.0,
	"timestamp": 0, # Increase in the order in which it was created -> 1, 2, ...
	... (add required variables)
}
```

Node array of each robot

```json
{
	"robot_id": 0,
	"history":[
		{Node Json},
		{Node Json},
		...
	],
	"trees":[   #result tree
		{
			"parent": -1,  #root node
			"state": [0.0, 0.0, 0.0], #x, y, z
			"cost": 0.0,
			"children": [1, 2], #index of children trees
		}, 
		{
			"parent": 0,
			"state": [0.0, 0.0, 0.0], #x, y, z
			"cost": 0.0,
			"children": [4], #index of children trees
		},
		...
	]
}
```