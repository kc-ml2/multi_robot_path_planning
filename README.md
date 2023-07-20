# multi_robot_path_planning
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