# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program

---

### Setup :
##### default project installation :
1. mkdir build
2. cd build
3. cmake ..
4. make
5. ./path_planning

##### my own Build Instructions
I was using Windows 10 and VisualStudio17

Make sure to include the **CMakeLists.txt** from this repo,
,because I added 2 files to resources :
- src/trajectory_generator.h
- src/trajectory_generator.cpp 

to build this project using **Bash for window** :

    navigate to projet folder
    write cmd : mkdir build && cd build
    write cmd : cmake .. -G "Unix Makefiles" && make
    write cmd : ./path_planning

---
### Goals
check **Goals** section, at [udacity repo](https://github.com/udacity/CarND-Path-Planning-Project)

let's start with the fun part
### Results
  * the car can drive more than 5 miles without incedents
  * it can keep safe distance from front car, and drive with constant speed without jerking
  * it's aware of surronding cars, and car act saftly in traffic jams
  * it will change lanes when needed too.
      * will prefare left lane over right lane, if both are available.
      * change lane in less than 3 second without jerking
      * can detect cars on side lanes and predect thier next position.
      * can detect cars on side lane for longer distance, witch allows better lane choice.

here are some gif animation examples : *note: gif files quality are reduced for smaller sizes*

1. the car will keep safe distance from the car in the front, and will change lane if available.
it will alwyas prefare left lane if both lane are available.
  
   ![ ](https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/master/res/_01-leftoverright.gif)
2. car will never get off the road, in this case it will change lane to right if left is blocked.

   ![ ](https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/master/res/_00-wontleaveroad.gif)
3. if front car breaks suddenly, our car can break without jerk, and keep safe distance , avoiding collision.

   ![ ](https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/master/res/_02break.gif)
4. car can detect the speed of side lanes cars, this provides better dicision making , because we know if the car in the next lane is speeding up or not.

   ![ ](https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/master/res/_03detectspeed.gif)
5. in case of traffic jam, car can drive as slow as the jam, avoiding collisions and not making chnage lane desisions without finding safe distances.

   ![ ](https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/master/res/_04jam.gif)
6. whenever car can find safe distance to do lane change, it will do it and do it fast, to fly out of the jam.

   ![ ](https://github.com/anasmatic/CarND-Term3-Project1-Path-Planning/blob/master/res/_05outofjam.gif)


### Code Exceplaning:
  * I used initial code from project Walk-Through video on Youtube, code is documented , you can find it in <span style="color:green;">main.cpp</span> <span style="color:red;">line:247</span> to <span style="color:red;">line:363</span>.
  * replaced the sensor fusion from the Walk-Through video to my function <span style="color:gray;">`plan_path`</span> <span style="color:red;">line:259</span>:
  * I'll take about code flow, more details can be found in code in-line documentation.

#### - state :
I'm using 5 state :
  * LANE_KEEP
  * PREPARE_LANE_CHANE_RIGHT
  * LANE_CHANE_RIGHT
  * PREPARE_LANE_CHANE_LEFT
  * LANE_CHANE_LEFT

#### - cost :
I'm also checking 2 costs :
  * Cost_of_Lane_change :<br/> which forks to
    * Cost_of_Lane_Keep
    * Cost_of_Lane_Change_Right
    * Cost_of_Lane_Change_Left
  * Cost of Collision

car will be by default in LANE_KEEP state, but will check the Cost_of_Lane_Keep :
- first check state
  - if state is LANE_KEEP
    - if no cost, we'll keep lane and state LANE_KEEP
    - else we have to check Cost_of_Lane_change
      - if cost is equal or more than MAX_COST, then no lane change, we'll keep lane and state LANE_KEEP
      - else we have to compare right cost vs left cost, but we won't change lane instantly
        - set state to PREPARE_LANE_CHANE_RIGHT or PREPARE_LANE_CHANE_LEFT, and wait for next update
  - if state is PREPARE_LANE_CHANE_RIGHT or PREPARE_LANE_CHANE_LEFT
    - check Cost of Collision
      - if has Collision cost, abort lane change, we'll keep lane and state LANE_KEEP
      - else set state to LANE_CHANE_RIGHT or LANE_CHANE_LEFT, and change lane right away
  - finally check if Lane change is done , if it is done change make sure to reset state to LANE_KEEP

#### - Cost functions details
 - **collision** cost function uses position prediction , to compare my car vs another car in the future, if they may collapse, then a collision is possible
 - **change lane** checks in two different approaches :
   - I'd like to note that we are checking cars on side lane now
   - **cars in-front us:**
     - if ahead car is slower than me, this will increase the cost of lane change.
     - then we check for distance between us and the ahead car, the close the more costly the decision is.
   - **cars behind us :**
     - first check distance, if too close, then don't change lane
     - then check position is future, if the car behind me will be ahead of me after 2 seconds, then abort lane change, it is too dangerous.


