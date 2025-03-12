#include "mRRT.h"

#include <iostream>
#include <fstream>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
#include <thread>
#include "ompl/base/Goal.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/util/GeometricEquations.h"


ompl::geometric::mRRT::mRRT(const base::SpaceInformationPtr &si)
  : base::Planner(si, "mRRT")
{
    ppm_.loadFile("/home/alex/Documents/Planner/multi_robot_path_planning/resources/ppm/0.ppm"); //This file is for the visualization - Change it depending on which map is being run in the main line
}

ompl::geometric::mRRT::~mRRT()
{
    for(int i=0;i<v_rrtStar_.size();i++){
        OMPL_INFORM("Found %d solutions", (int)v_rrtStar_[i]->pdef_->getSolutionCount());
    }
    recordSolution(0, 255, 0, 0);
    recordSolution(1, 0, 255, 0);
    //Add rows here for higher robot number count
    
    save("result_demo.ppm");
}

void ompl::geometric::mRRT::setup()
{
    Planner::setup();

    std::vector<std::vector<int>> p_start { {50, 50}, {50, 950}}; //Starting states for each robot
    std::vector<std::vector<int>> p_end { {950, 950}, {950, 50}}; //Goal states for each robot

    for(int n_robot=0;n_robot<p_start.size();n_robot++){

        auto rrtStar = std::make_shared<ompl::geometric::RRTstar>(si_);
        rrtStar->setName(getName());

        ompl::base::ProblemDefinitionPtr pdef = std::make_shared<base::ProblemDefinition>(si_);

        ompl::base::ScopedState<> start(si_->getStateSpace());
        start[0] = p_start[n_robot][0];
        start[1] = p_start[n_robot][1];
        ompl::base::ScopedState<> goal(si_->getStateSpace());
        goal[0] = p_end[n_robot][0];
        goal[1] = p_end[n_robot][1];


        rrtStar->setStartGoal(p_start, p_end);
        
        
        pdef->setStartAndGoalStates(start, goal);
        pdef->clearSolutionPaths();
        
        rrtStar->setProblemDefinition(pdef);
        rrtStar->setup();

        rrtStar->setRadius(10.0);
        
        rrtStar->setGoalBias(0.1);
        rrtStar->setRewireFactor(0.1);

        rrtStar->setTreePruning(0);
        rrtStar->setKNearest(1);
        rrtStar->setDelayCC(0); 
        rrtStar->setSampleRejection(1);
        rrtStar->setNewStateRejection(0);
        rrtStar->setAdmissibleCostToCome(0);
        rrtStar->setOrderedSampling(0);
        rrtStar->setBatchSize(1);



        v_rrtStar_.push_back(rrtStar);

    }
}

void ompl::geometric::mRRT::clear()
{
    Planner::clear();
    for(int i=0;i<v_rrtStar_.size();i++){
        v_rrtStar_[i]->clear();
    }
}

void ompl::geometric::mRRT::checkGoals(std::vector<ompl::geometric::RRTstar::LoopVariables> &v_lv) {
    //Check goals when appropriate - Separating this from main RRT* expansion helps it not dominate time complexity in difficult examples
    threadExit = false;
    if (v_rrtStar_[0]->as<ompl::geometric::RRTstar>()->checkForGoals(v_lv)) {

    }
    threadExit = true;
}

void ompl::geometric::mRRT::writeToCSV(std::vector<ompl::geometric::RRTstar::LoopVariables> v_lv, std::ofstream& myfile, std::ofstream& timefile)
{
    using Clock = std::chrono::steady_clock;
    using std::chrono::time_point;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using namespace std::literals::chrono_literals;

    time_point<Clock> start_thread;
    time_point<Clock> end_thread;

    time_point<Clock> first_start_thread = Clock::now();

    double first_solution_thread = -1.0;
    start_thread = Clock::now();
    //Iterate thread record every t time period
    while (true)
    {
        milliseconds diff_thread = duration_cast<milliseconds>(Clock::now()-start_thread);
        if (diff_thread.count() >= 50) {

                auto cost = v_rrtStar_[0]->as<ompl::geometric::RRTstar>()->getCost();
            
                if (first_solution_thread < 0.0) {
                    first_solution_thread = duration_cast<milliseconds>(Clock::now()-first_start_thread).count();
                    timefile << first_solution_thread << '\n';
                } 
                if (cost > 1000000.0) {
                    cost = 0.0;
                }
                myfile << cost << ",";


            start_thread = Clock::now();
        } 
    }
}

ompl::base::PlannerStatus ompl::geometric::mRRT::solve(const base::PlannerTerminationCondition &ptc)
{ 
    ompl::base::PlannerStatus ret;

    std::vector<ompl::geometric::RRTstar::LoopVariables> v_lv;

    for(int i=0;i<v_rrtStar_.size();i++){
        ompl::geometric::RRTstar::LoopVariables lv;
        auto res = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_init(ptc, lv);
        if(!res) {
            OMPL_ERROR("failed initialize RRTstart ==> %d", i);
            return base::PlannerStatus::INVALID_START;
        }

        v_lv.push_back(lv);

    }
    
    for (int i=0; i<v_rrtStar_.size(); i++) {
        v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->setIndex(i, v_lv[i]);
    }
    
    //Generator
    auto rng = std::default_random_engine {};
    std::vector<int> index_array = {0, 1};
    int res = 0;
    std::vector<int> used_index = {};

    int current_it = 0;
    int milliwait = 50; //Recording interval


    //Setting up timer for recordings
    time_t current_time;
    std::time(&current_time);

    time_t next_time;

    using Clock = std::chrono::steady_clock;
    using std::chrono::time_point;
    using std::chrono::duration_cast;
    using std::chrono::milliseconds;
    using namespace std::literals::chrono_literals;

    time_point<Clock> start;
    time_point<Clock> end;

    time_point<Clock> first_start = Clock::now();

    std::ofstream myfile ("DDORRT.csv", std::ios::out | std::ios::app);
    std::ofstream timefile ("DORRT-time.csv", std::ios::out | std::ios::app); //Change file name for recording times

    
    std::thread t1(&ompl::geometric::mRRT::writeToCSV, this, v_lv, std::ref(myfile), std::ref(timefile));
    t1.detach();

    double first_solution = -1.0;

    start = Clock::now();
    while (ptc == false)
    {
        //Shuffle before new iteration
        std::shuffle(std::begin(index_array), std::end(index_array), rng);
        used_index = {};

        //For debugging - This shouldn't matter as a new robot isn't checked until it is called and new_motion is set as new epoch
        for (int i=0; i<index_array.size(); i++) {
            v_lv[i].new_motion = NULL;
        }

        // Original code wasn't working because rewiring and initial path plan weren't separated


        bool success;
        for(int i=0;i<index_array.size();i++){
            //Append current index to index list for collision checks
            used_index.push_back(index_array[i]);

            success = true;
            int num_checks = 50;
            for (int checks=0; checks<num_checks; checks++) {
                res = v_rrtStar_[index_array[i]]->as<ompl::geometric::RRTstar>()->solve_once(ptc, v_lv[index_array[i]], v_lv, used_index); 
                if (res==1.0) {
                    break;
                }
                if (checks == num_checks-1) {
                    success = false;
                }
            }
            //Failed to find solution, other paths not possible
            if (!success) {
                break;
            }
            res = 0;
        }
        //If all cases were successful move the new motion to recently added motions
        if (success) {
            for (int i=0;i<index_array.size();i++) {
                v_lv[i].rmotion = v_lv[i].new_motion;
            }
        

        

            //For rewire all successful rewires are added (order comes priority)
            //Shuffle before new iteration

            std::shuffle(std::begin(index_array), std::end(index_array), rng);
            std::vector<std::vector<int>> checkLists = {};
            std::vector<int> checkList = {};
            used_index = {};
            for(int i=0;i<index_array.size();i++){
                //Append current index to index list for collision checks
                used_index.push_back(index_array[i]);

                //No need to iterate as we are using the recently added node nearest neighbors
                checkList = v_rrtStar_[index_array[i]]->as<ompl::geometric::RRTstar>()->rewire(ptc, v_lv[index_array[i]], v_lv, used_index); 
                //checkLists.push_back(checkList);
            }

            // Check for paths
            milliseconds diff = duration_cast<milliseconds>(Clock::now()-start);
            if ((diff.count() >= 1000 || (first_solution < 0.0 && (diff.count() >= 50))) && threadExit) {
                std::thread t2(&ompl::geometric::mRRT::checkGoals, this, std::ref(v_lv));
                t2.join();
                start = Clock::now();
            }
        }

        
        
    }
    

   //Record path for plotting
    for(int i=0;i<v_rrtStar_.size();i++){
        ret = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_end(ptc, v_lv[i]);
    }
 
    for (int i=0; i<v_lv.size(); i++) {
        auto out_path = v_lv[i].trueFinalGoalMotion;
        //v_lv[i].finalGoalMotion = new Motion(si_);
        for (int j=0; j<v_lv[i].trueFinalGoalMotion.size(); j++) {
            OMPL_INFORM("%f, %f", v_lv[i].trueFinalGoalMotion[j][0], v_lv[i].trueFinalGoalMotion[j][1]);
        }
        
        OMPL_INFORM("\n");
    }
  
    myfile << '\n';
    myfile.close();
    timefile.close();

    return ret;
}

void ompl::geometric::mRRT::getPlannerData(base::PlannerData &data) const
{
    for(int i=0;i<v_rrtStar_.size();i++){
        v_rrtStar_[i]->getPlannerData(data);
    }
}


bool ompl::geometric::mRRT::haveSolutionPath(int idx)
{
    return v_rrtStar_[idx]->pdef_->hasSolution();
}

ompl::geometric::PathGeometric ompl::geometric::mRRT::getSolutionPath(int idx)
{
    if (v_rrtStar_[idx]->pdef_)
    {
        const base::PathPtr &p = v_rrtStar_[idx]->pdef_->getSolutionPath();
        if (p)
            return static_cast<ompl::geometric::PathGeometric &>(*p);
    }
    throw Exception("No solution path");
}

void ompl::geometric::mRRT::recordSolution(int idx, unsigned char red, unsigned char green, unsigned char blue)
{
    OMPL_INFORM("??");
    if (!haveSolutionPath(idx)) {
    OMPL_INFORM("!!");
        return;
    }
    ompl::geometric::PathGeometric p = getSolutionPath(idx);
    p.interpolate();
    for (std::size_t i = 0; i < p.getStateCount(); ++i)
    {
        const int w = std::min(1858, (int)p.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[0]);
        const int h =
            std::min(1567, (int)p.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values[1]);
        ompl::PPM::Color &c = ppm_.getPixel(h, w);
        c.red = red;
        c.green = green;
        c.blue = blue;
    }
}


void ompl::geometric::mRRT::save(const char *filename)
{
    ppm_.saveFile(filename);
}
