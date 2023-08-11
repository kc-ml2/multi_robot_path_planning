#include "mRRT.h"

#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>
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
    ppm_.loadFile("/home/alex/Documents/Planner/multi_robot_path_planning/resources/ppm/floor.ppm");
}

ompl::geometric::mRRT::~mRRT()
{
    for(int i=0;i<v_rrtStar_.size();i++){
        OMPL_INFORM("Found %d solutions", (int)v_rrtStar_[i]->pdef_->getSolutionCount());
    }
    recordSolution(0, 255, 0, 0);
    recordSolution(1, 0, 255, 0);
    recordSolution(2, 0, 0, 255);
    recordSolution(3, 125, 0, 0);
    recordSolution(4, 0, 255, 255);
    recordSolution(5, 255, 0, 255);
    recordSolution(6, 0, 125, 0);
    recordSolution(7, 0, 0, 125);
    
    save("result_demo.ppm");
}

void ompl::geometric::mRRT::setup()
{
    Planner::setup();

    std::vector<std::vector<int>> p_start { {229, 109}, {253, 762}, {401, 1288}, {910,1405}, {1561, 1131}, {1537, 651}, {1625, 54}, {360, 543}};
    std::vector<std::vector<int>> p_end { {253, 762}, {401, 1288}, {910,1405}, {1561, 1131}, {1537, 651}, {1625, 54}, {360, 543}, {229, 109}};

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
        pdef->setStartAndGoalStates(start, goal);
        pdef->clearSolutionPaths();
        
        rrtStar->setProblemDefinition(pdef);
        rrtStar->setup();

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
    
    //Generator
    auto rng = std::default_random_engine {};
    std::vector<int> index_array = {0, 1, 2, 3, 4, 5, 6, 7};

    while (ptc == false)
    {
        //Shuffle before new iteration
        std::shuffle(std::begin(index_array), std::end(index_array), rng);
        std::vector<int> used_index = {};

        for(int i=0;i<v_rrtStar_.size();i++){
            //Append current index to index list for collision checks
            used_index.push_back(index_array[i]);
            auto res = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_once(ptc, v_lv[i], v_lv, used_index);          
            if(res == 1) {
                OMPL_INFORM("Optimal path is found: %d", i);
                break;
            }
        }
    }

    for(int i=0;i<v_rrtStar_.size();i++){
        ret = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_end(ptc, v_lv[i]);
    }

    //ret = v_rrtStar_[i]->solve(ptcond);
    //bool ret = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_once();

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
    if (!haveSolutionPath(idx))
        return;
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