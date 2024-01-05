#include "mRRT.h"
#include <math.h>
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

#include <cstdlib>


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

<<<<<<< Updated upstream
=======
        rrtStar->setRadius(5.0);

        rrtStar->setRange(200.0);
        
        rrtStar->setGoalBias(0.05);
        rrtStar->setRewireFactor(0.1);

        rrtStar->setTreePruning(1);
        rrtStar->setKNearest(1);
        rrtStar->setDelayCC(1); //maybe not
        rrtStar->setSampleRejection(1);
        rrtStar->setNewStateRejection(1);
        rrtStar->setAdmissibleCostToCome(1);
        rrtStar->setOrderedSampling(1);
        rrtStar->setBatchSize(10);



>>>>>>> Stashed changes
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

std::vector<std::vector<double>> ompl::geometric::mRRT::apply_shortcut(std::vector<ompl::base::State*> a, std::vector<double> b, std::vector<ompl::base::State*> c){

    // randomly select points along each of line segments a -> b and b > c
    std::vector<std::vector<double>> outs;

    double r1 = rand();
    double r2 = rand();
    std::vector<double> a_ = {};
    std::vector<double> c_ = {};
    for (int i=0; i<a.size(); i++) {
        a_[i] = b[i] - (1.0-r1)*(*a[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values) + 
                                *a[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
        c_[i] = *c[i]->as<ompl::base::RealVectorStateSpace::StateType>()->values - (1.0-r2)*b[i] + b[i];
    }

    outs[0] = a_;
    outs[1] = c_;

    return outs;

}

std::vector<std::vector<ompl::base::State*>> ompl::geometric::mRRT::apply_rsc(std::vector<std::vector<ompl::base::State*>> paths){
    std::vector<std::vector<ompl::base::State*>> new_paths;

     // No. Robots * Length of path * Dimension (State)
    
    //Shortcut iterations
    for (int i=0; i<500; i++) {

        int segNum;
        //No. robot * State
        std::vector<ompl::base::State*> a;
        std::vector<ompl::base::State*> b;
        std::vector<ompl::base::State*> c;
        
        //Choose a segment and pull states for each DoF
        segNum = rand() % (paths[0].size()-2) + 1;
        for (int j=0; j<paths.size(); j++) {
            a.push_back(paths[j][segNum-1]);
            b.push_back(paths[j][segNum]);
            c.push_back(paths[j][segNum+1]);
        }

        double radius;

        int dim = paths.size();
        double ab[dim];
        double nums[dim];

        double den = 0.0;
        double num = 0.0;
        
        //Get radius
        for (int j=0; j<dim; j++) {
            ab[j] = a[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values - 
                    c[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values;
            nums[j] = ab[j]*(*b[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values);
            num += std::pow(nums[j], 2.0);
            
            den += std::pow(ab[j], 2.0);
        }

        radius = std::pow(num, 0.5)/std::pow(den, 0.5);
        
        //distance to generate new points within different DoF
        double dist = radius * 0.2;

        std::vector<double> d = {};
        double total;
        double shift;

        //Begin calculating shifted point
        while ( true ) {
            auto rng = std::default_random_engine {};
            std::vector<int> index_array = {0, 1, 2, 3, 4, 5, 6, 7};
            std::shuffle(std::begin(index_array), std::end(index_array), rng);

            total = std::pow(radius, 2.0);

            for (int j=0; j<paths.size()-1; j++) {
                shift = (rand() - 0.5) * 2.0 * dist;
                d.push_back(*b[index_array[i]]->as<ompl::base::RealVectorStateSpace::StateType>()->values + shift);
                total -= d[d.size()-1];
                if (total < 0) {
                    break;
                }
            }
            
            if (total > 0) {
                d.push_back(*b[index_array[b.size()-1]]->as<ompl::base::RealVectorStateSpace::StateType>()->values + 
                                std::pow(total, 0.5));

                //Attempt Short-Cut
                std::vector<std::vector<double>> pta = apply_shortcut(a, d, c);
                for (int j=0; j<a.size(); j++) {
                    a[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values = &pta[0][j];
                    c[j]->as<ompl::base::RealVectorStateSpace::StateType>()->values = &pta[1][j];
                }

                //Check collisions here

                if (!v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->checkRobots(a, c)) {
                    break;
                }


                for (int j=0; j<paths.size(); j++) {
                    paths[j].erase(paths[j].begin() + segNum);
                    paths[j].insert(paths[j].begin() + segNum, c[j]);
                    paths[j].insert(paths[j].begin() + segNum, a[j]);
                }
                break;
            }
        }
    }

    return paths;   
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
    int res = 0;
    while (ptc == false)
    {
        //Shuffle before new iteration
        std::shuffle(std::begin(index_array), std::end(index_array), rng);
        std::vector<int> used_index = {};
        for(int i=0;i<v_rrtStar_.size();i++){
            //Append current index to index list for collision checks
            used_index.push_back(index_array[i]);
            while(res == 0) {
                res = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_once(ptc, v_lv[index_array[i]], v_lv, used_index); 
            }
            res = 0;
        }
    }

    // Merge paths // No. Robots * Length of path * Dimension (State)
    std::vector<std::vector<ompl::base::State*>> paths; 
    int maxSize = 0;
    for (int i=0; i<index_array.size(); i++) {
        auto cur = v_lv[i].bestGoal;
        std::vector<ompl::base::State*> path;
        while( cur->parent != NULL ) {
            path.push_back(cur->state);
            cur = cur->parent;
        }
        paths.push_back(path);
        if ( path.size() > maxSize ) {
            maxSize = path.size();
        }
    }
    
    for (int i=0; i<index_array.size(); i++) {
        for (int j=0; j<paths[i].size()-maxSize; j++) {
            paths[i].push_back(paths[i][paths[i].size()-1]);
        }
    }
    
    //This section can probably be moved to a different function


    // collect goal paths & pad final states -> extract state values out for complete ease in processing
    std::vector<std::vector<std::vector<double>>> pre_paths = {};
    int max_length = 0;
    int counter;
    int dim = sizeof(v_lv[0].rmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values)/sizeof(double);

    //Combine the state dimension across all robots s.t. num_segments * num_robots * dim_states -> num_segments * num_states
    for (int i=0; i<index_array.size(); i++) {
        auto motion = v_lv[i].finalGoalMotion;
        std::vector<std::vector<double>> cur_path = {};
        counter = 0;
        while( motion != NULL ) {
            std::vector<double> cur_state = {};
            for (int j=0; j<dim; j++) {
                cur_state.push_back(motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
            }
        }

        //For rewire all successful rewires are added (order comes priority)
        //Shuffle before new iteration
        //std::shuffle(std::begin(index_array), std::end(index_array), rng);
        used_index = {};
        for(int i=0;i<index_array.size();i++){
            //Append current index to index list for collision checks
            used_index.push_back(index_array[i]);

            //No need to iterate as we are using the recently added node nearest neighbors
            v_rrtStar_[used_index[i]]->as<ompl::geometric::RRTstar>()->rewire(ptc, v_lv[used_index[i]], v_lv, used_index); 

        }
    }
    
    //This section can probably be moved to a different function


    // collect goal paths & pad final states -> extract state values out for complete ease in processing
    std::vector<std::vector<std::vector<double>>> pre_paths = {};
    int max_length = 0;
    int counter;
    int dim = sizeof(v_lv[0].rmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values)/sizeof(double);

    //Combine the state dimension across all robots s.t. num_segments * num_robots * dim_states -> num_segments * num_states
    for (int i=0; i<index_array.size(); i++) {
        auto motion = v_lv[i].finalGoalMotion;
        std::vector<std::vector<double>> cur_path = {};
        counter = 0;
        while( motion != NULL ) {
            std::vector<double> cur_state = {};
            for (int j=0; j<dim; j++) {
                cur_state.push_back(motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
            }
            cur_path.push_back(cur_state);
            motion = motion->parent;
            counter += 1;
        }
        if (counter > max_length) {
            max_length = counter;
        }
        pre_paths.push_back(cur_path);
    }
    
    //pad endings and combine state
    for (int i=0; i<index_array.size(); i++) {
        if (pre_paths[i].size() < max_length) {
            for (int j=0; j<max_length-pre_paths[i].size(); j++) {
                pre_paths[i].insert(pre_paths[i].begin(), pre_paths[i][0]);
            }
        }
    }
    std::vector<std::vector<double>> paths = {};
    for (int i=0; i<index_array.size(); i++) {
        paths.push_back({});
        for (int j=0; j<pre_paths[0].size(); j++) {
            for (int k=0; k<pre_paths[0][0].size(); k++) {
                paths[i].push_back(pre_paths[i][j][k]);
            }
        }
    }


    //Apply Shortcutting - most will fail so this can be quite generous
    int num_attempts = 100;
    int total_dim = paths[0].size(); //num robots * dimensionality
    int path_length = paths.size();
    int path_index; //segment of path to apply shortcutting

    std::vector<double> p0;
    std::vector<double> p1;
    std::vector<double> p2;

    for (int i=0; i<num_attempts; i++) {
        path_index = (rand() % (path_index-2));
        p0 = paths[path_index];
        p1 = paths[path_index+1];
        p2 = paths[path_index+2];

        //Choose random point along line - for now uniform squared - ideally gaussian biased towards adjoining vertex
        double r1 = std::pow(rand(), 2.0);
        double r2 = std::pow(rand(), 2.0);

        //Find location of segment
        std::vector<double> n1 = {};
        std::vector<double> n2 = {};
        for (int j=0; j<p1.size(); j++) {
            n1.push_back(p1[j] + (p0[j]-p1[j])*r1);
            n2.push_back(p1[j] + (p2[j]-p1[j])*r2);
        }
        //Borrow from rrt to check motion for individual states
        bool success = true;
        for (int j=0; j<index_array.size(); j++) {
            if (!v_rrtStar_[j]->as<ompl::geometric::RRTstar>()->checkMotionObjectDouble(n1, n2, dim)) {
                success = false;
                break;
            }
        }

        //if successful modify path
        if (success) {
            paths.erase(paths.begin() + path_index+1);
            paths.insert(paths.begin() + path_index+1, n2);
            paths.insert(paths.begin() + path_index+1, n1);
        }
    }


    //Move path back to original state - keep in mind these nodes are reversed
    std::vector<std::vector<double>> final;
    std::vector<double> final_state;
    for (int i=0; i<index_array.size(); i++) {
        final = {};
        for (int j=0; j<paths.size(); j++) {
            final_state = {};
            for (int k=0; k<dim; k++) {
                final_state.push_back(paths[j][i*dim + k]);
            }
            final.push_back(final_state);
        }
        v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->convertMotion(final, v_lv[i], i);
    }


    for(int i=0;i<v_rrtStar_.size();i++){
        ret = v_rrtStar_[i]->as<ompl::geometric::RRTstar>()->solve_end(ptc, v_lv[i]);
    }

    //********

    //Need to add randomization

    //********

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