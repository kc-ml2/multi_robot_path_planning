/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2011, Rice University
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Rice University nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Authors: Alejandro Perez, Sertac Karaman, Ryan Luna, Luis G. Torres, Ioan Sucan, Javier V Gomez, Jonathan Gammell */

#include "RRTstar.h"
#include <algorithm>
#include <boost/math/constants/constants.hpp>
#include <limits>
#include <vector>

#include <math.h>
#include <cmath>

#include <ompl/base/spaces/RealVectorStateSpace.h>



ompl::geometric::RRTstar::RRTstar(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RRTstar")
{
    specs_.approximateSolutions = true;
    specs_.optimizingPaths = true;
    specs_.canReportIntermediateSolutions = true;

    Planner::declareParam<double>("range", this, &RRTstar::setRange, &RRTstar::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRTstar::setGoalBias, &RRTstar::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("rewire_factor", this, &RRTstar::setRewireFactor, &RRTstar::getRewireFactor,
                                  "1.0:0.01:2.0");
    Planner::declareParam<bool>("use_k_nearest", this, &RRTstar::setKNearest, &RRTstar::getKNearest, "0,1");
    Planner::declareParam<bool>("delay_collision_checking", this, &RRTstar::setDelayCC, &RRTstar::getDelayCC, "0,1");
    Planner::declareParam<bool>("tree_pruning", this, &RRTstar::setTreePruning, &RRTstar::getTreePruning, "0,1");
    Planner::declareParam<double>("prune_threshold", this, &RRTstar::setPruneThreshold, &RRTstar::getPruneThreshold,
                                  "0.:.01:1.");
    Planner::declareParam<bool>("pruned_measure", this, &RRTstar::setPrunedMeasure, &RRTstar::getPrunedMeasure, "0,1");
    Planner::declareParam<bool>("informed_sampling", this, &RRTstar::setInformedSampling, &RRTstar::getInformedSampling,
                                "0,1");
    Planner::declareParam<bool>("sample_rejection", this, &RRTstar::setSampleRejection, &RRTstar::getSampleRejection,
                                "0,1");
    Planner::declareParam<bool>("new_state_rejection", this, &RRTstar::setNewStateRejection,
                                &RRTstar::getNewStateRejection, "0,1");
    Planner::declareParam<bool>("use_admissible_heuristic", this, &RRTstar::setAdmissibleCostToCome,
                                &RRTstar::getAdmissibleCostToCome, "0,1");
    Planner::declareParam<bool>("ordered_sampling", this, &RRTstar::setOrderedSampling, &RRTstar::getOrderedSampling,
                                "0,1");
    Planner::declareParam<unsigned int>("ordering_batch_size", this, &RRTstar::setBatchSize, &RRTstar::getBatchSize,
                                        "1:100:1000000");
    Planner::declareParam<bool>("focus_search", this, &RRTstar::setFocusSearch, &RRTstar::getFocusSearch, "0,1");
    Planner::declareParam<unsigned int>("number_sampling_attempts", this, &RRTstar::setNumSamplingAttempts,
                                        &RRTstar::getNumSamplingAttempts, "10:10:100000");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });

}

ompl::geometric::RRTstar::~RRTstar()
{
    freeMemory();
}

void ompl::geometric::RRTstar::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!si_->getStateSpace()->hasSymmetricDistance() || !si_->getStateSpace()->hasSymmetricInterpolate())
    {
        OMPL_WARN("%s requires a state space with symmetric distance and symmetric interpolation.", getName().c_str());
    }

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });

    // Setup optimization objective
    //
    // If no optimization objective was specified, then default to
    // optimizing path length as computed by the distance() function
    // in the state space.
    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
            opt_ = pdef_->getOptimizationObjective();
        else
        {
            OMPL_INFORM("%s: No optimization objective specified. Defaulting to optimizing path length for the allowed "
                        "planning time.",
                        getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);

            // Store the new objective in the problem def'n
            pdef_->setOptimizationObjective(opt_);
        }

        // Set the bestCost_ and prunedCost_ as infinite
        bestCost_ = opt_->infiniteCost();
        prunedCost_ = opt_->infiniteCost();
    }
    else
    {
        OMPL_INFORM("%s: problem definition is not set, deferring setup completion...", getName().c_str());
        setup_ = false;
    }

    // Get the measure of the entire space:
    prunedMeasure_ = si_->getSpaceMeasure();

    // Calculate some constants:
    calculateRewiringLowerBounds();
}

void ompl::geometric::RRTstar::clear()
{
    setup_ = false;
    Planner::clear();
    sampler_.reset();
    infSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();

    bestGoalMotion_ = nullptr;
    goalMotions_.clear();
    startMotions_.clear();

    iterations_ = 0;
    bestCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedCost_ = base::Cost(std::numeric_limits<double>::quiet_NaN());
    prunedMeasure_ = 0.0;
}

bool ompl::geometric::RRTstar::solve_init(const base::PlannerTerminationCondition &ptc, LoopVariables& lv)
{
    checkValidity();

    lv.goal = pdef_->getGoal().get();

    lv.goal_s = dynamic_cast<base::GoalSampleableRegion *>(lv.goal);

    lv.symCost = opt_->isSymmetric();

    /*
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    bool symCost = opt_->isSymmetric();
    */
    // Check if there are more starts
    if (pis_.haveMoreStartStates() == true)
    {
        // There are, add them
        while (const base::State *st = pis_.nextStart())
        {
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, st);
            motion->cost = opt_->identityCost();
            nn_->add(motion);
            startMotions_.push_back(motion);
        }

        // And assure that, if we're using an informed sampler, it's reset
        infSampler_.reset();
    }
    // No else

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return false;
        //return base::PlannerStatus::INVALID_START;
    }

    // Allocate a sampler if necessary
    if (!sampler_ && !infSampler_)
    {
        allocSampler();
    }

    OMPL_INFORM("%s: Started planning with %u states. Seeking a solution better than %.5f.", getName().c_str(), nn_->size(), opt_->getCostThreshold().value());

    if ((useTreePruning_ || useRejectionSampling_ || useInformedSampling_ || useNewStateRejection_) &&
        !si_->getStateSpace()->isMetricSpace())
        OMPL_WARN("%s: The state space (%s) is not metric and as a result the optimization objective may not satisfy "
                  "the triangle inequality. "
                  "You may need to disable pruning or rejection.",
                  getName().c_str(), si_->getStateSpace()->getName().c_str());


    lv.intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    lv.approxGoalMotion = nullptr;
    lv.approxDist = std::numeric_limits<double>::infinity();

    lv.rmotion = new Motion(si_);
    lv.rstate = lv.rmotion->state;
    lv.xstate = si_->allocState();

    lv.rewireTest = 0;
    lv.statesGenerated = 0;

    /*
    const base::ReportIntermediateSolutionFn intermediateSolutionCallback = pdef_->getIntermediateSolutionCallback();

    Motion *approxGoalMotion = nullptr;
    double approxDist = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    base::State *xstate = si_->allocState();

    std::vector<Motion *> nbh;

    std::vector<base::Cost> costs;
    std::vector<base::Cost> incCosts;
    std::vector<std::size_t> sortedCostIndices;

    std::vector<int> valid;
    unsigned int rewireTest = 0;
    unsigned int statesGenerated = 0;

    */

    if (bestGoalMotion_)
        OMPL_INFORM("%s: Starting planning with existing solution of cost %.5f", getName().c_str(),
                    bestCost_.value());

    if (useKNearest_)
        OMPL_INFORM("%s: Initial k-nearest value of %u", getName().c_str(),
                    (unsigned int)std::ceil(k_rrt_ * log((double)(nn_->size() + 1u))));
    else
        OMPL_INFORM(
            "%s: Initial rewiring radius of %.2f", getName().c_str(),
            std::min(maxDistance_, r_rrt_ * std::pow(log((double)(nn_->size() + 1u)) / ((double)(nn_->size() + 1u)),
                                                     1 / (double)(si_->getStateDimension()))));

    // our functor for sorting nearest neighbors
    /*CostIndexCompare compareFn(lv.costs, *opt_);*/

    return true;
}

/* ***********************************************************************************************************
*                                                                                                              *
*                                                                                                              *
*                      Short-Cutting Here                                                                      *
*                                                                                                              *
*                                                                                                              *
***************************************************************************************************************/



/* ***********************************************************************************************************
*                                                                                                              *
*                                                                                                              *
*                      Collision Checking Here                                                                 *
*                                                                                                              *
*                                                                                                              *
***************************************************************************************************************/


std::vector<std::vector<std::vector<float>>> ompl::geometric::RRTstar::getPathStates(ompl::geometric::RRTstar::Motion* motion, std::vector<ompl::geometric::RRTstar::LoopVariables> valid_trees, int dim) {
    std::vector<std::vector<std::vector<float>>> path_states = {};
    std::vector<ompl::geometric::RRTstar::Motion*> copy_trees = {};
    std::vector<std::vector<float>> cur_states = {};
    ompl::geometric::RRTstar::Motion* motion_copy = motion;

    int max_path_length = 0;
    int length_counter;

    int counter=0;
    //Iterate through paths to get history to current point for completed points (could be optimized)
    for (int i=0; i<valid_trees.size()-1; i++) {
        cur_states = {};
        copy_trees.push_back(valid_trees[i].new_motion);
        length_counter = 0;
        while ( copy_trees[i] != NULL ) {
            std::vector<float> states = {};
            for (int j=0; j<dim; j++) {
                states.push_back(copy_trees[i]->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
            }

            cur_states.push_back(states);
            copy_trees[i] = copy_trees[i]->parent;
            if (length_counter > max_path_length) {
                max_path_length = length_counter;
            }

        }
        std::reverse(cur_states.begin(), cur_states.end());
        path_states.push_back(cur_states);
        
    }

    //Current motion (different from trees as it's not added yet)
    cur_states = {};
    length_counter = 0;
    while ( motion_copy != NULL ) {
        std::vector<float> states = {};
        for (int j=0; j<dim; j++) {
            states.push_back(motion_copy->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j]);
        }
        cur_states.push_back(states);
        motion_copy = motion_copy->parent;
        if (length_counter > max_path_length) {
                max_path_length = length_counter;
        }

    }

    std::reverse(cur_states.begin(), cur_states.end());
    path_states.push_back(cur_states);

    //make sure all lengths of robots are same, if not pause at goal
    for (int i=0; i<path_states.size(); i++) {
        while (path_states[i].size() < max_path_length) {
            path_states[i].push_back(path_states[i][path_states.size()-1]);
        }
    }
    
    return path_states;
}

bool ompl::geometric::RRTstar::colDistance(std::vector<float> rob1, std::vector<float> rob2, double col_dist, int dim) {
    double distance = 0.0;
    double temp = 0.0;

    for (int i=0; i<dim; i++) {
        temp = (rob1[i] - rob2[i]);
        distance += temp * temp;
    }
    if (distance < col_dist*col_dist) {
        return false;
    }
    return true;

}

std::vector<float> ompl::geometric::RRTstar::stepVector(std::vector<float> p1, std::vector<float> p2, float step) {
    std::vector<float> out = {};

    for (int i=0; i<p1.size(); i++) {
        out.push_back(p1[i] + (p2[i]-p1[i])*step);
    }
    return out;
}

//motion: current robot to be checked
//robots: all robots
//index: indexes of robots to consider (last one is current)
bool ompl::geometric::RRTstar::checkRobots(Motion* motion, std::vector<LoopVariables>& robots, std::vector<int> index) {
    //This should be moved elsewhere; either steps along collisions are integer movements or fractions of each branch
    ompl::base::RealVectorStateSpace space;
    int dim = sizeof(motion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values)/sizeof(float);
   
    double col_dist = getRadius();
    bool one_node_per = true;
    double step = 0.1;
    
    //These should be retrieved and reversed so that it is calculated from first point (goal states continue after full movement)
    //Get robots current processed
    std::vector<LoopVariables> valid_trees = {};
    for (int i=0; i<index.size(); i++) {
        valid_trees.push_back(robots[index[i]]);
    }

    //Extract trees
    std::vector<std::vector<std::vector<float>>> path_states = getPathStates(motion, valid_trees, dim);

    //iterate over path segments, over robots 1->n-1
    std::vector<std::vector<float>> new_robot = path_states[path_states.size()-1];
    float current_step = step;

    for (int i=0; i<path_states.size()-2; i++) {
        //Choose one robot to compare to
        std::vector<std::vector<float>> old_robot = path_states[i];

        //iterate through segments
        current_step = step;

        if (old_robot.size() == 0) {
            return true;
        }

        for (int j=0; j<(std::min(new_robot.size(), old_robot.size()))-1; j++) {
            //iterate through steps
            for (int k=0; k < (int) 1.0/step; k++) {
                if (!colDistance(stepVector(old_robot[j], old_robot[j+1], current_step), 
                                 stepVector(new_robot[j], new_robot[j+1], current_step), col_dist, dim)) {
                    //Collision
                    return false;
                }
            }
            current_step += step;            
        }
        
    }
    
    return true;
}

bool ompl::geometric::RRTstar::checkMotionObjectDouble(std::vector<double> p1, std::vector<double> p2, int dim) {
    ompl::base::State* s1 = si_->allocState();
    ompl::base::State* s2 = si_->allocState();

    for (int i=0; i<p1.size()/dim; i++) {
        for (int j=0; j<dim; j++) {
            s1->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = p1[i*dim + j];
            s2->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = p2[i*dim + j];
            if (!checkMotionObject(s1, s2)) {
                return false;
            }
        }
    }
    return true;
}

void ompl::geometric::RRTstar::convertMotion(std::vector<std::vector<double>> final, LoopVariables& lv, int index) {
    //put back into form - remember order is reversed
    std::vector<Motion*> m = {};
    for (int i=0; i<final.size(); i++) {
        m.push_back(new Motion(si_));
        for (int j=0; j<final[0].size(); j++) {
            m[i]->state->as<ompl::base::RealVectorStateSpace::StateType>()->values[j] = final[i][j];
        }
    }

    for (int i=0; i<final.size()-1; i++) {
        m[i]->parent = m[i+1];
    }
    m[m.size()-1]->parent = NULL;

    bestGoalMotion_ = m[0];
    lv.finalGoalMotion = m[0];
}

bool ompl::geometric::RRTstar::checkMotionObject(base::State* state, base::State* dstate)
{
    double radius = getRadius();
    ompl::base::State* p1 = si_->allocState();
    ompl::base::State* p2 = si_->allocState();

    for (int i=0; i<edgeChecks; i++) {
        double factor = 2.0*M_PI * ((double) i / edgeChecks);

        p1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 
                state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] + radius*std::cos(factor);

        p1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 
                state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] + radius*std::sin(factor);

        p2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 
                dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] + radius*std::cos(factor);

        p2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 
                dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] + radius*std::sin(factor);
        
        //Need to be implemented in a different way
        if (p1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] < radius) {
            return false;
        }
        if (p1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] < radius) {
            return false;
        }
        if (p2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] < radius) {
            return false;
        }
        if (p2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] < radius) {
            return false;
        }


        if (p1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] > 2000.0-radius) {
            return false;
        }
        if (p1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] > 2000.0-radius) {
            return false;
        }
        if (p2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] > 2000.0-radius) {
            return false;
        }
        if (p2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] > 2000.0-radius) {
            return false;
        }
        if (!si_->checkMotion(p1, p2)) {
            si_->freeState(p1);
            si_->freeState(p2);

            return false;
        }
    }

    si_->freeState(p1);
    si_->freeState(p2);

    return true;
}



int ompl::geometric::RRTstar::solve_once(const base::PlannerTerminationCondition &ptc, LoopVariables& lv, std::vector<LoopVariables>& robots, std::vector<int> index)
{
    CostIndexCompare compareFn(lv.costs, *opt_);

    iterations_++;

    // sample random state (with goal biasing)
    // Goal samples are only sampled until maxSampleCount() goals are in the tree, to prohibit duplicate goal
    // states.
    if (lv.goal_s && goalMotions_.size() < lv.goal_s->maxSampleCount() && rng_.uniform01() < goalBias_ &&
        lv.goal_s->canSample())
        lv.goal_s->sampleGoal(lv.rstate);
    else
    {
        // Attempt to generate a sample, if we fail (e.g., too many rejection attempts), skip the remainder of this
        // loop and return to try again
        if (!sampleUniform(lv.rstate))
            return 0; //continue;
    }

    // find closest state in the tree
    Motion *nmotion = nn_->nearest(lv.rmotion);

    if (lv.intermediateSolutionCallback && si_->equalStates(nmotion->state, lv.rstate))
        return 0; //continue;

    base::State *dstate = lv.rstate;

    // find state to add to the tree
    double d = si_->distance(nmotion->state, lv.rstate);
    if (d > maxDistance_)
    {
        si_->getStateSpace()->interpolate(nmotion->state, lv.rstate, maxDistance_ / d, lv.xstate);
        dstate = lv.xstate;
    }

    //LoG
    /*double* pnt = nmotion->state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    double* pnt2 = dstate->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    bool check = si_->checkMotion(nmotion->state, dstate);

    OMPL_INFORM("%d iteration First motion state: %f, %f, %f", iterations_, pnt[0], pnt[1], pnt[2]);
    OMPL_INFORM("%d iteration dstate %d : %f, %f, %f", iterations_, pnt2[0], pnt2[1], pnt2[2], check);
    */

    // Initialize branch indexes
    if (lv.rmotion->stateOrder == -1) {
        lv.rmotion->stateOrder = 0;
    }

    // Check if the motion between the nearest state and the state to add is valid
    if (checkMotionObject(nmotion->state, dstate))
    {
        // create a motion
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, dstate);
        motion->parent = nmotion;
        motion->incCost = opt_->motionCost(nmotion->state, motion->state);
        motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);

        //Check robot collisions here
        if (index.size() > 1 && !checkRobots(motion, robots, index)) {
            return 0;
        }
 
        // Find nearby neighbors of the new motion
        getNeighbors(motion, lv.nbh);

        lv.rewireTest += lv.nbh.size();

        //CJH inform .. Node number -> lv.stateGenerated.
        ++lv.statesGenerated;


        // cache for distance computations
        //
        // Our cost caches only increase in size, so they're only
        // resized if they can't fit the current neighborhood
        if (lv.costs.size() < lv.nbh.size())
        {
            lv.costs.resize(lv.nbh.size());
            lv.incCosts.resize(lv.nbh.size());
            lv.sortedCostIndices.resize(lv.nbh.size());
        }

        // cache for motion validity (only useful in a symmetric space)
        //
        // Our validity caches only increase in size, so they're
        // only resized if they can't fit the current neighborhood
        if (lv.valid.size() < lv.nbh.size())
            lv.valid.resize(lv.nbh.size());
        std::fill(lv.valid.begin(), lv.valid.begin() + lv.nbh.size(), 0);

        // Finding the nearest neighbor to connect to
        // By default, neighborhood states are sorted by cost, and collision checking
        // is performed in increasing order of cost
        if (delayCC_)
        {
            // calculate all costs and distances
            for (std::size_t i = 0; i < lv.nbh.size(); ++i)
            {
                lv.incCosts[i] = opt_->motionCost(lv.nbh[i]->state, motion->state);
                lv.costs[i] = opt_->combineCosts(lv.nbh[i]->cost, lv.incCosts[i]);
            }

            // sort the nodes
            //
            // we're using index-value pairs so that we can get at
            // original, unsorted indices
            for (std::size_t i = 0; i < lv.nbh.size(); ++i)
                lv.sortedCostIndices[i] = i;
            std::sort(lv.sortedCostIndices.begin(), lv.sortedCostIndices.begin() + lv.nbh.size(), compareFn);

            // collision check until a valid motion is found
            //
            // ASYMMETRIC CASE: it's possible that none of these
            // neighbors are valid. This is fine, because motion
            // already has a connection to the tree through
            // nmotion (with populated cost fields!).
            for (std::vector<std::size_t>::const_iterator i = lv.sortedCostIndices.begin();
                    i != lv.sortedCostIndices.begin() + lv.nbh.size(); ++i)
            {
                if (lv.nbh[*i] == nmotion ||
                    ((!useKNearest_ || si_->distance(lv.nbh[*i]->state, motion->state) < maxDistance_) &&
                        si_->checkMotion(lv.nbh[*i]->state, motion->state)))
                {
                    motion->incCost = lv.incCosts[*i];
                    motion->cost = lv.costs[*i];
                    motion->parent = lv.nbh[*i];
                    lv.valid[*i] = 1;
                    break;
                }
                else
                    lv.valid[*i] = -1;
            }
        }
        else  // if not delayCC
        {
            motion->incCost = opt_->motionCost(nmotion->state, motion->state);
            motion->cost = opt_->combineCosts(nmotion->cost, motion->incCost);
            // find which one we connect the new state to
            for (std::size_t i = 0; i < lv.nbh.size(); ++i)
            {
                if (lv.nbh[i] != nmotion)
                {
                    lv.incCosts[i] = opt_->motionCost(lv.nbh[i]->state, motion->state);
                    lv.costs[i] = opt_->combineCosts(lv.nbh[i]->cost, lv.incCosts[i]);
                    if (opt_->isCostBetterThan(lv.costs[i], motion->cost))
                    {
                        if ((!useKNearest_ || si_->distance(lv.nbh[i]->state, motion->state) < maxDistance_) &&
                            si_->checkMotion(lv.nbh[i]->state, motion->state))
                        {
                            motion->incCost = lv.incCosts[i];
                            motion->cost = lv.costs[i];
                            motion->parent = lv.nbh[i];
                            lv.valid[i] = 1;
                        }
                        else
                            lv.valid[i] = -1;
                    }
                }
                else
                {
                    lv.incCosts[i] = motion->incCost;
                    lv.costs[i] = motion->cost;
                    lv.valid[i] = 1;
                }
            }
        }

        

        if (useNewStateRejection_)
        {
            if (opt_->isCostBetterThan(solutionHeuristic(motion), bestCost_))
            {
                nn_->add(motion);
                motion->parent->children.push_back(motion);
                //lv.rmotion = motion;
            }
            else  // If the new motion does not improve the best cost it is ignored.
            {
                si_->freeState(motion->state);
                delete motion;
                return 0; //continue;
            }
        }
        else
        {
            // add motion to the tree
            nn_->add(motion);
            motion->parent->children.push_back(motion);

            //lv.rmotion = motion;
            //return 1;
        }

        bool checkForSolution = false;
        
        // Add the new motion to the goalMotion_ list, if it satisfies the goal
        double distanceFromGoal;
        if (lv.goal->isSatisfied(motion->state, &distanceFromGoal))
        {
            motion->inGoal = true;
            goalMotions_.push_back(motion);
            checkForSolution = true;
        }

        // Checking for solution or iterative improvement
        if (checkForSolution)
        {
            bool updatedSolution = false;
            if (!bestGoalMotion_ && !goalMotions_.empty())
            {
                // We have found our first solution, store it as the best. We only add one
                // vertex at a time, so there can only be one goal vertex at this moment.
                bestGoalMotion_ = goalMotions_.front();
                bestCost_ = bestGoalMotion_->cost;
                updatedSolution = true;

                OMPL_INFORM("%s: Found an initial solution with a cost of %.2f in %u iterations (%u "
                            "vertices in the graph)",
                            getName().c_str(), bestCost_.value(), iterations_, nn_->size());
            }
            else
            {
                // We already have a solution, iterate through the list of goal vertices
                // and see if there's any improvement.
                for (auto &goalMotion : goalMotions_)
                {
                    // Is this goal motion better than the (current) best?
                    if (opt_->isCostBetterThan(goalMotion->cost, bestCost_))
                    {
                        bestGoalMotion_ = goalMotion;
                        bestCost_ = bestGoalMotion_->cost;
                        updatedSolution = true;

                        // Check if it satisfies the optimization objective, if it does, break the for loop
                        if (opt_->isSatisfied(bestCost_))
                        {
                            break;
                        }
                    }
                }
            }

            if (updatedSolution)
            {
                if (useTreePruning_)
                {
                    pruneTree(bestCost_);
                }

                if (lv.intermediateSolutionCallback)
                {
                    std::vector<const base::State *> spath;
                    Motion *intermediate_solution =
                        bestGoalMotion_->parent;  // Do not include goal state to simplify code.

                    // Push back until we find the start, but not the start itself
                    while (intermediate_solution->parent != nullptr)
                    {
                        spath.push_back(intermediate_solution->state);
                        intermediate_solution = intermediate_solution->parent;
                    }

                    lv.intermediateSolutionCallback(this, spath, bestCost_);
                }
            }
        }
        
        // Checking for approximate solution (closest state found to the goal)
        if (goalMotions_.size() == 0 && distanceFromGoal < lv.approxDist)
        {
            lv.approxGoalMotion = motion;
            lv.approxDist = distanceFromGoal;
        }

        motion->stateOrder = motion->parent->stateOrder+1;
        lv.new_motion = motion;
        lv.rmotion = motion;
    } else {
        return 0;
    }
    
    
    // terminate if a sufficient solution is found
    if (bestGoalMotion_ && opt_->isSatisfied(bestCost_)){
        OMPL_INFORM("Best goal is founded");
        lv.finalGoalMotion = bestGoalMotion_;
        return 1; //break;
    }

    return 1;
}


void ompl::geometric::RRTstar::rewire(const base::PlannerTerminationCondition &ptc, LoopVariables& lv, std::vector<LoopVariables>& robots, std::vector<int> index)
{
    //********************
    //********************
    //Rewiring
    //********************
    //********************
    
    bool checkForSolution = false;

    //bring newly added motion in
    auto motion = lv.rmotion; //Could also use rmotion but this is for debugging epoch-wise
    bool motionValid = false;    

    for (std::size_t i = 0; i < lv.nbh.size(); i++)
    {
        motionValid = false;
        motion = lv.rmotion;
        if (lv.nbh[i] != motion->parent)
        {

            base::Cost nbhIncCost;
            if (lv.symCost)
                nbhIncCost = lv.incCosts[i];
            else
                nbhIncCost = opt_->motionCost(motion->state, lv.nbh[i]->state);
            base::Cost nbhNewCost = opt_->combineCosts(motion->cost, nbhIncCost);
            if (opt_->isCostBetterThan(nbhNewCost, lv.nbh[i]->cost))
            {
                //bool motionValid;
                bool noRobotCollision;
                if (lv.valid[i] == 0)
                {
                    motionValid =
                        (!useKNearest_ || si_->distance(lv.nbh[i]->state, motion->state) < maxDistance_) &&
                        checkMotionObject(motion->state, lv.nbh[i]->state);
                    auto temp_state = *lv.nbh[i];

                    // Which state to check if rewiring - this is where ordering matters
                    removeFromParent(&temp_state);
                    temp_state.parent = motion;
                    //temp_state->parent->stateOrder = temp_state->stateOrder+1;
  
                    noRobotCollision = checkRobots(&temp_state, robots, index);
                   
                    //Combine collisions
                    motionValid = motionValid && noRobotCollision;


                }
                else
                {
                    motionValid = (lv.valid[i] == 1);
                }

                if (motionValid)
                {
                    
                    // Remove this node from its parent list
                    removeFromParent(lv.nbh[i]);

                    // Add this node to the new parent
                    lv.nbh[i]->parent = motion;
                    lv.nbh[i]->incCost = nbhIncCost;
                    lv.nbh[i]->cost = nbhNewCost;
                    lv.nbh[i]->parent->children.push_back(lv.nbh[i]);

                    // Update the costs of the node's children
                    updateChildCosts(lv.nbh[i]);

                    checkForSolution = true;
                }
            }
        }
    }
}


ompl::base::PlannerStatus ompl::geometric::RRTstar::solve_end(const base::PlannerTerminationCondition &ptc, LoopVariables& lv)
{

    // Add our solution (if it exists)
    Motion *newSolution = nullptr;
    if (bestGoalMotion_)
    {
        // We have an exact solution
        newSolution = bestGoalMotion_;
    }
    else if (lv.approxGoalMotion)
    {
        // We don't have a solution, but we do have an approximate solution
        newSolution = lv.approxGoalMotion;
    }
    // No else, we have nothing

    // Add what we found
    if (newSolution)
    {
        ptc.terminate();
        // construct the solution path
        std::vector<Motion *> mpath;
        Motion *iterMotion = newSolution;
        while (iterMotion != nullptr)
        {
            mpath.push_back(iterMotion);
            iterMotion = iterMotion->parent;
        }

        // set the solution path
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);

        // Add the solution path.
        base::PlannerSolution psol(path);
        psol.setPlannerName(getName());

        // If we don't have a goal motion, the solution is approximate
        if (!bestGoalMotion_)
            psol.setApproximate(lv.approxDist);

        // Does the solution satisfy the optimization objective?
        psol.setOptimized(opt_, newSolution->cost, opt_->isSatisfied(bestCost_));
        pdef_->addSolutionPath(psol);
    }
    // No else, we have nothing

    si_->freeState(lv.xstate);
    if (lv.rmotion->state)
        si_->freeState(lv.rmotion->state);
    delete lv.rmotion;

    OMPL_INFORM("%s: Created %u new states. Checked %u rewire options. %u goal states in tree. Final solution cost "
                "%.3f",
                getName().c_str(), lv.statesGenerated, lv.rewireTest, goalMotions_.size(), bestCost_.value());

    // We've added a solution if newSolution == true, and it is an approximate solution if bestGoalMotion_ == false
    return {newSolution != nullptr, bestGoalMotion_ == nullptr};
}

ompl::base::PlannerStatus ompl::geometric::RRTstar::solve(const base::PlannerTerminationCondition &ptc)
{
    /*if(!solve_init(ptc)) return base::PlannerStatus::INVALID_START;

    while (ptc == false)
    {
        if(solve_once(ptc) == 1) break;
    }

    return solve_end(ptc);*/
    return base::PlannerStatus::UNKNOWN;
}

void ompl::geometric::RRTstar::getNeighbors(Motion *motion, std::vector<Motion *> &nbh) const
{
    auto cardDbl = static_cast<double>(nn_->size() + 1u);
    if (useKNearest_)
    {
        //- k-nearest RRT*
        unsigned int k = std::ceil(k_rrt_ * log(cardDbl));
        nn_->nearestK(motion, k, nbh);
    }
    else
    {
        double r = std::min(
            maxDistance_, r_rrt_ * std::pow(log(cardDbl) / cardDbl, 1 / static_cast<double>(si_->getStateDimension())));
        nn_->nearestR(motion, r, nbh);
    }
}

void ompl::geometric::RRTstar::removeFromParent(Motion *m)
{
    for (auto it = m->parent->children.begin(); it != m->parent->children.end(); ++it)
    {
        if (*it == m)
        {
            m->parent->children.erase(it);
            break;
        }
    }
}

void ompl::geometric::RRTstar::updateChildCosts(Motion *m)
{
    for (std::size_t i = 0; i < m->children.size(); ++i)
    {
        m->children[i]->cost = opt_->combineCosts(m->cost, m->children[i]->incCost);
        updateChildCosts(m->children[i]);
    }
}

void ompl::geometric::RRTstar::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::RRTstar::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (bestGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(bestGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}

int ompl::geometric::RRTstar::pruneTree(const base::Cost &pruneTreeCost)
{
    // Variable
    // The percent improvement (expressed as a [0,1] fraction) in cost
    double fracBetter;
    // The number pruned
    int numPruned = 0;

    if (opt_->isFinite(prunedCost_))
    {
        fracBetter = std::abs((pruneTreeCost.value() - prunedCost_.value()) / prunedCost_.value());
    }
    else
    {
        fracBetter = 1.0;
    }

    if (fracBetter > pruneThreshold_)
    {
        // We are only pruning motions if they, AND all descendents, have a estimated cost greater than pruneTreeCost
        // The easiest way to do this is to find leaves that should be pruned and ascend up their ancestry until a
        // motion is found that is kept.
        // To avoid making an intermediate copy of the NN structure, we process the tree by descending down from the
        // start(s).
        // In the first pass, all Motions with a cost below pruneTreeCost, or Motion's with children with costs below
        // pruneTreeCost are added to the replacement NN structure,
        // while all other Motions are stored as either a 'leaf' or 'chain' Motion. After all the leaves are
        // disconnected and deleted, we check
        // if any of the the chain Motions are now leaves, and repeat that process until done.
        // This avoids (1) copying the NN structure into an intermediate variable and (2) the use of the expensive
        // NN::remove() method.

        // Variable
        // The queue of Motions to process:
        std::queue<Motion *, std::deque<Motion *>> motionQueue;
        // The list of leaves to prune
        std::queue<Motion *, std::deque<Motion *>> leavesToPrune;
        // The list of chain vertices to recheck after pruning
        std::list<Motion *> chainsToRecheck;

        // Clear the NN structure:
        nn_->clear();

        // Put all the starts into the NN structure and their children into the queue:
        // We do this so that start states are never pruned.
        for (auto &startMotion : startMotions_)
        {
            // Add to the NN
            nn_->add(startMotion);

            // Add their children to the queue:
            addChildrenToList(&motionQueue, startMotion);
        }

        while (motionQueue.empty() == false)
        {
            // Test, can the current motion ever provide a better solution?
            if (keepCondition(motionQueue.front(), pruneTreeCost))
            {
                // Yes it can, so it definitely won't be pruned
                // Add it back into the NN structure
                nn_->add(motionQueue.front());

                // Add it's children to the queue
                addChildrenToList(&motionQueue, motionQueue.front());
            }
            else
            {
                // No it can't, but does it have children?
                if (motionQueue.front()->children.empty() == false)
                {
                    // Yes it does.
                    // We can minimize the number of intermediate chain motions if we check their children
                    // If any of them won't be pruned, then this motion won't either. This intuitively seems
                    // like a nice balance between following the descendents forever.

                    // Variable
                    // Whether the children are definitely to be kept.
                    bool keepAChild = false;

                    // Find if any child is definitely not being pruned.
                    for (unsigned int i = 0u; keepAChild == false && i < motionQueue.front()->children.size(); ++i)
                    {
                        // Test if the child can ever provide a better solution
                        keepAChild = keepCondition(motionQueue.front()->children.at(i), pruneTreeCost);
                    }

                    // Are we *definitely* keeping any of the children?
                    if (keepAChild)
                    {
                        // Yes, we are, so we are not pruning this motion
                        // Add it back into the NN structure.
                        nn_->add(motionQueue.front());
                    }
                    else
                    {
                        // No, we aren't. This doesn't mean we won't though
                        // Move this Motion to the temporary list
                        chainsToRecheck.push_back(motionQueue.front());
                    }

                    // Either way. add it's children to the queue
                    addChildrenToList(&motionQueue, motionQueue.front());
                }
                else
                {
                    // No, so we will be pruning this motion:
                    leavesToPrune.push(motionQueue.front());
                }
            }

            // Pop the iterator, std::list::erase returns the next iterator
            motionQueue.pop();
        }

        // We now have a list of Motions to definitely remove, and a list of Motions to recheck
        // Iteratively check the two lists until there is nothing to to remove
        while (leavesToPrune.empty() == false)
        {
            // First empty the current leaves-to-prune
            while (leavesToPrune.empty() == false)
            {
                // If this leaf is a goal, remove it from the goal set
                if (leavesToPrune.front()->inGoal == true)
                {
                    // Warn if pruning the _best_ goal
                    if (leavesToPrune.front() == bestGoalMotion_)
                    {
                        OMPL_ERROR("%s: Pruning the best goal.", getName().c_str());
                    }
                    // Remove it
                    goalMotions_.erase(std::remove(goalMotions_.begin(), goalMotions_.end(), leavesToPrune.front()),
                                       goalMotions_.end());
                }

                // Remove the leaf from its parent
                removeFromParent(leavesToPrune.front());

                // Erase the actual motion
                // First free the state
                si_->freeState(leavesToPrune.front()->state);

                // then delete the pointer
                delete leavesToPrune.front();

                // And finally remove it from the list, erase returns the next iterator
                leavesToPrune.pop();

                // Update our counter
                ++numPruned;
            }

            // Now, we need to go through the list of chain vertices and see if any are now leaves
            auto mIter = chainsToRecheck.begin();
            while (mIter != chainsToRecheck.end())
            {
                // Is the Motion a leaf?
                if ((*mIter)->children.empty() == true)
                {
                    // It is, add to the removal queue
                    leavesToPrune.push(*mIter);

                    // Remove from this queue, getting the next
                    mIter = chainsToRecheck.erase(mIter);
                }
                else
                {
                    // Is isn't, skip to the next
                    ++mIter;
                }
            }
        }

        // Now finally add back any vertices left in chainsToReheck.
        // These are chain vertices that have descendents that we want to keep
        for (const auto &r : chainsToRecheck)
            // Add the motion back to the NN struct:
            nn_->add(r);

        // All done pruning.
        // Update the cost at which we've pruned:
        prunedCost_ = pruneTreeCost;

        // And if we're using the pruned measure, the measure to which we've pruned
        if (usePrunedMeasure_)
        {
            prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);

            if (useKNearest_ == false)
            {
                calculateRewiringLowerBounds();
            }
        }
        // No else, prunedMeasure_ is the si_ measure by default.
    }

    return numPruned;
}

void ompl::geometric::RRTstar::addChildrenToList(std::queue<Motion *, std::deque<Motion *>> *motionList, Motion *motion)
{
    for (auto &child : motion->children)
    {
        motionList->push(child);
    }
}

bool ompl::geometric::RRTstar::keepCondition(const Motion *motion, const base::Cost &threshold) const
{
    // We keep if the cost-to-come-heuristic of motion is <= threshold, by checking
    // if !(threshold < heuristic), as if b is not better than a, then a is better than, or equal to, b
    if (bestGoalMotion_ && motion == bestGoalMotion_)
    {
        // If the threshold is the theoretical minimum, the bestGoalMotion_ will sometimes fail the test due to floating point precision. Avoid that.
        return true;
    }

    return !opt_->isCostBetterThan(threshold, solutionHeuristic(motion));
}

ompl::base::Cost ompl::geometric::RRTstar::solutionHeuristic(const Motion *motion) const
{
    base::Cost costToCome;
    if (useAdmissibleCostToCome_)
    {
        // Start with infinite cost
        costToCome = opt_->infiniteCost();

        // Find the min from each start
        for (auto &startMotion : startMotions_)
        {
            costToCome = opt_->betterCost(
                costToCome, opt_->motionCost(startMotion->state,
                                             motion->state));  // lower-bounding cost from the start to the state
        }
    }
    else
    {
        costToCome = motion->cost;  // current cost from the state to the goal
    }

    const base::Cost costToGo =
        opt_->costToGo(motion->state, pdef_->getGoal().get());  // lower-bounding cost from the state to the goal
    return opt_->combineCosts(costToCome, costToGo);            // add the two costs
}

void ompl::geometric::RRTstar::setTreePruning(const bool prune)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // If we just disabled tree pruning, but we wee using prunedMeasure, we need to disable that as it required myself
    if (prune == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Store
    useTreePruning_ = prune;
}

void ompl::geometric::RRTstar::setPrunedMeasure(bool informedMeasure)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option only works with informed sampling
    if (informedMeasure == true && (useInformedSampling_ == false || useTreePruning_ == false))
    {
        OMPL_ERROR("%s: InformedMeasure requires InformedSampling and TreePruning.", getName().c_str());
    }

    // Check if we're changed and update parameters if we have:
    if (informedMeasure != usePrunedMeasure_)
    {
        // Store the setting
        usePrunedMeasure_ = informedMeasure;

        // Update the prunedMeasure_ appropriately, if it has been configured.
        if (setup_ == true)
        {
            if (usePrunedMeasure_)
            {
                prunedMeasure_ = infSampler_->getInformedMeasure(prunedCost_);
            }
            else
            {
                prunedMeasure_ = si_->getSpaceMeasure();
            }
        }

        // And either way, update the rewiring radius if necessary
        if (useKNearest_ == false)
        {
            calculateRewiringLowerBounds();
        }
    }
}

void ompl::geometric::RRTstar::setInformedSampling(bool informedSampling)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setSampleRejection, assert that:
    if (informedSampling == true && useRejectionSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // If we just disabled tree pruning, but we are using prunedMeasure, we need to disable that as it required myself
    if (informedSampling == false && getPrunedMeasure() == true)
    {
        setPrunedMeasure(false);
    }

    // Check if we're changing the setting of informed sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (informedSampling != useInformedSampling_)
    {
        // If we're disabled informedSampling, and prunedMeasure is enabled, we need to disable that
        if (informedSampling == false && usePrunedMeasure_ == true)
        {
            setPrunedMeasure(false);
        }

        // Store the value
        useInformedSampling_ = informedSampling;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RRTstar::setSampleRejection(const bool reject)
{
    if (static_cast<bool>(opt_) == true)
    {
        if (opt_->hasCostToGoHeuristic() == false)
        {
            OMPL_INFORM("%s: No cost-to-go heuristic set. Informed techniques will not work well.", getName().c_str());
        }
    }

    // This option is mutually exclusive with setInformedSampling, assert that:
    if (reject == true && useInformedSampling_ == true)
    {
        OMPL_ERROR("%s: InformedSampling and SampleRejection are mutually exclusive options.", getName().c_str());
    }

    // Check if we're changing the setting of rejection sampling. If we are, we will need to create a new sampler, which
    // we only want to do if one is already allocated.
    if (reject != useRejectionSampling_)
    {
        // Store the setting
        useRejectionSampling_ = reject;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RRTstar::setOrderedSampling(bool orderSamples)
{
    // Make sure we're using some type of informed sampling
    if (useInformedSampling_ == false && useRejectionSampling_ == false)
    {
        OMPL_ERROR("%s: OrderedSampling requires either informed sampling or rejection sampling.", getName().c_str());
    }

    // Check if we're changing the setting. If we are, we will need to create a new sampler, which we only want to do if
    // one is already allocated.
    if (orderSamples != useOrderedSampling_)
    {
        // Store the setting
        useOrderedSampling_ = orderSamples;

        // If we currently have a sampler, we need to make a new one
        if (sampler_ || infSampler_)
        {
            // Reset the samplers
            sampler_.reset();
            infSampler_.reset();

            // Create the sampler
            allocSampler();
        }
    }
}

void ompl::geometric::RRTstar::allocSampler()
{
    // Allocate the appropriate type of sampler.
    if (useInformedSampling_)
    {
        // We are using informed sampling, this can end-up reverting to rejection sampling in some cases
        OMPL_INFORM("%s: Using informed sampling.", getName().c_str());
        infSampler_ = opt_->allocInformedStateSampler(pdef_, numSampleAttempts_);
    }
    else if (useRejectionSampling_)
    {
        // We are explicitly using rejection sampling.
        OMPL_INFORM("%s: Using rejection sampling.", getName().c_str());
        infSampler_ = std::make_shared<base::RejectionInfSampler>(pdef_, numSampleAttempts_);
    }
    else
    {
        // We are using a regular sampler
        sampler_ = si_->allocStateSampler();
    }

    // Wrap into a sorted sampler
    if (useOrderedSampling_ == true)
    {
        infSampler_ = std::make_shared<base::OrderedInfSampler>(infSampler_, batchSize_);
    }
    // No else
}

bool ompl::geometric::RRTstar::sampleUniform(base::State *statePtr)
{
    // Use the appropriate sampler
    if (useInformedSampling_ || useRejectionSampling_)
    {
        // Attempt the focused sampler and return the result.
        // If bestCost is changing a lot by small amounts, this could
        // be prunedCost_ to reduce the number of times the informed sampling
        // transforms are recalculated.
        return infSampler_->sampleUniform(statePtr, bestCost_);
    }
    else
    {
        // Simply return a state from the regular sampler
        sampler_->sampleUniform(statePtr);

        // Always true
        return true;
    }
}

void ompl::geometric::RRTstar::calculateRewiringLowerBounds()
{
    const auto dimDbl = static_cast<double>(si_->getStateDimension());

    // k_rrt > 2^(d + 1) * e * (1 + 1 / d).  K-nearest RRT*
    k_rrt_ = rewireFactor_ * (std::pow(2, dimDbl + 1) * boost::math::constants::e<double>() * (1.0 + 1.0 / dimDbl));

    // r_rrt > (2*(1+1/d))^(1/d)*(measure/ballvolume)^(1/d)
    // If we're not using the informed measure, prunedMeasure_ will be set to si_->getSpaceMeasure();
    r_rrt_ =
        rewireFactor_ *
        std::pow(2 * (1.0 + 1.0 / dimDbl) * (prunedMeasure_ / unitNBallMeasure(si_->getStateDimension())), 1.0 / dimDbl);
}
