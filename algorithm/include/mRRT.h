#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MRRTSTAR_

#include "PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/PPM.h>
#include <vector>


#include "RRTstar.h"
#include "PathSimplifier.h"

namespace ompl
{
    namespace geometric
    {
        class mRRT : public base::Planner
        {
        public:
            mRRT(const base::SpaceInformationPtr &si);

            ~mRRT() override;
            void getPlannerData(base::PlannerData &data) const override;

            base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;

            std::vector<std::vector<ompl::base::State*>> apply_rsc(std::vector<std::vector<ompl::base::State*>> paths);
            
            std::vector<std::vector<double>> apply_shortcut(std::vector<ompl::base::State*> a, std::vector<double> b, std::vector<ompl::base::State*> c);
            
            void clear() override;

            void setup() override;

        protected:
            std::vector<ompl::base::PlannerPtr> v_rrtStar_;

            ompl::PPM ppm_;

            bool haveSolutionPath(int idx);
            ompl::geometric::PathGeometric getSolutionPath(int idx);

            void recordSolution(int idx, unsigned char red, unsigned char green, unsigned char blue);


            void save(const char *filename);

        };
    }
}

#endif