#ifndef OMPL_GEOMETRIC_PLANNERS_RRT_MRRTSTAR_
#define OMPL_GEOMETRIC_PLANNERS_RRT_MRRTSTAR_


#include "PlannerIncludes.h"
#include "ompl/base/OptimizationObjective.h"
#include "ompl/datastructures/NearestNeighbors.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/PPM.h>
#include <vector>


#include "RRTstar.h"

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

            void clear() override;

            void setup() override;

        protected:
            std::vector<ompl::base::PlannerPtr> v_rrtStar_;

            ompl::PPM ppm_;

            std::vector<std::vector<std::vector<double>>> borrowMotion;

            bool threadExit = true;

            bool haveSolutionPath(int idx);
            ompl::geometric::PathGeometric getSolutionPath(int idx);

            void recordSolution(int idx, unsigned char red, unsigned char green, unsigned char blue);


            void save(const char *filename);
            
            void saveFiles(const char *filename);

            void writeToCSV(std::vector<ompl::geometric::RRTstar::LoopVariables> v_lv, std::ofstream& myfile, std::ofstream& timefile);
            void checkGoals(std::vector<ompl::geometric::RRTstar::LoopVariables> &v_lv);
        };
    }
}

#endif