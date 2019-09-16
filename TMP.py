import argparse
from DataStructures.PlanRefinementNode import PlanRefinementNode
from DataStructures.PlanRefinementGraph import PlanRefinementGraph
from PRGraphRefinementAlgorithms.PRRefinement import PRRefinement
from Wrappers.ProblemSpecification import ProblemSpecification
import Config
import States
import random
import numpy as np
import time


class TMP(object):
    def __init__(self, args=None):
        random.seed(int(time.time()))
        np.random.seed(int(time.time()))
        problem_spec = ProblemSpecification(pddl_domain_file=Config.DEFAULT_PDDL_FILE,
                                            pddl_problem_file=Config.DEFAULT_PROBLEM_FILE,
                                            ll_state_type=Config.LL_STATE_TYPE,
                                            hl_state_type=Config.HL_STATE_TYPE,
                                            hl_planner_name=Config.HL_PLANNER,
                                            ll_planner_name=Config.OPENRAVE_PLANNER)

        initial_pr_node = PlanRefinementNode(problem_specification=problem_spec)

        plan_refinement_graph = PlanRefinementGraph(initial_pr_node)

        PRRefinement(plan_refinement_graph,problem_spec).run()



if __name__ == "__main__":

    # parser = argparse.ArgumentParser()
    # parser.add_argument('--domain', type=str, help='PDDL Domain File', default=Config.DEFAULT_PDDL_FILE)
    # parser.add_argument('--problem', type=str, help='PDDL problem File', default=Config.DEFAULT_PROBLEM_FILE)
    # parser.add_argument('--lldomain', type=str, help='Low Level PDDL or Simulator environment XML')
    # parser.add_argument('--plannerName', type=str, help='High Level Planner', default='ff')
    # parser.add_argument('--outputFile', type=str, help='Plan Output File', default=Config.DEFAULT_OUTPUT_FILE)
    # args = parser.parse_args()
    Config.setPaths()
    import time
    start_time = time.time()
    print(start_time)
    Config.blockprint()
    tmp = TMP()
    Config.enablePrint()
    Config.resetPaths()
    print("--- %s seconds ---" % (time.time() - start_time))
    # raw_input("Exit?")
    # exit()
