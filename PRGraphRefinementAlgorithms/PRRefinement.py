from DataStructures.PlanRefinementNode import PlanRefinementNode
from DataStructures.HighLevelPlanGraph import HighLevelPlanGraph
from DataStructures.HighLevelActionSequence import HighLevelAcionSequnce
from Wrappers.ProblemSpecification import ProblemSpecification
from DataStructures.HighLevelProblem import HighLevelProblem
import copy
import util
from Parser.PDDLPlanParser import PDDLPlanParser
from Planner import PlannerFactory
from tmp_exceptions import *
import time
import random
import Config
import numpy as np
from TimePlotting import TimePlotting
import pickle

import networkx as nx
from Queue import PriorityQueue
import pickle
import openravepy

class PRRefinement:
    def __init__(self, plan_refinement_graph,problem_specification):
        self.plan_refinement_graph = plan_refinement_graph
        self.current_pr_refinement_node = None
        self.stack = []
        self.problem_specification = problem_specification
        self.get_parent = False
        self.issue = None
        self.plotter = None

    def dfs(self):
        if self.current_pr_refinement_node is None:
            return self.plan_refinement_graph.get_root_node()
        else:
            children = self.current_pr_refinement_node.get_children()
            if len(children) > 0 and self.current_pr_refinement_node.next_child_count < len(
                    children) and not self.get_parent:
                self.current_pr_refinement_node.next_child_count += 1
                return children[self.current_pr_refinement_node.next_child_count - 1]
            else:
                self.get_parent = False
                if self.issue == "timeout":
                    if self.current_pr_refinement_node.get_parent() is None:
                        return self.current_pr_refinement_node
                return self.current_pr_refinement_node.get_parent()

    def cost_batch_dfs(self):
        if self.current_pr_refinement_node is None:
            return self.plan_refinement_graph.get_root_node()
        else:
            children = self.current_pr_refinement_node.get_children()
            if len(children) > 0 and self.current_pr_refinement_node.next_child_count < len(
                    children) and not self.get_parent:
                self.current_pr_refinement_node.next_child_count += 1
                if self.current_pr_refinement_node.child_queue is None:
                    self.current_pr_refinement_node.child_queue = PriorityQueue()
                    for child in children:
                        self.current_pr_refinement_node.child_queue.put((child.hl_plan_tree.cost,child))
                return self.current_pr_refinement_node.child_queue.get()[1]
            else:
                self.get_parent = False
                if self.issue == "timeout":
                    if self.current_pr_refinement_node.get_parent() is None:
                        return self.current_pr_refinement_node
                return self.current_pr_refinement_node.get_parent()

    def random(self):
        return None

    def select_pr_node_for_refinment(self):
        print "SELECTING NEW PR NODE TO REFINE..."
        #TODO PR node selection algorithm goes here
        if Config.PR_STRATEGY == 'dfs':
            return self.dfs()
        elif Config.PR_STRATEGY == "random":
            return self.random()
        elif Config.PR_STRATEGY == "cost_batch_dfs":
            return self.cost_batch_dfs()
        else:
            return self.dfs()


    def update_ll_state(self,backtrack = False):
        if backtrack:
            if self.current_pr_refinement_node.last_refined_hl_action_node is None:
                pass
            else:
                self.current_pr_refinement_node.last_refined_hl_action_node.ll_state.values = copy.deepcopy(self.current_pr_refinement_node.last_refined_hl_action_node.init_ll_values)
                self.current_pr_refinement_node.last_refined_hl_action_node.ll_state.sync_simulator()
        else:
            if self.current_pr_refinement_node.last_refined_hl_plan_graph_node is None:
                self.current_pr_refinement_node.hl_plan_tree.get_root().ll_state.values = copy.deepcopy(self.current_pr_refinement_node.hl_plan_tree.get_root().ll_state.values)
                self.current_pr_refinement_node.hl_plan_tree.get_root().ll_state.sync_simulator()
            else:
                # self.current_pr_refinement_node.last_refined_hl_plan_graph_node.ll_state.values = copy.deepcopy(self.current_pr_refinement_node.last_refined_hl_plan_graph_node.refined_ll_values)
                self.current_pr_refinement_node.last_refined_hl_plan_graph_node.ll_state.sync_simulator()
        pass





    def run(self):


        while True:
            self.last_refined_pr_node = self.current_pr_refinement_node
            self.current_pr_refinement_node = self.select_pr_node_for_refinment()

            if self.current_pr_refinement_node is None:
                break

        # If we don't have a High Level plan in this pr node, create it
            if self.current_pr_refinement_node.hl_plan_tree is None:

                hl_plan_graph = self.create_HL_plan_graph(self.current_pr_refinement_node.problem_specification)
                self.current_pr_refinement_node.hl_plan_tree = hl_plan_graph
            else:
                self.update_ll_state(backtrack=False)
                # pass

            leafQueue = self.current_pr_refinement_node.prepare_queue(new = True)
            if Config.PLOT:
                if self.plotter is None:
                    self.plotter = TimePlotting.TimePlotter(self.plan_refinement_graph)
            success = False
            totalSuccess = False
            error_mode = np.random.choice(["backtrack", "partial_traj"],
                                          p=[Config.BACKTRACK_PROB, 1 - Config.BACKTRACK_PROB])

            while leafQueue.qsize() > 0:
                print "Sequences in Leaf Queue : {}".format(leafQueue.qsize())
                temp = leafQueue.get()[1]
                success = False

                self.current_pr_refinement_node.set_hl_plan_graph(temp)
                hl_action_node = None
                mode = "err_free"
                flag_new_pr = False
                self.current_pr_refinement_node.start_time = time.time()
                while not success:
                    try:
                        if mode == "err_free":
                            '''
                            Error free mode code here
                            '''
                            success, mode, failed_hl_plan_node = self.current_pr_refinement_node.try_refine(
                                hl_action_node=hl_action_node, \
                                mode=mode, \
                                resource_limit=30)
                            revisit = False
                            if success:
                                totalSuccess = True
                                break
                            else:
                                totalSuccess = False
                        if error_mode == "partial_traj" and not success:
                            '''
                            Partial Traj mode code here
                            '''
                            if not revisit:
                                batch_count = 0
                                original_hl_plan_tree = self.current_pr_refinement_node.hl_plan_tree
                                hl_plan_tree_copy = copy.deepcopy(self.current_pr_refinement_node.hl_plan_tree)
                                hl_plan_tree_copy.recalculate_all_edges()

                                self.current_pr_refinement_node.hl_plan_tree = hl_plan_tree_copy

                                newleafQueue = self.current_pr_refinement_node.prepare_queue(new=True)

                                temp = newleafQueue.get()[1]

                                self.current_pr_refinement_node.set_hl_plan_graph(temp)
                                refine_from_hl_node = None

                            ## Check the deep copy for hlpg_node_ref getting None.
                            ## Run in the partial traj mode and replan
                            new_task_plan_found = False

                            while not new_task_plan_found:
                                Config.enablePrint()
                                print "*****ERRx_MODE*****"
                                Config.blockprint()
                                # try:
                                try:
                                    success, failed_predicates, failed_hl_plan_node = self.current_pr_refinement_node.try_refine(
                                        hl_action_node=refine_from_hl_node,
                                        mode=mode,
                                        resource_limit=20)
                                except OutOfPossibleErrorsException, e:
                                    failed_hl_plan_node = e.failure_node
                                    flag_new_pr = True
                                    issue = "search_over"
                                    break
                                Config.enablePrint()
                                print "****NEW_PLAN*****"
                                Config.blockprint()
                                # Create the new HL plan graph after incorporating errors
                                pddl_problem_file = self.current_pr_refinement_node.problem_specification.pddl_problem_file
                                pddl_domain_file = self.current_pr_refinement_node.problem_specification.pddl_domain_file

                                try:
                                    new_pddl_problem = util.incorporate_errors_to_pddl(pddl_problem_file,
                                                                                       copy.deepcopy(failed_hl_plan_node.hl_state),
                                                                                       failed_predicates)
                                except StandardError, e:
                                    flag_new_pr = True
                                    issue = "no_error"
                                    break
                                else:
                                    new_problem_spec = ProblemSpecification(pddl_domain_file=pddl_domain_file,
                                                                            pddl_problem_file=new_pddl_problem,
                                                                            hl_state_type=self.current_pr_refinement_node.hl_state_type,
                                                                            ll_state_type=self.current_pr_refinement_node.problem_specification.ll_state_type,
                                                                            # initial_ll_state=failed_hl_plan_node.ll_state,
                                                                            # initial_hl_state=failed_hl_plan_node.hl_state,
                                                                            hl_planner_name=self.current_pr_refinement_node.problem_specification.hl_planner_name,
                                                                            ll_planner_name=self.current_pr_refinement_node.problem_specification.ll_planner_name)
                                    try:
                                        new_hl_plan_tree = self.create_HL_plan_graph(new_problem_spec)
                                    except StandardError,e:
                                        print "Plan not found"
                                        print e
                                        new_task_plan_found = False
                                    else:
                                        new_task_plan_found = True

                            if new_task_plan_found:
                                if failed_hl_plan_node.get_parent() is None:
                                    copy_policy_tree = new_hl_plan_tree
                                    parent_of_new_hl_plan_graph = None
                                else:
                                    parent_of_new_hl_plan_graph = failed_hl_plan_node.get_parent().hlpg_node_ref
                                    copy_policy_tree = hl_plan_tree_copy.merge(parent_of_new_hl_plan_graph, new_hl_plan_tree,
                                                            failed_hl_plan_node.hlpg_node_ref,
                                                            "ERR_Label")  # What should the err label be?

                                # Update the candidate for refinement
                                if parent_of_new_hl_plan_graph is not None:
                                    ll_state_values = parent_of_new_hl_plan_graph.ll_state.get_values_from_env(
                                        parent_of_new_hl_plan_graph.ll_state.simulator.env)
                                else:
                                    ll_state_values = hl_plan_tree_copy.get_root().ll_state.get_values_from_env(
                                        hl_plan_tree_copy.get_root().ll_state.simulator.env)
                                new_pr_node_candidate = PlanRefinementNode(problem_specification=new_problem_spec,
                                                                           hl_plan_tree= copy_policy_tree,
                                                                           last_refined_hl_plan_graph_node=parent_of_new_hl_plan_graph,
                                                                           ll_state_values=ll_state_values)
                                self.current_pr_refinement_node.add_child(new_pr_node_candidate)
                                self.current_pr_refinement_node.hl_plan_tree = original_hl_plan_tree
                                new_pr_node_candidate.set_parent(self.current_pr_refinement_node)
                                if batch_count < Config.K-1:
                                    mode = "partial_traj"
                                    revisit = True
                                    self.current_pr_refinement_node.hl_plan_tree = hl_plan_tree_copy
                                    refine_from_hl_node = failed_hl_plan_node
                                    refine_from_hl_node.reset_child_generator()
                                    batch_count +=1
                                else:
                                    flag_new_pr = True
                                    issue = "misc"
                                    break

                        if error_mode == "backtrack" and not success:
                            '''
                            Backtracking code here
                            '''
                            Config.enablePrint()
                            print "backtracking to the parent.."
                            Config.blockprint()
                            hl_action_node = failed_hl_plan_node.get_parent()
                            self.update_ll_state(backtrack=True)
                            if hl_action_node is not None:
                                hl_action_node.reset_child_generator()
                                failed_hl_plan_node.reset_child_generator()
                                self.reset_plan_tree(hl_action_node)
                            else:
                                failed_hl_plan_node.reset_child_generator()
                                self.reset_plan_tree(failed_hl_plan_node)
                            leafQueue = self.current_pr_refinement_node.prepare_queue(new=True)
                            mode = "err_free"

                    except TimeOutException as e:
                        flag_new_pr = True
                        failed_hl_plan_node = e.hl_action_node
                        issue = "timeout"
                        break

                if flag_new_pr:
                    self.current_pr_refinement_node.last_refining_node = failed_hl_plan_node
                    self.current_pr_refinement_node.mode = mode
                    self.current_pr_refinement_node.visited = True
                    self.issue = issue
                    break

                if Config.PLOT:
                    if self.plotter is not None:
                        self.plotter.update(self.current_pr_refinement_node)


            if totalSuccess:
                print "Done..Exit?"
                self.successful_pr_node = self.current_pr_refinement_node
                print("End Time: ",time.time())
                print("*** ")
                break

        if success:
            if Config.PLOT:
                if self.plotter is not None:
                    self.plotter.generate_plot()
            if Config.RUN_TRAJ:
                self.run_plan(self.successful_pr_node)
            # raw_input("Done..Exit?")
            print "Solved.."
        else:
            print "Cant FInd solution"




    def reset_plan_tree(self,hl_action_node):
        hlpg_ref_node = hl_action_node.hlpg_node_ref
        queue = [hlpg_ref_node]
        parent_ll_state_cpy = copy.deepcopy(hlpg_ref_node.ll_state)
        while len(queue) > 0:
            node = queue.pop(0)
            node.ll_state = None
            node.ll_plan = None
            for child in node.get_children():
                e = self.current_pr_refinement_node.hl_plan_tree.get_edge(node,child)
                e.refined_ll_values = None
                e.refined_ll_state = None
                e.generated_ll_values = None
                e.ll_plan = None
                queue.append(child)
        hlpg_ref_node.ll_state = copy.deepcopy(parent_ll_state_cpy)


    def create_HL_plan_graph(self, problem_specification):
        hl_solution,state_list = self.get_HL_Solution(problem_specification)
        if hl_solution is None:
            raise StandardError

        else:

            hl_plan_graph = HighLevelPlanGraph(hl_solution, problem_specification,state_list)
            ll_state = problem_specification.ll_state_type()
            if self.current_pr_refinement_node.ll_state_values is None:
                self.current_pr_refinement_node.ll_state_values = ll_state.get_values_from_env(ll_state.simulator.env)
            hl_plan_graph.get_root().set_ll_state(ll_state)
            return hl_plan_graph

    def get_HL_Solution(self,problem_specification):
        hl_problem = HighLevelProblem(problem_specification.pddl_domain_file, problem_specification.pddl_problem_file)
        planner = PlannerFactory.create(problem_specification.hl_planner_name)
        hl_solution,state_list = planner.solve(hl_problem,problem_specification)
        return hl_solution,state_list

    # def create_HL_plan_tree(self,problem_specification):
    #     hl_problem = HighLevelProblem(problem_specification.pddl_domain_file, problem_specification.pddl_problem_file)
    #     planner = PlannerFactory.create(problem_specification.hl_planner_name)
    #     hl_solution = planner.solve(hl_problem)
    #     if hl_solution.success is False:
    #         assert False,"Can not find a HL Plan"
    #     else:
    #         pass
    #
    #
    #     return None
    #

    def run_plan(self, pr_node_refinement_candidate):

        root = pr_node_refinement_candidate.hl_plan_tree.get_root()
        simulator = root.ll_state.simulator
        # simulator.env.SetViewer('RViz')
        root.ll_state.sync_simulator()
        nodeq = [root]
        trajectory_pickle = []
        while len(nodeq) > 0:
            node = nodeq.pop()
            children = node.get_children()
            if len(children) > 0:
                if len(children) > 1:
                    choice = int(raw_input("Enter 0 to make error, 1 not to: "))
                    if choice == 1:
                        fail = False
                    else:
                        fail = True
                    nodes = children[:]
                    if fail:
                        index = None
                        for i in range(len(nodes)):
                            hl_state = nodes[i].hl_state
                            b = False
                            for pred in hl_state.getTrueProps():
                                # if pred[1:3] == "at" and "about" not in pred:
                                if "ingripper" in pred:
                                    b = True
                                    break
                            if not b:
                                index = i
                    else:
                        index = None
                        for i in range(len(nodes)):
                            hl_state = nodes[i].hl_state
                            b = False
                            for pred in hl_state.getTrueProps():
                                # if pred[1:3] == "at" and "about" not in pred:
                                if "ingripper" in pred:
                                    b = True
                                    break
                            if b:
                                index = i
                    child = children[index]
                else:
                    fail = False
                    child = children[0]
                edge = pr_node_refinement_candidate.hl_plan_tree.get_edge(node,child)
                if edge.has_mp:
                    state = node.ll_state
                    # pr_node_refinement_candidate.update_ll_state_execute(node,state)
                    ll_plan = edge.ll_plan
                    all_generated = edge.generated_values
                    exec_sequence = edge.ll_action_spec.exec_sequence

                    for arg in exec_sequence:
                        t = ll_plan[arg]
                        simulator.execute(t["type"],t["value"],all_generated)
                        trajectory_pickle.append([t["type"],t["value"],all_generated])
                    
                nodeq.append(child)
            else:
                break
        traj_dump = open(Config.PROJ_DIR+"/real_world_exec/openrave_trajectories.pickle","wb")
        pickle.dump(trajectory_pickle, traj_dump)
        traj_dump.close()
