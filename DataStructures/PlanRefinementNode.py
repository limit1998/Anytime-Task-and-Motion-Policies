from GraphNode import GraphNode
import sys
from Planner import PlannerFactory
import random
import Config
from Queue import PriorityQueue
from DataStructures.HighLevelActionSequence import HighLevelAcionSequnce
import networkx as nx
from tmp_exceptions import *
import copy
from openravepy import *

class PlanRefinementNode(GraphNode):
    def __init__(self, problem_specification=None,hl_plan_graph=None, hl_plan_tree= None, parent=None, last_refined_hl_plan_graph_node=None, \
                 lqueue = None,ll_state_values=None,mode = "err_free",visited = True,start_time = None):
        super(PlanRefinementNode,self).__init__()
        self.__parent = parent
        self.__children = []
        self.problem_specification = problem_specification
        self.hl_plan_graph = hl_plan_graph
        self.lqueue = lqueue
        self.next_child_count = 0
        self.hl_plan_tree = hl_plan_tree
        self.hl_state_type = Config.HL_STATE_TYPE
        self.last_refined_hl_plan_graph_node = last_refined_hl_plan_graph_node
        self.ll_state_values = ll_state_values
        self.mode = mode
        self.visited = visited
        self.start_time = start_time
        self.last_refined_hl_action_node = None
        self.child_queue = None



    def prepare_queue(self,new = False):
        if self.lqueue is None or new:
            root = self.hl_plan_tree.get_root()
            self.lqueue = PriorityQueue()
            for node in self.hl_plan_tree.graph.nodes:
                if self.hl_plan_tree.graph.out_degree[node] == 0:
                    path = nx.shortest_path(self.hl_plan_tree.graph,root,node)
                    last_edge = self.hl_plan_tree.get_edge(path[-2],path[-1])
                    if last_edge.ll_plan is not None:
                        continue
                    hlas = HighLevelAcionSequnce(path)
                    prob = last_edge.prob
                    if prob == 0:
                        for action in path:
                            print action.hl_action
                        exit(0)

                    p = len(path) / float(prob)
                    self.lqueue.put((p,hlas))
        if self.lqueue.qsize() > 1:
            pass

        return self.lqueue

    def set_parent(self, node):
        self.__parent = node

    def set_hl_plan_graph(self, hl_plan_graph):
        self.hl_plan_graph = hl_plan_graph

    def get_children(self):
        return self.__children
    def get_parent(self):
        return self.__parent

    def add_child(self, child):
        self.__children.append(child)

    def try_refine(self,hl_action_node=None, mode="err_free", resource_limit=10,force_start_from_begining=False):
        while True:
            if resource_limit == 0:
                return False,"timeout",None
            reset_ll_sepc = False
            if hl_action_node is None:
                self.hl_plan_graph.reset_child_generators()
                hl_action_node = self.hl_plan_graph.get_root()
                reset_ll_sepc = True
            hl_action_node.hlpg_node_ref.ll_state.sync_simulator()
            child = hl_action_node.generate_child()
            if child is not None:
                edge = self.hl_plan_tree.get_edge(hl_action_node.hlpg_node_ref,child.hlpg_node_ref)
            else:
                print "Finished for this sequence..."
                return True,None,None

            if edge.ll_action_spec is None:
                ll_state = copy.deepcopy(hl_action_node.hlpg_node_ref.ll_state)
                edge.refined_ll_state = ll_state
                child.set_ll_state(ll_state)
                child.hlpg_node_ref.set_ll_state(ll_state)
                hl_action_node = child
                continue

            if edge.ll_plan is None:
                if reset_ll_sepc:
                    edge.ll_action_spec.reset_argument_generators()
                if hl_action_node.get_parent() is None:
                    resource_limit -= 1
                hl_action = edge.hl_action
                hl_action.hl_arguments['robot'] = Config.ROBOT_NAME
                failed_predicates = None
                ll_complete_action_spec =  edge.ll_action_spec
                hl_state = hl_action_node.hl_state
                child_hl_state = child.hl_state
                ll_complete_action_spec.generated_values["current_hl_state"] = hl_state
                ll_complete_action_spec.generated_values["next_hl_state"] = child_hl_state
                ll_state = copy.deepcopy(hl_action_node.hlpg_node_ref.ll_state)
                ll_state_values = ll_state.get_values_from_env()
                hl_action_node.init_ll_values = copy.deepcopy(ll_state_values)
                hl_action_node.hlpg_node_ref.init_ll_values = copy.deepcopy(ll_state_values)
                ll_argument_generator = ll_complete_action_spec.generator(low_level_state=ll_state,
                                                                          backtrack_on_failure=(mode == 'err_free'),
                                                                          start_time=self.start_time)
                try:
                    result,generated_values, next_ll_state = ll_argument_generator.next()
                    ll_plan = result#TODO
                    edge.has_mp = True
                    edge.ll_plan = ll_plan
                    edge.generated_values = generated_values
                    edge.refined_ll_values = copy.deepcopy(ll_state.get_values_from_env(ll_state.simulator.env))
                    edge.refined_ll_state = copy.deepcopy(next_ll_state)
                    self.stable_ll_state = copy.deepcopy(next_ll_state)
                    self.last_refined_hl_action_node = hl_action_node
                    child.set_ll_state(copy.deepcopy(next_ll_state))
                    child.hlpg_node_ref.set_ll_state(copy.deepcopy(next_ll_state))
                    hl_action_node = child
                except FailedPredicateException as failed_predicate_excep:
                    # ll_state = ll_state_cpy
                    ll_state.values = copy.deepcopy(ll_state_values)
                    ll_state.sync_simulator(ll_state_values)
                    failed_predicate = failed_predicate_excep.predicate
                    if failed_predicate is not None:
                        failed_predicate.set_proposition_string(failed_predicate_excep.failure_reason)
                    last_successful_state = failed_predicate_excep.last_successful_state
                    print "Failed At: " + str(hl_action)
                    return False, failed_predicate, hl_action_node

                except OutOfPossibleErrorsException:
                    # ll_state = ll_state_cpy
                    ll_state.values = copy.deepcopy(ll_state_values)
                    ll_state.sync_simulator()
                    ll_complete_action_spec.reset_argument_generators()
                    return False, None, hl_action_node

                except TimeOutException:
                    raise TimeOutException(hl_action_node=hl_action_node)
            else:
                if child.ll_state is None:
                    child.set_ll_state(copy.deepcopy(edge.refined_ll_state))
                    child.hlpg_node_ref.set_ll_state(copy.deepcopy(edge.refined_ll_state))
                    edge.refined_ll_state.sync_simulator()
                hl_action_node = child

    # def try_refine(self, hl_action_node=None, mode='err_free', resource_limit=10 ,force_start_from_beginning=False):
    #     '''
    #     :return success, failure_node
    #     '''
    #
    #     while True:
    #         if resource_limit == 0:
    #             return False,"timeout",None
    #
    #         if hl_action_node is None:
    #             self.hl_plan_graph.reset_child_generators()
    #             hl_action_node = self.hl_plan_graph.get_root()
    #
    #             if hl_action_node.hlpg_node_ref.ll_plan is None:
    #                 if hl_action_node.hlpg_node_ref.ll_action_spec:
    #                     hl_action_node.hlpg_node_ref.ll_action_spec.reset_argument_generators()
    #             hl_action_node.hlpg_node_ref.ll_state.sync_simulator()
    #         else:
    #             if hl_action_node.get_parent() is not None:
    #                 hl_action_node.get_parent().hlpg_node_ref.ll_state.sync_simulator(self.hl_plan_tree.get_refined_ll_values(hl_action_node.hlpg_node_ref.get_parent(),hl_action_node.hlpg_node_ref))
    #
    #
    #         if hl_action_node.hlpg_node_ref.ll_action_spec is None: #If there is no corresponding ll_action spec for the action, don't refine
    #             ll_state = hl_action_node.ll_state
    #             hl_action_node = hl_action_node.generate_child()
    #             if hl_action_node is None:
    #                 return True, None, None
    #
    #             hl_action_node.set_ll_state(ll_state)
    #             continue;
    #
    #         if not self.hl_plan_tree.is_refined(hl_action_node.hlpg_node_ref):
    #
    #             if hl_action_node.get_parent() is None:
    #                 resource_limit = resource_limit - 1
    #
    #             hl_action = hl_action_node.hl_action
    #
    #             hl_action.hl_arguments['robot'] = Config.ROBOT_NAME
    #             print hl_action
    #             # #######TEST############
    #             # if hl_action.actionName == 'moveto':
    #             #     hl_action.hl_arguments = {u'object_name':'object0'}
    #             #
    #             # elif hl_action.actionName == 'grasp':
    #             #     hl_action.hl_arguments = {u'object_name':'object0', u'robot':'fetch'}
    #             #
    #             # ######################
    #
    #             failed_predicates = None
    #             ll_complete_action_spec = hl_action_node.hlpg_node_ref.ll_action_spec
    #             ll_state = copy.deepcopy(hl_action_node.ll_state)
    #             hl_action_node.hlpg_node_ref.set_ll_state(copy.deepcopy(ll_state))
    #             current_hl_state = hl_action_node.hlpg_node_ref.hl_state
    #             child = hl_action_node.generate_child()
    #             if child is not None:
    #                 next_hl_state = child.hlpg_node_ref.hl_state
    #             else:
    #                 next_hl_state = None
    #             ll_complete_action_spec.generated_values["current_hl_state"] = current_hl_state
    #             ll_complete_action_spec.generated_values["next_hl_state"] = next_hl_state
    #
    #             ll_argument_generator = ll_complete_action_spec.generator(low_level_state=ll_state,
    #                                                                       backtrack_on_failure=(mode == 'err_free'), start_time = self.start_time)
    #             ll_state_values = ll_state.get_values_from_env()
    #             ll_state.set_values(ll_state_values)
    #             hl_action_node.init_ll_values = copy.deepcopy(ll_state_values)
    #             hl_action_node.hlpg_node_ref.init_ll_values = copy.deepcopy(ll_state_values)
    #             try:
    #                 result,generated_values, next_ll_state = ll_argument_generator.next() #TODO need resulting_state and ll_plan
    #                 # NOTE: Once we have all the argument values, update the low level state, simulate forward in time.
    #                 # Currently this is implicitly done during argument generation
    #
    #                 ll_plan = result#TODO
    #                 hl_action_node.has_mp = True
    #                 hl_action_node.hlpg_node_ref.has_mp = True
    #
    #                 # if mode == "partial_traj":
    #                 #     raise FailedPredicateException(None,failure_reason = None)
    #
    #                 hl_action_node.hlpg_node_ref.save(ll_plan, next_ll_state)
    #                 hl_action_node.save(ll_plan, next_ll_state)
    #                 hl_action_node.hlpg_node_ref.generated_values = generated_values
    #                 # hl_action_node.generated_values = generated_values
    #
    #                 refined_ll_values = copy.deepcopy(ll_state.get_values_from_env(ll_state.simulator.env))
    #                 self.hl_plan_tree.store_mp(hl_action_node.hlpg_node_ref,child.hlpg_node_ref,generated_values)
    #                 self.hl_plan_tree.store_refined_ll_values(hl_action_node.hlpg_node_ref,child.hlpg_node_ref,refined_ll_values)
    #                 # hl_action_node.hlpg_node_ref.refined_ll_values = copy.deepcopy(ll_state.get_values_from_env(ll_state.simulator.env))
    #
    #                 self.stable_ll_state = copy.deepcopy(next_ll_state)
    #                 self.last_refined_hl_action_node = hl_action_node
    #
    #
    #                 # child = hl_action_node.generate_child()
    #                 if child is not None:
    #                     child.set_ll_state(copy.deepcopy(next_ll_state))
    #                     child.hlpg_node_ref.set_ll_state(copy.deepcopy(next_ll_state))
    #                     hl_action_node = child
    #
    #                     # return self.try_refine(child, mode, resource_limit)
    #                 else:
    #                     print("Finished for this sequence...")
    #                     return True, None,None  # "Finished"
    #
    #             except FailedPredicateException as failed_predicate_excep:
    #                 # ll_state = ll_state_cpy
    #                 ll_state.values = copy.deepcopy(ll_state_values)
    #                 ll_state.sync_simulator(ll_state_values)
    #                 failed_predicate = failed_predicate_excep.predicate
    #                 if failed_predicate is not None:
    #                     failed_predicate.set_proposition_string(failed_predicate_excep.failure_reason)
    #                 last_successful_state = failed_predicate_excep.last_successful_state #TODO need last_successful_state
    #                 print "Failed At: " + str(hl_action)
    #                 return False, failed_predicate, hl_action_node
    #
    #             except OutOfPossibleErrorsException:
    #                 # ll_state = ll_state_cpy
    #                 ll_state.values = copy.deepcopy(ll_state_values)
    #                 ll_state.sync_simulator()
    #                 hl_action_node.hlpg_node_ref.ll_action_spec.reset_argument_generators()
    #                 return False, None, hl_action_node
    #
    #             except TimeOutException:
    #                 raise TimeOutException(hl_action_node=hl_action_node)
    #
    #         else:
    #             child = hl_action_node.generate_child()
    #             if child is not None:
    #                 if child.ll_state is None:
    #                     child.set_ll_state(copy.deepcopy(hl_action_node.hlpg_node_ref.ll_state))
    #                     # child.hlpg_node_ref.set_ll_state(hl_action_node.hlpg_node_ref.ll_state)
    #                 ll_state_values = hl_action_node.hlpg_node_ref.refined_ll_values
    #                 ll_state = hl_action_node.ll_state
    #                 ll_state.sync_simulator(ll_state_values)
    #                 hl_action_node = child
    #