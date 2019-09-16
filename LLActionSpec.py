from DataStructures.LowLevelPlanGraph import LowLevelDependencyGraph
from DataStructures.LowLevelProblem import LowLevelProblem
import copy
import util


class LLActionSpec(object):

    def __init__(self, name, precondition, effect, ll_gen_sequence=None, arg_to_generator_type_map=None, depender_map={}, dependee_map={}, gen_type_to_class_map=None):
        self.name = name
        self.precondition = precondition
        self.effect = effect
        self.ll_dep_graph = LowLevelDependencyGraph(ll_gen_sequence, arg_to_generator_type_map, depender_map, dependee_map,
                                                    gen_type_to_class_map)

    def __deepcopy__(self, memodict={}):
        name_cpy = copy.deepcopy(self.name)
        preconditions_cpy = copy.deepcopy(self.precondition)
        eff_cpy = copy.deepcopy(self.effect)
        ll_action_cpy = LLActionSpec(name_cpy, preconditions_cpy, eff_cpy)
        ll_action_cpy.ll_dep_graph = copy.deepcopy(self.ll_dep_graph)
        return ll_action_cpy

    def get_failed_predicates_in_precondition(self, arg_val_map, ll_state_for_eval, planner, hl_action):
        failed_predicates = []
        for predicate in self.precondition.get_predicates():
            if predicate.evaluate(ll_state_for_eval, planner, arg_val_map, hl_action.hl_arguments) is False:
                failed_predicates.append(predicate)
        return failed_predicates

    def __recursive_configure(self,generator,hl_arguments, ll_state):
        stack = util.Stack()
        while generator is not None:
            while generator.dependencies is not None:
                generators = generator.dependencies
                for g in generators:
                    if not g.is_configured:
                        stack.push(generator)
                        generator = g
                        break;
                else:
                    break
            generator.configure(hl_arguments, ll_state)
            generator = stack.pop()

    def __recursive_get_next(self, generator):
        stack = util.Stack()
        visited = []
        while generator is not None:
            while generator.dependencies is not None:
                generators = generator.dependencies
                for g in generators:
                    if g not in visited:
                        stack.push(generator)
                        generator = g
                        break;
                else:
                    break
            required_values = {}
            for g in generator.dependencies:
                required_values[type(g)] = g.get_current()
            gen_value = generator.get_next(required_values)
            visited.append(generator)
            generator = stack.pop()
        return gen_value

    def __recursive_get_current(self, generator):
        stack = util.Stack()
        visited = []
        while generator is not None:
            while generator.dependencies is not None:
                generators = generator.dependencies
                for g in generators:
                    if g not in visited:
                        stack.push(generator)
                        generator = g
                        break;
                else:
                    break
            gen_value = generator.get_current()
            visited.append(generator)
            generator = stack.pop()
        return gen_value

    def reset_all_generators(self):
        node = self.ll_dep_graph.get_root()
        while node is not None:
            node.generator.reset()
            node = node.get_child()

    def get_LL_problem_generator(self, hl_arguments, ll_state):
        print "\tGenerating--"
        while True:
            root_node = self.ll_dep_graph.get_root()
            ll_problems = []
            node = root_node
            flow_through = True
            arg_to_gen_values_map = {}
            while node is not None:
                print "\t\tGen: "+node.arg,
                if not node.is_generator_configured():
                    print "--configuring--",
                    assert ll_state != None, "Cannot generate LL problem , ll_state is none for " + self.name
                    # self.__recursive_configure(node, hl_arguments, ll_state)
                    node.configure_generator(hl_arguments, ll_state)
                    flow_through = False
                try:
                    child = node.get_child()
                    if not flow_through or child is None:
                        genvalue = node.get_new_value(arg_to_gen_values_map)
                        arg_to_gen_values_map[type(node.generator)] = genvalue
                    else:
                        genvalue = node.get_current_value()
                        arg_to_gen_values_map[type(node.generator)] = genvalue

                    generated_type = node.generator.type
                    dofs = node.generator.dofs
                    # ll_state = ll_state.apply_value_to_state(genvalue, generated_type)
                    ll_problem = LowLevelProblem(hl_arguments, genvalue, generated_type, node.arg, dofs=dofs)
                    ll_problems.append(ll_problem)
                    node = child
                    print ""
                except StopIteration:
                    print "-->No More, Backup"
                    node.reset_generator()
                    if node == root_node:
                        yield None
                    else:
                        # TODO: There are 2 bugs here
                        # TODO BUG1 Consider pgp, gp, lp. If lp fails, we should not change gp, since pgp
                        # TODO: depends on gp. We must go up to the pgp node, change that and recompute
                        # TODO: BUG2: when we compute pgp, we also compute gp, however, this code does not use the
                        # TODO: precomputed value of gp when it reaches gp node, but computes a new value, this is WRONG
                        ll_problems.pop()
                        parent = node.get_parent()
                        node = parent

                        # dependees = parent.dependees
                        # dep, pos_int = self.__get_dependee_with_lowest_position(parent)
                        # if not dep:
                        #     dep, pos_int = parent, parent.position_int
                        # if pos_int != parent.position_int:
                        #     node = node.get_parent()
                        #
                        # while node.position_int != pos_int:
                        #     ll_problems.pop()
                        #     if node.arg not in dependees:
                        #         node.reset_generator()
                        #     node = node.get_parent()
                        flow_through = False

            yield ll_problems
