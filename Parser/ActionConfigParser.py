from Wrappers.Argument import Argument
from Predicates.Predicate import Predicate
from Predicates import CanWorldPredicates
from LLActionSpec import LLActionSpec
from Wrappers.Effect import Effect
import json
from Precondition.Precondition import Precondition



class ActionConfigParser(object):

    def __init__(self, action_config_file):
        with open(action_config_file) as f:
            self.action_config = json.load(f)

        self.__gen_type_to_class_map = self.__parse_gen_type_to_class_map()
        self.__predicate_to_class_map = self.__parse_predicate_to_class_map()
        self.__action_obj_map = self.__parse_ll_action_specs()

    def get_action_config_map(self):
        return self.__action_obj_map

    def get_gen_type_to_class_map(self):
        return self.__gen_type_to_class_map

    def get_precondition_to_class_map(self):
        return self.__predicate_to_class_map

    def get_action_config(self, action_name):
        return self.__action_obj_map.get(action_name)

    def __parse_gen_type_to_class_map(self):
        return self.action_config['generator_type_to_class_map']

    def __parse_predicate_to_class_map(self):
        return self.action_config['predicate_to_class_map']

    def __parse_ll_action_specs(self):
        action_obj_map = {}
        config_map = self.action_config['config_map']
        for action in config_map:
            action_spec = config_map[action]
            action_obj = self.__parse_action(action, action_spec)
            action_obj_map[action] = action_obj

        return action_obj_map

    def __parse_action(self, action_name, action_spec_map):

        ll_gen_sequence = action_spec_map['LL_GEN_SEQUENCE']
        arg_type_map = action_spec_map['LLARGS']
        predicates = self.__parse_predicates(action_spec_map['PRECONDITIONS'], arg_type_map)
        precondition = Precondition(predicates)
        effects = self.__parse_effects(action_spec_map['EFFECTS'])
        depender_map = action_spec_map.get('GEN_DEP',[]) #depender -> [dependees]
        dependee_map = {}

        for k in depender_map:
            for v in depender_map.get(k):
                if v in dependee_map:
                    dependee_map[v].append(k)
                else:
                    dependee_map[v] = [k]


        return LLActionSpec(action_name,
                            precondition,
                            effects,
                            ll_gen_sequence,
                            arg_type_map,
                            depender_map,
                            dependee_map,
                            self.__gen_type_to_class_map)

    def __parse_effects(self, effects):
        effects_obj_list = []
        for effect in effects:
            effect_obj = Effect(None, None)
            effects_obj_list.append(effect_obj)
        return effects_obj_list

    def __parse_predicates(self, predicates, arg_gen_type_map={}):
        '''Returns a list of predicate objects'''
        predicate_obj_list = []
        for predicate in predicates:

            arg_object_list = None
            if '(' in predicate:
                open_paren_index = predicate.index('(')
                predicate_name = predicate[0:open_paren_index]
                args = predicate[open_paren_index + 1:-1].split(',')
                for arg in args:
                    arg_obj = Argument(arg, arg_gen_type_map[arg])
                    if arg_object_list is None:
                        arg_object_list = []
                    arg_object_list.append(arg_obj)

            else:
                predicate_name = predicate

            predicate_spec_name = self.__predicate_to_class_map.get(predicate_name)
            predicate_class_str = None
            if predicate_spec_name:
                predicate_class_str = predicate_spec_name.get('class')
            if predicate_class_str:
                predicate_obj = getattr(CanWorldPredicates, predicate_class_str)(predicate_name, arg_object_list)
            else:
                predicate_obj = Predicate(predicate_name, arg_object_list)
            predicate_obj_list.append(predicate_obj)
        return predicate_obj_list
