# import copy
# from Generator import Generator
#
# class PreGraspPoseGenerator(Generator):
#     def __init__(self, ll_state=None, obj_name=None):
#         super(PreGraspPoseGenerator, self).__init__()
#         self.__object_name = obj_name
#         self.__ll_state = ll_state
#         self.__yielder = None
#         self.type = "OBJECT_POSE"
#
#     def __deepcopy__(self, memodict={}):
#         obj_name_cpy = copy.deepcopy(self.__object_name)
#         ll_state_cpy = copy.deepcopy(self.__ll_state)
#         dependencies_cpy = copy.deepcopy(self.dependencies)
#         gpg_cpy = PreGraspPoseGenerator(ll_state_cpy, obj_name_cpy)
#         gpg_cpy.dependencies = dependencies_cpy
#         return gpg_cpy
#
#     def configure(self, args=None, ll_state=None):
#         super(PreGraspPoseGenerator, self).configure()
#         self.__ll_state = ll_state
#         last_underscore_loc = args[1].rfind('_')
#         if last_underscore_loc != -1:
#             obj_name = args[1][last_underscore_loc + 1:]
#             self.__object_name = obj_name
#
#     def reset(self):
#         super(PreGraspPoseGenerator, self).reset()
#         self.__yielder = None
#
#     def get_next(self, required_values_map=None):
#         grasp_pose = required_values_map.get(GraspPoseGenerator)
#         # if self.__yielder is None:
#         self.__yielder = self.__generate(grasp_pose)
#         genvalue = self.__yielder.next()
#         self.store(genvalue)
#         return genvalue
#
#     def __generate(self, grasp_pose):
#         lst = OpenRaveHelper.generate_pre_grasp_pose(self.__ll_state, self.__object_name, grasp_pose)
#         for gt in lst:
#             yield gt
