import subprocess
import pickle
# sys.path.append(os.path.dirname(os.path.dirname(os.path.realpath(__file__))))

from Object import Object
from Predicates.Precondition import Precondition
from Wrappers.Effect import Effect
from Wrappers.Argument import Argument
import ActionConfig
from Functions.Function import Function


class InputParser(object):
    __instance = None

    @staticmethod
    def getInstance():
        if InputParser.__instance == None:
            raise Exception("InputParser Singleton not initialized!")
        return InputParser.__instance

    def __init__(self, hlDomainFile=None, hlProblemFile=None, llDomainFile=None, llProblemFile=None,
                 actionConfigFile=None, action_str=None):
        self.hlDomainFile = hlDomainFile
        self.hlProblemFile = hlProblemFile
        self.llDomainFile = llDomainFile
        self.llProblemFile = llProblemFile
        self.actionConfigFile = actionConfigFile
        # Todo: Action is composed of action and arguments. Seperate each of them
        self.action_str = action_str
        self.invokeParser()
        InputParser.__instance = self

    # ToDo: Implementation
    def invokeParser(self):
        parserFile = ["./python3Parser.py", "--domain", self.hlDomainFile, "--problem", self.hlProblemFile, "--action",
                      self.action_str]
        process = subprocess.Popen(parserFile, shell=False, stdout=subprocess.PIPE)
        obj, _ = process.communicate()
        print(obj)

    def getActionConfig(self,action_name):
        return ActionConfig.config_map[action_name]


    def parseHLUniverse(self):
        '''
        Implement Here
        '''
        # self.invokeParser()
        universeObjects = pickle.load(open("objects_in_the_universe.p", "rb"))
        list_universe_objects = []
        for item in universeObjects:
            list_universe_objects.append(Object(None, item, universeObjects[item]))

        return list_universe_objects


    def parseLLUniverse(self):
        universeObjects = pickle.load(open("objects_in_the_universe.p", "rb"))
        list_universe_objects = []
        for item in universeObjects:
            list_universe_objects.append(Object(None, item, universeObjects[item]))

        return list_universe_objects


    def parseHLFunctions(self):
        '''
        Implement HERE
        '''
        # self.invokeParser()
        predicates = pickle.load(open("predicates.p", "rb"))
        return predicates
        return True


    def parseLLFunctions(self):
        #OpenRave Specific to load Geometry and Poses
        #Geometry -- Dimensions
        #Poses -- Translations and Rotations
        
        env = Environment() # create openrave environment
        env.Load('SampleTasks/openrave-aair-lab.env.xml') # load a simple scene
        bodies = env.GetBodies()

        list_of_functions = []
        argument = []
        value = []
        for items in bodies:
            argument.append('object', type(items.GetTransformPose()))
            value.append(items.GetTransformPose())
        list_of_functions.append(Function('pose',argument,value))

        return list_of_functions

        '''list_universe_objects = self.parseLLUniverse()
        for objects in list_universe_objects:
            for items in bodies:
                if objects.getObjectname()==items.GetName():
                    objects.setGeometry(None)
                    objects.setPose(items.GetTransformPose())
                    break
                else:
                    objects.setGeometry(None)
                    objects.setPose(None)
        
        return list_universe_objects'''


    def getActionDefinition(self):
        # self.invokeParser()

        Precondition
        pos_precondition = pickle.load(open("pos_precondition.p", "rb"))
        neg_precondition = pickle.load(open("neg_precondition.p", "rb"))
        precondition = Precondition(pos_precondition, neg_precondition)

        # Effect
        pos_effect = pickle.load(open("pos_effect.p", "rb"))
        neg_effect = pickle.load(open("neg_effect.p", "rb"))
        effect = Effect(pos_effect, neg_effect)

        argumentList = pickle.load(open("argument.p", "rb"))
        list_argument_objects = []
        for item in argumentList:
            list_argument_objects.append(Argument(item[0], item[1], item[2]))

        return precondition, effect, list_argument_objects


parser = InputParser(hlDomainFile='/home/fetchc/Desktop/Kislay/TMP_Merged/SampleTasks/cup-washer.pddl', hlProblemFile='/home/fetchc/Desktop/Kislay/TMP_Merged/SampleTasks/cup-washer.problem', action_str='pick clorox')
obj = parser.parseLLFunctions()
for items in obj:
    print items.getPose()