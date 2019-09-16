import TaskPlanner

# ######
# from src.aair_ros_pkg.src.Utils import FileUtils
# import src.aair_ros_pkg.src.Utils.CommandLineUtils as CommandLineUtils
# from src.aair_ros_pkg.src.TaskPlannerInterface.TaskPlannerOutputParser import TaskPlannerOutputParser
# import src.aair_ros_pkg.src.Config as Config
# #####



###---When Running from ROS-----
from Utils import FileUtils
import Utils.CommandLineUtils as CommandLineUtils
from TaskPlannerOutputParser import TaskPlannerOutputParser
import Config as Config
###

class FastForwardTaskPlanner(object):
    def __init__(self, plannerName, hlDomainFile, hlProblemFile, outputFile):
#        super(FastForwardTaskPlanner, self).__init__(*args, **kwargs)
        self.pddlDomainFile = hlDomainFile
        self.pddlProblemFile = hlProblemFile
        self.outputFile = outputFile
        self.successStr = 'found legal plan as follows'

    def __runPlanner(self):
        commandStr = Config.FastForwardExe + " -o " + self.pddlDomainFile + " -f " + self.pddlProblemFile

        retVal = CommandLineUtils.executeCommand(commandStr, self.successStr, self.outputFile)
        if retVal == -1:
            return -1, -1, -1

        rawOutput = FileUtils.read(self.outputFile)
        ffOutStr = TaskPlannerOutputParser(rawOutput, "ff").getFFPlan()
        return ffOutStr, rawOutput, 1

    def getResult(self):
        planStr, rawOut, planCount = self.__runPlanner()
        if planStr == -1:
            return -1, -1, -1
        planStrF = FileUtils.getStringIOFile(planStr)
        return planStrF, rawOut, planCount





if __name__ == "__main__":
    ffPlanner = FastForwardTaskPlanner(plannerName='ff',
                                        hlDomainFile=Config.DEFAULT_PDDL_FILE,
                                       hlProblemFile=Config.DEFAULT_PROBLEM_FILE,
                                       outputFile=Config.DEFAULT_OUTPUT_FILE,
                                       )

    a,b,c = ffPlanner.getResult();
    print a
    print "*********"
    print b
    print "*********"
    print c
    # ffOutStr, rawOutput, retVal = ffPlanner.runPlanner()
    # print ffOutStr