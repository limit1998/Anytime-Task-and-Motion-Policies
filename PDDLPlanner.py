from Planner import Planner
from TaskPlannerInterface.FastForwardTaskPlanner import FastForwardTaskPlanner

class PDDLPLanner(Planner):

    def __init__(self, plannerName, hlDomainFile, hlProblemFile, outputFile):
        self.plannerName = plannerName
        self.outputFile = outputFile
        self.hlDomainFile = hlDomainFile
        self.hlProblemFile = hlProblemFile
        super(PDDLPLanner,self).__init__(self.plannerName)

    def call(self):
        if self.plannerName=='ff':
            planner = FastForwardTaskPlanner(self.plannerName, self.hlDomainFile, self.hlProblemFile, self.outputFile)
            planStrFileH, rawOut, planCount = planner.getResult()
            return planStrFileH, rawOut, planCount
        else:
            pass