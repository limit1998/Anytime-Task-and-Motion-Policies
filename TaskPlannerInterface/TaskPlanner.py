class TaskPlanner(object):
    def __init__(self, *args, **kwargs):
        self.pddlDomainFile = kwargs['pddlDomainFile']
        self.pddlProblemFile = kwargs['pddlProblemFile']
        self.outputFile = kwargs['outputFile']


    def computeNewPlan(self):
        return "No planner defined"
