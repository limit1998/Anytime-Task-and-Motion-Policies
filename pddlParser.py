#!/usr/bin/env python3

import pddlpy
import argparse
import sys





def parser(domainFile, problemFile):
	domainProblem = pddlpy.DomainProblem(domainFile, problemFile)
	return domainProblem.worldobjects()


if __name__ == '__main__':
	list_objects = parser(args.domain, args.problem)
	sys.stdout.write(str(list_objects))