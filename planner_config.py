'''
Description: Configurations for all planners
'''
import IPBasicPRM
import IPVISBasicPRM

import IPVisibilityPRM
import IPVISVisibilityPRM

import IPLazyPRM
import IPVISLazyPRM

plannerFactory = dict()

# Basic PRM ----------------------------------------

basicConfig = dict()
basicConfig["radius"] = 8
basicConfig["numNodes"] = 600
plannerFactory["basePRM"] = [IPBasicPRM.BasicPRM, basicConfig, IPVISBasicPRM.basicPRMVisualize]

# Visibility PRM -----------------------------------

visbilityConfig = dict()
visbilityConfig["ntry"] = 600
plannerFactory["visibilityPRM"] = [IPVisibilityPRM.VisPRM, visbilityConfig, IPVISVisibilityPRM.visibilityPRMVisualize ]

# Lazy PRM -----------------------------------------

lazyConfig = dict()
lazyConfig["initialRoadmapSize"] = 40
lazyConfig["updateRoadmapSize"]  = 10
lazyConfig["kNearest"] = 12
lazyConfig["maxIterations"] = 50
plannerFactory["lazyPRM"] = [IPLazyPRM.LazyPRM, lazyConfig, IPVISLazyPRM.lazyPRMVisualize]

# -------------------------------------------------