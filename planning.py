'''
Description: planning and smoothing for all benchmark tasks and planner
'''
from IPPerfMonitor import IPPerfMonitor
import ResultCollection

def planning(plannerFactory, testList):
    '''
    This function performs planning and smoothing for all benchmark tasks and planner.\n
    :param plannerFactory: List of all planers and their configurations\n
    :param testList: List of all Benchmark tasks\n
    '''
    resultList = list()
    for key,producer in list(plannerFactory.items()):
        print(key, producer)
        for benchmark in testList:
            print ("Planning: " + key + " - " + benchmark.name)
            planner = producer[0](benchmark.collisionChecker)
            IPPerfMonitor.clearData()
            try:
                resultList.append(ResultCollection.ResultCollection(key,
                                            planner, 
                                            benchmark, 
                                            planner.planPath(benchmark.startList,benchmark.goalList,producer[1]),
                                            IPPerfMonitor.dataFrame()
                                            ),
                            )
                if(resultList[-1].solution == []):
                    print(f"no path found {resultList[-1].plannerFactoryName} {benchmark.name}")
            except Exception as e:
                print ("PLANNING ERROR ! ", e)
                pass
    return resultList