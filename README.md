# caric_baseline

## caric_baseline
This repo is for CDC2023 challange caric_baseline. The challenge details can be find in 
https://ntu-aris.github.io/caric/
The main task can be divided into two part, the Task Assignment Part and the Swarm Planning Part. Until 2023/08/23 the baseline is just for mbs task, we wish it can help the participants to solve some techinical problem.
## Update Log
2023/08/23 Update the mbs baseline 
## Quick Demo
    Pull down this repo. Catin build and launch the demo_paths.launch in this repo.

## Task Assignment Part
    Notice: The code here is not a general method for all subtasks. Until 2023/08/13 the code is only for mbs tasks.
### Methodology
  The task assignment means " who will search which bounding box" and it also can be simplified as a Multiple Traveling Salesman Problem(MTSP). In the baseline, we just use an exhaustive search to solve this problem. One drawback of this approach is that it incurs significant time costs when dealing with a large number of bounding boxes, as it requires extensive efforts to seek solutions for the problem. In certain scenarios, such as the crane scene, obtaining a solution can take as long as 25 minutes. Therefore, participants are encouraged to engage in further contemplation and algorithmic enhancements.
### Code Example
    xxh_initial_task.h/cpp
## Swarm planning part
### Methodology
    In the baseline approach, due to the mismatch between the capabilities of the explorer and the photographer, a grouping strategy is employed during task initialization based on the number of explorers. The positioning control of the target bounding box follows a strategy of exploring along the longest edge before scanning. Greedy strategies are utilized for controlling the gimbal orientation and the orientation of the agent. Upon completion of the task, a return to the starting point is executed.
### Code Example
    baseline_planner.h/cpp

    
