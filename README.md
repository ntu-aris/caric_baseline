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

    
