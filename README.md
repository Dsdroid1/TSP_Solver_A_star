# Assignment 2 (AI) : TSP solver

Assignment 2 for AI course, implementation of a TSP solver using A\* with MST as the heuristic.

### Steps to execute

The program is written in python, to execute it, run
`python main.py`

The program will then ask for the input filename, which contains the graph information in a predefined format.

### Input file format

The first line will contain the number of nodes in the graph. The subsequent lines contain the edge information in the following format:
`i j cost` or `i,j,cost` where `i` and `j` are the nodes for the corresponding edge
To use the comma separated format, input file should have an extension of `csv`.[See sample input files]

### Heuristic Used

Consider that the graph has the set of nodes G and we have already visited some set V from it. Then, as the next node in our tour, we pick some node 'n'. To evaluate the heuristic in such a situation, we use:

- The cost of the minimum spanning tree(MST) of the set of nodes `G-(V+n)`
- The cost of visiting the closest node in the MST from the node n
- The cost of completing the tour from the MST, i.e. the cost of the closest node in the MST from the source vertex 's'

All these costs are added to get the heuristic cost. This heuristic dominates the simple MST cost and hence is better. It is still admissible as we relax the proper tour condition and take the minimum such costs.

### Output format

We show the progress of A\*, i.e. which state has been picked for expansion and what nodes were visited(in which order) to reach the current state, along with the f-cost of that state.
This allows us to track the progress of A\* and check the f-contours being expanded.
When a goal node is popped from the fringe list, we calculate the solution and display the entire path, along with its cost. We also report the statistics about the number of nodes that were generated and those which were expanded.
