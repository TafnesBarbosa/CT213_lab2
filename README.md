# Path Planning
This project uses three kind of path planning algorithms to find a path between 2 points in a virtual map with obstacles. Below, there are some pictures showing their functioning.

## Dijkstra
Using the very known Dijkstra algorithm is possible to find a optimal path between two points.

![image](https://user-images.githubusercontent.com/119422200/227800590-9eca9652-2ec5-4854-9f6f-61d5c0d2155d.png)

## Greedy Search
Using greedy search with only the euclidian distance as heuristic, is possible to find a suboptimal path between two points faster than Dijkstra algorithm.

![image](https://user-images.githubusercontent.com/119422200/227800637-c16e2456-4b56-4282-a423-45794e82930c.png)

## A*
This algorithm is faster than Dikstra and slower than greedy search, but it finds an optimal path when using a consistent heuristic (euclidian distance) with the cost of the nodes.

![image](https://user-images.githubusercontent.com/119422200/227800719-11f686ed-d2e8-4e75-ad1d-35548f5faac0.png)
