#include "route_planner.h"
#include <algorithm>

/* b/c &model is a reference & references must use initialization lists,
   m_Model is initialized to model just before this function runs */
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    /* FindClosestNode function is in route_model.cpp
         RoutePlanner's start_node and end_node variables are
         declared in route_planner.h */
      start_node = &m_Model.FindClosestNode(start_x, start_y);
      end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
// route_model.h has h_value and distance variables.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node); // distance is in route_model.h
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value.
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.
// FindNeighbors() is in route_model.cpp

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  current_node->FindNeighbors();
 /* See route_model.h for all the Node variables.
    See Part 3 - Writing Multifile Programs
    See headings "Pointers to Other Object Types" &
    "Single File Code"
    Using range-based for loop.  See heading "Practice" and under line
    "void AddOneToEach(vector<int> &v)"
    neighbors is in route_model.h */
  
  for (auto& neighbor_node: current_node->neighbors) {
    neighbor_node->parent = current_node;
    neighbor_node->g_value = current_node->g_value
                              + neighbor_node->distance(*current_node);
    neighbor_node->h_value = CalculateHValue(neighbor_node);
    neighbor_node->visited = true;
    open_list.push_back(neighbor_node);  // open_list is in route_planner.h
  }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
/* See Part 2 - A* Search
   See heading "To Complete This Exercise" at the bottom
   See code written under line // Get the next node
   sort() sorts nodes in open_list in descending order, so node with lowest
   f is the last element in the open_list vector.
   Instead of using Compare() which requires creating a struct Compare, I
   used the Lambda function which has the format [ ]( ){ }.  Inside the
   square brackets [ ], we include the values to capture from the current
   scope.  Inside () are the arguments that get passed to the function,
   inside {} is the body of the function.  The function is return f1 > f2,
   where f = g + h */
  std::sort(open_list.begin(), open_list.end(), []
            (const RouteModel::Node* node1, const RouteModel::Node* node2)
            {return node1->g_value + node1->h_value >
             node2->g_value + node2->h_value;});
  /* See Part 3 - Writing Multifile Programs
     See heading "Pointers to Other Object Types"
     current_node points to address of last element in open_list */
  auto* current_node = open_list.back();
  /* See Part 3 - Writing Multifile Programs
     See heading "Returning a Pointer from a Function"
     pop_back() removes last element from open_list */
  open_list.pop_back();
  return current_node;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Video #8 discusses this method
    distance = 0.0f;
    /* Create path_found vector
       path_found is a vector of nodes from start to finish */
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.push_back(*current_node);
    // parent is in route_model.h
      while (current_node != start_node) {
      path_found.push_back(*current_node->parent);
      distance += current_node->distance(*current_node->parent);
      current_node = current_node->parent;
    }
    /* From "Part 2 - A* Search" under heading "To Complete This
       Exercise"
       parent of current_node will be last element, so last
       element of path_found will be start_node, so you need to
       reverse path_found so that start_node will be 1st element
       and current_node will be last element */
    reverse(path_found.begin(), path_found.end());
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale();
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
  start_node->visited = true;
  // emplace_back is like push_back but avoids unnecessary copying
  open_list.emplace_back(start_node);
  AddNeighbors(start_node);
  current_node = NextNode();

  while (current_node != end_node) {
    AddNeighbors(current_node);
    current_node = NextNode();
  }
  
  // if current_node = end_node
  m_Model.path = ConstructFinalPath(current_node);
}
