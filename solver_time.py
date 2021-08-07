
import sys
import json
import math

from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def print_solution(data, manager, routing, solution):
    """Prints solution on console."""
    print(f'Objective: {solution.ObjectiveValue()}')
    time_dimension = routing.GetDimensionOrDie('Time')
    total_time = 0
    for vehicle_id in range(data['num_vehicles']):
        index = routing.Start(vehicle_id)
        plan_output = 'Route for vehicle {}:\n'.format(vehicle_id)
        while not routing.IsEnd(index):
            time_var = time_dimension.CumulVar(index)
            plan_output += '{0} Time({1},{2}) -> '.format(
                manager.IndexToNode(index), solution.Min(time_var),
                solution.Max(time_var))
            index = solution.Value(routing.NextVar(index))
        time_var = time_dimension.CumulVar(index)
        plan_output += '{0} Time({1},{2})\n'.format(manager.IndexToNode(index),
                                                    solution.Min(time_var),
                                                    solution.Max(time_var))
        plan_output += 'Time of the route: {}min\n'.format(
            solution.Min(time_var))
        print(plan_output)
        total_time += solution.Min(time_var)
    print('Total time of all routes: {}min'.format(total_time))

def get_routes(solution, routing, manager):
    """Get vehicle routes from a solution and store them in an array."""
    # Get vehicle routes and store them in a two dimensional array whose
    # i,j entry is the jth location visited by vehicle i along its route.
    routes = []
    for route_nbr in range(routing.vehicles()):
        index = routing.Start(route_nbr)
        route = [manager.IndexToNode(index)]
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            route.append(manager.IndexToNode(index))
        routes.append(route)
    return routes

def get_cumul_data(solution, routing, dimension):
    """Get cumulative data from a dimension and store it in an array."""
    # Returns an array cumul_data whose i,j entry contains the minimum and
    # maximum of CumulVar for the dimension at the jth node on route :
    # - cumul_data[i][j][0] is the minimum.
    # - cumul_data[i][j][1] is the maximum.

    cumul_data = []
    for route_nbr in range(routing.vehicles()):
        route_data = []
        index = routing.Start(route_nbr)
        dim_var = dimension.CumulVar(index)
        route_data.append([solution.Min(dim_var), solution.Max(dim_var)])
        while not routing.IsEnd(index):
            index = solution.Value(routing.NextVar(index))
            dim_var = dimension.CumulVar(index)
            route_data.append([solution.Min(dim_var), solution.Max(dim_var)])

        cumul_data.append(route_data)

    return cumul_data


def main():
    """Entry point of the program."""
    # Instantiate the data problem.
    data = json.loads(sys.argv[1])

    # Create the routing index manager.
    manager = pywrapcp.RoutingIndexManager(len(data['time_matrix']),
                                           data['num_vehicles'],
                                           data['starts'],
                                           data['ends'])

    # Create Routing Model.
    routing = pywrapcp.RoutingModel(manager)

    # Create and register a transit callback.
    def time_callback(from_index, to_index):
        """Returns the travel time between the two nodes."""
        # Convert from routing variable Index to time matrix NodeIndex.
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        service_time = 0 if from_node in data['starts'] else data['service_times'][from_node]
        time = data['time_matrix'][from_node][to_node] + service_time
        return time

    transit_callback_index = routing.RegisterTransitCallback(time_callback)

    # Define cost of each arc.
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    # Add Time Windows constraint.
    time = 'Time'
    routing.AddDimension(
        transit_callback_index,
        2592000,  # allow waiting time
        2592000,  # maximum time per vehicle
        False,  # Don't force start cumul to zero.
        time)
    time_dimension = routing.GetDimensionOrDie(time)


    # Add time window constraints for each location except depot.
    for location_idx, time_window in enumerate(data['time_windows']):
        if location_idx in data['starts']:
            continue

        if location_idx in data['ends']:
            continue

        index = manager.NodeToIndex(location_idx)
        time_dimension.CumulVar(index).SetRange(time_window[0], time_window[1])

    # Add time window constraints for each vehicle start node.
 #   depot_idx = data['starts'][0]
 #   for vehicle_id in range(data['num_vehicles']):
 #       index = routing.Start(vehicle_id)
 #       time_dimension.CumulVar(index).SetRange(data['time_windows'][depot_idx][0], data['time_windows'][depot_idx][1])

    # Instantiate route start and end times to produce feasible times.
    for i in range(data['num_vehicles']):
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.Start(i)))
        routing.AddVariableMinimizedByFinalizer(time_dimension.CumulVar(routing.End(i)))

    # Priority
    if 'high_priority_nodes' in data:
        # Create a "counter" dimension
        routing.AddConstantDimension(
          1, # add one at each node
          999999999, # max
          True, # start to 0
          'counter' # name of this dimension
        )
        counter = routing.GetDimensionOrDie('counter')

        # Priority.
        for node in data['high_priority_nodes']:
            index = manager.NodeToIndex(node)
            counter.SetCumulVarSoftUpperBound(index, len(data['high_priority_nodes']), 60 * 60)


    # Allow to drop nodes.
    penalty = 100000
    for node in range(0, len(data['time_matrix'])):
        index = manager.NodeToIndex(node)
        if index != -1:
            routing.AddDisjunction([index], penalty)


    # Locks
    if 'locks' in data:
        result = list(map(lambda x: manager.NodeToIndex(x), data['locks']))
        routing.ApplyLocks(result)

    # Setting first solution heuristic.
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # Solve the problem.
    solution = routing.SolveWithParameters(search_parameters)

    # Print solution on console.
    if solution:
      #  print_solution(data, manager, routing, solution)
        routes = get_routes(solution, routing, manager)
     #   windows = get_cumul_data(solution, routing, time_dimension)
        print(json.dumps(routes))
    else:
        print('No solution found !')

if __name__ == '__main__':
    main()
