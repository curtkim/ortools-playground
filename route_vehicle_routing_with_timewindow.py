import math
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2

def create_data_array():

  locations = [[820, 760], [960, 440], [500, 50], [490, 80], [130, 70], [290, 890], [580, 300], [840, 390], [140, 240], [120, 390],
               [30, 820], [50, 100], [980, 520], [840, 250], [610, 590], [10, 650], [880, 510], [910, 20], [190, 320], [930, 30],
               [500, 930], [980, 140], [50, 420], [420, 90], [610, 620], [90, 970], [800, 550], [570, 690], [230, 150], [200, 700],
               [850, 600], [980, 50]]

  demands =  [0, 19, 21, 6, 19, 7, 12, 16, 6, 16,
              8, 14, 21, 16, 3, 22, 18, 19, 1, 24,
              8, 12, 4, 8, 24, 24, 2, 20, 15, 2,
              14, 9]

  start_times =  [0, 508, 103, 493, 225, 531, 89, 565, 540, 108,
                  602, 466, 356, 303, 399, 382, 362, 521, 23, 489,
                  445, 318, 380, 55, 574, 515, 110, 310, 387, 491,
                  328, 73]

  # tw_duration is the width of the time windows.
  tw_duration = 2150

  # In this example, the width is the same at each location, so we define the end times to be
  # start times + tw_duration. For problems in which the time window widths vary by location,
  # you can explicitly define the list of end_times, as we have done for start_times.
  end_times = [0] * len(start_times)

  for i in range(len(start_times)):
    end_times[i] = start_times[i] + tw_duration
  data = [locations, demands, start_times, end_times]
  return data

'''
Total distance of all routes: 10560

Route 0:
  0 Load(0) Time(0, 0) ->
  27 Load(0) Time(320, 330) ->
  18 Load(20) Time(1130, 1140) ->
  8 Load(21) Time(1263, 1273) ->
  11 Load(27) Time(1511, 1521) ->
  4 Load(41) Time(1663, 1673) ->
  28 Load(60) Time(1900, 1910) ->
  23 Load(75) Time(2195, 2205) ->
  3 Load(83) Time(2299, 2643) ->
  0 Load(89) Time(3327, 86400)


Route 1:
  0 Load(0) Time(0, 0) ->
  21 Load(0) Time(780, 978) ->
  31 Load(12) Time(906, 1104) ->
  19 Load(21) Time(1003, 1201) ->
  17 Load(45) Time(1105, 1303) ->
  2 Load(64) Time(1602, 1800) ->
  6 Load(85) Time(1995, 2193) ->
  14 Load(97) Time(2351, 2549) ->
  0 Load(100) Time(2740, 86400)


Route 2:
  0 Load(0) Time(0, 0) ->
  20 Load(0) Time(490, 684) ->
  5 Load(8) Time(764, 958) ->
  25 Load(15) Time(1065, 1259) ->
  10 Load(39) Time(1347, 1541) ->
  15 Load(47) Time(1561, 1755) ->
  22 Load(69) Time(1897, 2091) ->
  9 Load(73) Time(2009, 2203) ->
  29 Load(89) Time(2447, 2641) ->
  0 Load(91) Time(3133, 86400)


Route 3:
  0 Load(0) Time(0, 0) ->
  24 Load(0) Time(574, 2146) ->
  30 Load(24) Time(906, 2478) ->
  0 Load(38) Time(1138, 86400)


Route 4:
  0 Load(0) Time(0, 0) ->
  26 Load(0) Time(230, 1430) ->
  13 Load(2) Time(576, 1776) ->
  7 Load(18) Time(764, 1964) ->
  1 Load(34) Time(982, 2182) ->
  12 Load(53) Time(1139, 2339) ->
  16 Load(74) Time(1312, 2512) ->
  0 Load(92) Time(1676, 86400)
'''

def distance(x1, y1, x2, y2):
    # Manhattan distance
    dist = abs(x1 - x2) + abs(y1 - y2)
    return dist

# Distance callback
class CreateDistanceCallback(object):
  """Create callback to calculate distances and travel times between points."""

  def __init__(self, locations):
    """Initialize distance array."""
    num_locations = len(locations)
    self.matrix = {}

    for from_node in xrange(num_locations):
      self.matrix[from_node] = {}
      for to_node in xrange(num_locations):
        x1 = locations[from_node][0]
        y1 = locations[from_node][1]
        x2 = locations[to_node][0]
        y2 = locations[to_node][1]
        self.matrix[from_node][to_node] = distance(x1, y1, x2, y2)

  def Distance(self, from_node, to_node):
    return self.matrix[from_node][to_node]


# Demand callback
class CreateDemandCallback(object):
  """Create callback to get demands at location node."""

  def __init__(self, demands):
    self.matrix = demands

  def Demand(self, from_node, to_node):
    return self.matrix[from_node]

# Service time (proportional to demand) callback.
class CreateServiceTimeCallback(object):
  """Create callback to get time windows at each location."""

  def __init__(self, demands, time_per_demand_unit):
    self.matrix = demands
    self.time_per_demand_unit = time_per_demand_unit

  def ServiceTime(self, from_node, to_node):
    return self.matrix[from_node] * self.time_per_demand_unit

# Create the travel time callback (equals distance divided by speed).
class CreateTravelTimeCallback(object):
  """Create callback to get travel times between locations."""

  def __init__(self, dist_callback, speed):
    self.dist_callback = dist_callback
    self.speed = speed

  def TravelTime(self, from_node, to_node):
    travel_time = self.dist_callback(from_node, to_node) / self.speed
    return travel_time

# Create total_time callback (equals service time plus travel time).
class CreateTotalTimeCallback(object):
  """Create callback to get total times between locations."""

  def __init__(self, service_time_callback, travel_time_callback):
    self.service_time_callback = service_time_callback
    self.travel_time_callback = travel_time_callback

  def TotalTime(self, from_node, to_node):
    service_time = self.service_time_callback(from_node, to_node)
    travel_time = self.travel_time_callback(from_node, to_node)
    return service_time + travel_time


def main():
  # Create the data.
  data = create_data_array()
  locations = data[0]
  demands = data[1]
  start_times = data[2]
  end_times = data[3]
  num_locations = len(locations)
  depot = 0
  num_vehicles = 5
  search_time_limit = 400000

  # Create routing model.
  if num_locations > 0:

    # The number of nodes of the VRP is num_locations.
    # Nodes are indexed from 0 to num_locations - 1.
    # By default the start of a route is node 0.
    routing = pywrapcp.RoutingModel(num_locations, num_vehicles, depot)
    search_parameters = pywrapcp.RoutingModel.DefaultSearchParameters()

    # Setting first solution heuristic: the
    # method for finding a first solution to the problem.
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

    # The 'PATH_CHEAPEST_ARC' method does the following:
    # Starting from a route "start" node, connect it to the node which produces the
    # cheapest route segment, then extend the route by iterating on the last
    # node added to the route.


    # Put callbacks to the distance function and travel time functions here.
    dist_between_locations = CreateDistanceCallback(locations)
    dist_callback = dist_between_locations.Distance
    routing.SetArcCostEvaluatorOfAllVehicles(dist_callback)

    demands_at_locations = CreateDemandCallback(demands)
    demands_callback = demands_at_locations.Demand

    # Adding capacity dimension constraints.
    VehicleCapacity = 100;
    NullCapacitySlack = 0;
    fix_start_cumul_to_zero = True
    capacity = "Capacity"
    routing.AddDimension(demands_callback, NullCapacitySlack, VehicleCapacity,
                         fix_start_cumul_to_zero, capacity)

    # Add time dimension.
    time_per_demand_unit = 3
    horizon = 24 * 60* 60
    time = "Time"
    speed = 1

    service_times = CreateServiceTimeCallback(demands, time_per_demand_unit)
    service_time_callback = service_times.ServiceTime

    travel_times = CreateTravelTimeCallback(dist_callback, speed)
    travel_time_callback = travel_times.TravelTime

    total_times = CreateTotalTimeCallback(service_time_callback, travel_time_callback)
    total_time_callback = total_times.TotalTime

    routing.AddDimension(total_time_callback,  # total time function callback
                         horizon,
                         horizon,
                         fix_start_cumul_to_zero,
                         time)

    # Add time window constraints.
    time_dimension = routing.GetDimensionOrDie(time)
    for location in range(1, num_locations):
      start = start_times[location]
      end = end_times[location]
      time_dimension.CumulVar(location).SetRange(start, end)


    # Solve displays a solution if any.
    assignment = routing.SolveWithParameters(search_parameters)
    if assignment:
      size = len(locations)
      # Solution cost.
      print "Total distance of all routes: " + str(assignment.ObjectiveValue()) + "\n"
      # Inspect solution.
      capacity_dimension = routing.GetDimensionOrDie(capacity);
      time_dimension = routing.GetDimensionOrDie(time);

      for vehicle_nbr in range(num_vehicles):
        index = routing.Start(vehicle_nbr)
        plan_output = 'Route {0}:'.format(vehicle_nbr)

        while not routing.IsEnd(index):
          node_index = routing.IndexToNode(index)
          load_var = capacity_dimension.CumulVar(index)
          time_var = time_dimension.CumulVar(index)
          plan_output += \
                    " {node_index} Load({load}) Time({tmin}, {tmax}) -> ".format(
                        node_index=node_index,
                        load=assignment.Value(load_var),
                        tmin=str(assignment.Min(time_var)),
                        tmax=str(assignment.Max(time_var)))
          index = assignment.Value(routing.NextVar(index))

        node_index = routing.IndexToNode(index)
        load_var = capacity_dimension.CumulVar(index)
        time_var = time_dimension.CumulVar(index)
        plan_output += \
                  " {node_index} Load({load}) Time({tmin}, {tmax})".format(
                      node_index=node_index,
                      load=assignment.Value(load_var),
                      tmin=str(assignment.Min(time_var)),
                      tmax=str(assignment.Max(time_var)))
        print plan_output
        print "\n"
    else:
      print 'No solution found.'
  else:
    print 'Specify an instance greater than 0.'


if __name__ == '__main__':
  main()