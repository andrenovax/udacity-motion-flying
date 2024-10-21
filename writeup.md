
Writeup:

It's quite confusing to make a writeup for this project, since I just follow instructions and dont do anything special.

1. Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning.

  - self.local_position_callback:
    motion planning sets waypoints by planning path, they are not hardcoded.

  - self.state_callback:
    motion planning transitions to planning state after arming, before takeoff

  - self.plan_path:
    creates waypoints based on colliders.csv data, TARGET_ALTITUDE, SAFETY_DISTANCE

  - self.arming_transition:
    motion planning doesnt set home position

  - self.target_position:
    motion planning has heading


2. In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position()):

  - Read in obstacle map and take lat and lon elements
    ```
    def get_initial_lat_lon(self):
        with open('colliders.csv', 'r') as file:
            first_line = file.readline()

        # Split the first line by commas and spaces to extract lat0 and lon0
        lat_str, lon_str = first_line.split(',')

        # Further split each string by space and convert to floats
        lat0 = float(lat_str.split()[1])
        lon0 = float(lon_str.split()[1])
        return lat0, lon0
    ```

  - update plan_path:
    ```
    lat0, lon0 = self.get_initial_lat_lon()
    self.set_home_position(lat0, lon0, 0)
    ```

3. In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from self._latitude, self._longitude and self._altitude. Then use the utility function global_to_local() to convert to local position (using self.global_home as well, which you just set).

  ```
  position = self.global_position
  current_north, current_east, _ = global_to_local(position, self.global_home)
  ```

4. In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

  ```
  grid_start = (int(current_north) - north_offset, int(current_east) - east_offset)
  ```

5. In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

  - manually flied to the point and used that coordinate as a goal:
    ```
    goal_global_position = [-122.399380, 37.794715, 0]
    ```

  - convert to local position on the grid:
    ```
    goal_end_north, goal_end_east, _ = global_to_local(goal_global_position, self.global_home)
    grid_goal = ((int(goal_end_north) - north_offset), (int(goal_end_east) - east_offset))
    ```

6. Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you're encouraged to get creative and try other methods from the lessons and beyond!

  - updated action
    ```
    DIAGONAL_COST = np.sqrt(2)

    class Action(Enum):
      # ...
      SOUTH_WEST = (1, -1, DIAGONAL_COST)
      SOUTH_EAST = (1, 1, DIAGONAL_COST)
      NORTH_WEST = (-1, -1, DIAGONAL_COST)
      NORTH_EAST = (-1, 1, DIAGONAL_COST)
    ```

  - updated valid actions
    ```
    def valid_actions(grid, current_node):
        # ...
        if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
            valid_actions.remove(Action.NORTH_WEST)
        if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
            valid_actions.remove(Action.SOUTH_WEST)
        if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
            valid_actions.remove(Action.NORTH_EAST)
        if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
            valid_actions.remove(Action.SOUTH_EAST)
    ```

7. Cull waypoints from the path you determine using search.
  - add helper
    ```
    def filter_collinear_waypoints(waypoints, epsilon=1e-2):
      if len(waypoints) < 3:
          return waypoints

      non_collinear_points = [waypoints[0]]

      # Loop from 1st to n-1th point
      for i in range(1, len(waypoints) - 1):
          p1 = non_collinear_points[-1]
          p2 = waypoints[i]
          p3 = waypoints[i + 1]

          matrix = np.array([
              [p1[0], p1[1], 1],
              [p2[0], p2[1], 1],
              [p3[0], p3[1], 1]
          ])

          det = np.linalg.det(matrix)

          if abs(det) > epsilon:
              non_collinear_points.append(p2)

      non_collinear_points.append(waypoints[-1])
      
      return non_collinear_points
      ```

  - update plan_path
    ```
    # TODO: prune path to minimize number of waypoints
    # TODO (if you're feeling ambitious): Try a different approach altogether!
    path = filter_collinear_waypoints(path)
    ```