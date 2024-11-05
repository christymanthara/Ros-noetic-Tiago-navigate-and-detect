The project's objective is to guide the robot Tiago from its starting position, "Starting Pose," to the target location, "Pose B," by navigating through an environment that includes two rooms, a corridor, and various obstacles. Upon reaching Pose B, Tiago must detect cylindrical obstacles in its surroundings, identify how many there are, and determine their locations within the room. Tiago can reach Pose B through one of three modes:

1. **Mode 1:** Using the custom motion control law we developed, Tiago moves directly to Pose B without relying on the Navigation stack.
2. **Mode 2:** Tiago uses the custom motion control law to cross the corridor, then switches to the Navigation stack to reach Pose B.
3. **Mode 3:** Tiago uses the existing control law entirely, fully utilizing the Navigation stack to reach Pose B.
