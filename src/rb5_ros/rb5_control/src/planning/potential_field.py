import numpy as np

def distance(cell0, cell1):
    return np.sqrt((cell0[0] - cell1[0])**2 + (cell0[1] - cell1[1])**2)

def get_neighbors(cell, mapp):
    """Get Neighbors function for Potential Field Method, no occupancy check"""
    i, j = cell
    potential_neighbors = [
        [i+1, j+1],
        [i-1, j-1],
        [i+1, j-1],
        [i-1, j+1],
        [i-1, j],
        [i+1, j],
        [i, j-1],
        [i, j+1]
    ]
    valids = []
    for ne in potential_neighbors:
        if ne[0] >= 0 and ne[0] < mapp.shape[0] and ne[1] >= 0 and ne[1] < mapp.shape[1]:
            valids.append(ne)
    return valids

def construct_potential_field(goal, world, obs_centers):
    GOAL_GAIN = 5.5
    OBSTACLE_GAIN = 1000
    OBSTACLE_INFLUENCE = 5 # 5 cells of influence
    # create attractive potential field
    U_att = np.zeros(world.shape)
    for i in range(world.shape[0]):
        for j in range(world.shape[1]):
            rho = distance(goal, [i, j])
            U_att[i][j] = 0.5 * GOAL_GAIN * (rho**2)

    # create repulsive potential fields
    U_reps = []
    for obs in obs_centers:
        U_rep = np.zeros(world.shape)
        for i in range(world.shape[0]):
            for j in range(world.shape[1]):
                d = distance(obs, [i, j])
                if d <= OBSTACLE_INFLUENCE:
                    U_rep[i][j] = 0.5 * OBSTACLE_GAIN + (1/d - 1/OBSTACLE_INFLUENCE)
                else:
                    U_rep[i][j] = 0
        U_reps.append(U_rep)

    U_rep = np.zeros(world.shape)
    for U_temp in U_reps:
        U_rep = U_rep + U_temp

    U = U_att + U_rep
    return U

def gradient_descent(start, goal, field):
    """
    path returned is in matrix i,j format not x,y, convert later in safest_route method
    """
    cur = start
    stuck = False
    path = [start]
    count = 0
    mod_field = field.copy()
    while not stuck and cur != goal:
        print(cur)
        nes = get_neighbors(cur, mod_field)
        if len(nes) == 0:
            print("No neighbors")
            return path
        if len(nes) == 1:
            ne = nes[0]
            if mod_field[ne[0]][ne[1]] < mod_field[cur[0]][cur[1]]:
                cur = ne
                path.append(cur)
            else:
                stuck = True
                print("Local minima")
        else:
            ne = nes[0]
            best_grad = mod_field[ne[0]][ne[1]] - mod_field[cur[0]][cur[1]]
            best_ne = ne
            for i in range(1, len(nes)):
                ne = nes[i]
                grad = mod_field[ne[0]][ne[1]] - mod_field[cur[0]][cur[1]]
                if grad < best_grad:
                    best_grad = grad
                    best_ne = ne
            if best_grad >= 0:
                stuck = True
                print("Local minima")
            else:
                cur = best_ne
                path.append(cur)
        if stuck and count == 0:
            stuck = False
            count += 1
            mod_field[cur[0]][cur[1]] += 200
            print("adding noise")
        else:
            count = 0
    return path, stuck

def safest_route(start, goal, world, obs_centers):
	U = construct_potential_field(goal, world, obs_centers)
	path, stuck = gradient_descent(start, goal, U)

	if stuck:
		raise Exception("Local Minima encountered, no path found")
	else:
		# convert matrix i,j notation to world x,y notation (ie. swap i and j)
		path = np.array(path)
		coordinates = np.hstack([path[:, 1].reshape(-1, 1), path[:, 0].reshape(-1, 1)])
		return coordinates # sequence of cells in world, not waypoints
