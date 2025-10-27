# apf_search.py — Núcleo del algoritmo (extraído)
import numpy as np

def distance(a, b):
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return float(np.linalg.norm(a - b))

def unit_vector(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v / n if n > 1e-12 else np.zeros_like(v, dtype=float)

def apf_search(start_xy, goal_xy, obstacles, *,
               max_iters=20000,
               influence_area=80,           # d0
               attraction_factor=1.0,       # k_att
               repulsion_factor=20.0,       # k_rep
               close_enough=1.0):
    """
    start_xy: (x, y) inicial
    goal_xy:  (x, y) objetivo
    obstacles: lista de (x, y, r)   # r no entra en tu fórmula original, se ignora aquí
    Devuelve: np.ndarray [N x 2] con la trayectoria
    """
    q = np.array(start_xy, dtype=float)
    goal = np.array(goal_xy, dtype=float)
    d0 = float(influence_area)

    path = [q.copy()]
    iters = 0
    d_goal = np.inf

    while d_goal > close_enough and iters < max_iters:
        iters += 1

        # --- Fuerza atractiva (tu _compute_attractive_foce) ---
        d_goal = distance(q, goal)
        f_att = attraction_factor * unit_vector(goal - q)

        # --- Fuerza repulsiva (tu _compute_repulsive_force) ---
        f_rep = np.zeros(2, dtype=float)
        min_dist_obs = np.inf
        for (ox, oy, _r) in obstacles:
            o = np.array([ox, oy], dtype=float)
            d_obs = distance(q, o)
            min_dist_obs = min(min_dist_obs, d_obs)

            if d_obs > d0:
                f_rep_obs = np.zeros(2, dtype=float)
            else:
                # Misma forma que tu código: (1/d - 1/d0) * dirección desde obstáculo
                f_obs_dir = unit_vector(q - o)
                f_rep_obs = repulsion_factor * (1.0/d_obs - 1.0/d0) * f_obs_dir
            f_rep += f_rep_obs

        # --- Resultante y “movimiento” (equivalente a robot.move(f_total, speed)) ---
        f_total = f_att + f_rep
        speed = np.exp(d_goal / 100.0)               # igual que en tu run()

        # Avance en la dirección de la fuerza total, escalado por speed
        n = np.linalg.norm(f_total)
        if n > 1e-12:
            q = q + (f_total / n) * speed

        path.append(q.copy())

    return np.asarray(path, dtype=float)

# Ejemplo rápido (similar a tu __main__)
if __name__ == "__main__":
    start = (0.0, 0.0)
    goal  = (100.0, 100.0)
    obstacles = [(20, 30, 10), (10, 80, 25), (70, 60, 15), (80, 90, 20), (80, 20, 20)]
    traj = apf_search(start, goal, obstacles,
                      max_iters=20000,
                      influence_area=80,
                      attraction_factor=1.0,
                      repulsion_factor=20.0,
                      close_enough=1.0)
    # 'traj' es tu path (N x 2). Píntalo como prefieras.