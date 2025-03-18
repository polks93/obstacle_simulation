import numpy as np
from typing import Union, Tuple, Set, Sequence

from obstacle_simulation import ShipObstacle

def lidar(pose, Ship, lidar_params):
    # type: (np.ndarray, ShipObstacle, dict) -> Tuple[np.ndarray, np.ndarray, Set[int]]

    """
    Simula un sensore LiDAR per rilevare un ostacolo definito da un oggetto ShipObstacle.
    Args:
        pose (np.ndarray): La posa del sensore LiDAR, rappresentata come un array numpy [x, y, theta].
        Ship (ShipObstacle): L'ostacolo da rilevare.
        lidar_params (dict): Parametri del LiDAR, inclusi 'max_range' (portata massima), 
                             'n_beams' (numero di raggi) e 'FoV' (campo visivo).
    Returns:
        Tuple[np.ndarray, np.ndarray, set[int]]: Una tupla contenente:
            - ranges (np.ndarray): Distanze rilevate per ciascun raggio.
            - angles (np.ndarray): Angoli dei raggi rispetto alla posa del sensore.
            - seen_segments (set[int]): Insieme degli ID dei segmenti visti.
    """

    # Import dati lidar
    max_range = lidar_params['max_range']
    n_beams = lidar_params['n_beams']
    FoV = lidar_params['FoV']

    # Init dei ranges al valore massimo
    ranges = np.full(n_beams, max_range, dtype=np.float64)
    angles = np.linspace(- FoV / 2, FoV / 2, n_beams) + pose[2]
    seen_segments = set() # type: Set[int]

    # Se la circonferenza non e in max_range, ritorna i ranges iniziali
    ship_center = np.array([Ship.center[0], Ship.center[1]])
    if np.linalg.norm(pose[:2] - ship_center) > (max_range + Ship.radius):
        return ranges, angles, seen_segments

    # Se la bounding box non e in max_range, ritorna i ranges iniziali
    ship_rect = Ship.get_bounding_box()
    if not is_rect_in_max_range(ship_rect, pose, max_range):
        return ranges, angles, seen_segments
    

    # Angoli assoluti dei raggi
    angles = np.linspace(- FoV / 2, FoV / 2, n_beams) + pose[2]
    for i, angle in enumerate(angles):
        min_distance = max_range
        distance, seen_segment_id = ray_polygon_intersection(pose, angle, Ship)
        if distance is not None and distance < min_distance:
            assert seen_segment_id is not None
            seen_segments.add(seen_segment_id)
            min_distance = distance
        ranges[i] = min_distance
    
    return ranges, angles, seen_segments

def proximity_sensor(pose, heading, max_range, Ship):
    # type: (np.ndarray, float, float, ShipObstacle) -> float
    """
    Calcola la distanza tra una nave e un ostacolo utilizzando un sensore di prossimita`.
    Args:
        pose (np.ndarray): La posizione attuale dell'agente [x, y, theta].
        heading (float): Heading del sensore in radianti.
        max_range (float): La distanza massima che il sensore puo` rilevare.
        Ship (ShipObstacle): Ostacolo da rilevare.
    Returns:
        float: La distanza rilevata tra la nave e l'ostacolo. Se non ci sono ostacoli entro il raggio massimo, restituisce max_range.
    """
    angle = heading + pose[2]
    min_distance = max_range
    distance, _ = ray_polygon_intersection(pose, angle, Ship)
    if distance is not None and distance < min_distance:
        min_distance = distance

    return min_distance

def ray_polygon_intersection(pose, angle, Ship):
    # type: (np.ndarray, float, ShipObstacle) -> Tuple[Union[float, None], Union[int, None]]
    """
    Calcola l'intersezione tra un raggio e un poligono rappresentato da un oggetto ShipObstacle.
    Args:
        pose (np.ndarray): La posizione iniziale del raggio come array numpy [x, y].
        angle (float): L'angolo del raggio in radianti.
        Ship (ShipObstacle): L'oggetto ShipObstacle che contiene i segmenti del poligono.
    Returns:
        Tuple[Union[float, None], Union[int, None]]: Una tupla contenente la distanza minima dall'origine del raggio al punto 
        di intersezione piu` vicino (o None se non ci sono intersezioni) e l'ID del segmento visto (o None se non ci sono intersezioni).
    """
    x0, y0 = pose[:2]
    min_distance = None
    dx = np.cos(angle)
    dy = np.sin(angle)

    seen_segment_id = None

    for segment in Ship.segments_dict.values():
        edge = (segment.start_point, segment.end_point)
        point = ray_segment_intersection(x0, y0, dx, dy, edge)
        if point is not None:
            distance = np.hypot(point[0] - x0, point[1] - y0)
            if min_distance is None or distance < min_distance:
                min_distance = distance
                seen_segment_id = segment.id

    return min_distance, seen_segment_id

def ray_segment_intersection(x0, y0, dx, dy, segment):
    # type: (float, float, float, float, Tuple) -> Union[Tuple, None]
    """
    Calcola il punto di intersezione tra un raggio e un segmento di linea.
    Parametri:
    - x0 (float): Coordinata x del punto di partenza del raggio.
    - y0 (float): Coordinata y del punto di partenza del raggio.
    - dx (float): Componente x del vettore direzione del raggio.
    - dy (float): Componente y del vettore direzione del raggio.
    - segment (tuple): Tupla contenente le coordinate degli estremi del segmento di linea nel formato ((x1, y1), (x2, y2)).
    Ritorna:
    - tuple o None: Se esiste un punto di intersezione, restituisce una tupla (ix, iy) contenente le coordinate x e y del punto di intersezione. Se non esiste un punto di intersezione, restituisce None.
    """
    # Estrae le coordinate del segmento
    (x1, y1), (x2, y2) = segment
    x3, y3 = x0, y0
    x4, y4 = x0 + dx, y0 + dy
    
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)

    if abs(den) <= 1e-9:
        return None # Le linee sono parallele o sovrapposte
    
    # Calcola i parametri di intersezione
    t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / den
    u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / den

    if 0 <= t <= 1 and u >= 0:
        ix = x1 + t * (x2 - x1)
        iy = y1 + t * (y2 - y1)
        return (ix, iy)
    else:
        return None

def is_rect_in_max_range(rectangle, pose, max_range):
    # type: (Sequence, np.ndarray, float) -> bool
    """
    Verifica se un rettangolo e` all'interno di un raggio massimo da una data posizione.
    Args:
        rectangle (Tuple): Coordinate del rettangolo nel formato (xmin, ymin, xmax, ymax).
        pose (Union[Sequence, np.ndarray]): Posizione di riferimento nel formato (cx, cy).
        max_range (float): Raggio massimo entro il quale verificare la presenza del rettangolo.
    Returns:
        bool: True se almeno un segmento del rettangolo e` all'interno del raggio massimo, False altrimenti.
    """
    # Coordinate del centro del raggio
    C = pose[:2]
    # Coordinate del rettangolo
    xmin, ymin, xmax, ymax = rectangle
    # Coordinate di tutti i vertici
    vertices = np.array([[xmin, ymin], [xmax, ymin], [xmax, ymax], [xmin, ymax]])
    # Segmenti del rettangolo
    segments = [(vertices[i], vertices[(i + 1) % 4]) for i in range(4)]
    # Verifica se almeno un segmento e` all'interno del raggio massimo
    for s in segments:
        if point_to_segment_distance(C, s) <= max_range:
            return True
    return False

def point_to_segment_distance(P, s):
    # type: (np.ndarray, Sequence[np.ndarray]) -> np.float64
    """
    Calcola la distanza minima tra un punto e un segmento.
    Parametri:
    P (np.ndarray): Coordinata del punto.
    s (np.ndarray): Segmento definito da due punti.
    Ritorna:
    np.float64: La distanza minima tra il punto e il segmento.
    """
    # Estrai i punti del segmento
    v = s[0]
    w = s[1]

    # Calcola la proiezione del punto P sulla linea del segmento
    l2 = np.sum((w - v) ** 2)  # Lunghezza al quadrato del segmento
    if l2 == 0.0:
        return np.linalg.norm(P - v)  # s[0] e s[1] sono lo stesso punto
    t = max(0, min(1, np.dot(P - v, w - v) / l2))
    projection = v + t * (w - v)
    return np.linalg.norm(P - projection)

if __name__ == "__main__":
    import matplotlib.pyplot as plt

    Ship = ShipObstacle((3,0))
    Cx, Cy = Ship.center
    radius = Ship.radius
    points = Ship.points
    x_ship, y_ship = zip(*points)

    theta = np.linspace(0, 2*np.pi, 100)
    x_circle = Cx + radius * np.cos(theta)
    y_circle = Cy + radius * np.sin(theta)

    x_min = Ship.x_min
    x_max = Ship.x_max
    y_min = Ship.y_min
    y_max = Ship.y_max
    square_points = [(x_min, y_min), (x_max, y_min), (x_max, y_max), (x_min, y_max), (x_min, y_min)]
    x_square, y_square = zip(*square_points)

    pose = np.array([5, 0, np.pi])
    lidar_params = {'max_range': 2, 'n_beams': 90, 'FoV': np.pi/2}
    FoV = lidar_params['FoV']
    theta_lidar = np.linspace(- FoV / 2, FoV / 2, 100) + pose[2]

    x_range = pose[0] + lidar_params['max_range'] * np.cos(theta_lidar)
    y_range = pose[1] + lidar_params['max_range'] * np.sin(theta_lidar)

    ranges, angles, seen_segments_id = lidar(pose, Ship, lidar_params)
    x_lidar = pose[0] + ranges * np.cos(angles)
    y_lidar = pose[1] + ranges * np.sin(angles)
    print(ranges)
    print(seen_segments_id)

    proximity_sensor_heading = np.pi/2
    proximity_sensor_max_range = 2
    distance = proximity_sensor(pose, proximity_sensor_heading, proximity_sensor_max_range, Ship)
    x_proximity = pose[0] + distance * np.cos(proximity_sensor_heading + pose[2])
    y_proximity = pose[1] + distance * np.sin(proximity_sensor_heading+ pose[2])
    print(distance)

    segment_points = []
    if len(seen_segments_id) > 0:
        for id in seen_segments_id:
            segment = Ship.segments_dict[id]
            x_s, y_s = segment.mid_point
            segment_points.append((x_s, y_s))
        x_segment, y_segment = zip(*segment_points)

    plt.figure(figsize=(8, 4))
    plt.scatter(pose[0], pose[1], color='b', label='Posizione Lidar')
    plt.plot(x_range, y_range, '--')
    plt.scatter(x_lidar, y_lidar, color='g', label='Misurazioni Lidar')
    plt.plot(x_ship, y_ship, label='Nave')
    if len(seen_segments_id) > 0:
        plt.plot(x_segment, y_segment, '+r', label='Segmento visto')
    # plt.scatter(Cx, Cy, color='r', label='Centro della Nave', s=100)
    # plt.plot(x_circle, y_circle, '--', label='Circonferenza di Bounding Box')
    # plt.plot(x_square, y_square, '--', label='Bounding Box')
    plt.scatter(x_proximity, y_proximity, color='r', label='Misurazione Prossimita`')
    plt.axis('equal')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.grid(True)
    plt.show()
