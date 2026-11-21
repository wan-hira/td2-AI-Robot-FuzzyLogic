"""
Fuzzy Logic Inference
TP2-AI-Webots e-puck Wall Follower.
Waneeza Hira AWAN - M1 Group 1
"""


def triangular(x, a, b, c):
    """
    Triangular function defined by three points (a, b, c).
    Returns degree in [0.0, 1.0].
    """
    if x <= a or x >= c:
        return 0.0
    elif a < x <= b:
        return (x - a) / (b - a)
    else:
        return (c - x) / (c - b)


def trapezoidal(x, a, b, c, d):
    """
    Trapezoidal function defined by four points (a, b, c, d).
    Returns degree in [0.0, 1.0].
    """
    if x <= a or x >= d:
        return 0.0
    elif b <= x <= c:
        return 1.0
    elif a < x < b:
        return (x - a) / (b - a)
    else:
        return (d - x) / (d - c)


def fuzzify_distance(sensor_val):
    """
    Convert e-puck proximity sensor reading (0 to ~4096) into linguistic degrees.
    Higher sensor reading = closer obstacle.
    - Far: value < 300 (wall is far or absent)
    - Medium: peak around 350 (ideal wall-following distance ~3-4 cm)
    - Close: value > 450 (too close to the wall)
    """
    far = trapezoidal(sensor_val, -1.0, 0.0, 120.0, 300.0)
    medium = triangular(sensor_val, 150.0, 350.0, 600.0)
    close = trapezoidal(sensor_val, 450.0, 700.0, 4096.0, 5000.0)

    return {"far": far, "medium": medium, "close": close}


def compute_fuzzy_control(left_front_diag, left_side, left_rear, right_front_diag, right_side, right_rear):
    """
    Compute normalized linear speed and steering angle delta_theta
    using a 6-rule fuzzy inference system for left-wall following.
    """
    #Fuzzification
    l_diag = fuzzify_distance(left_front_diag)
    l_side = fuzzify_distance(left_side)
    l_rear = fuzzify_distance(left_rear)
    r_diag = fuzzify_distance(right_front_diag)
    r_side = fuzzify_distance(right_side)
    r_rear = fuzzify_distance(right_rear)

    # Rule Base & Inference (using min for AND operator)
    # Positive delta_theta = turn LEFT | Negative delta_theta = turn RIGHT
    rules_angle = []
    rules_speed = []

    # Rule 1: Inward corner / Front obstacle -> Turn RIGHT sharply, slow down
    w_inward_corner = max(l_diag["close"], r_diag["close"], l_diag["medium"])
    rules_angle.append((w_inward_corner, -1.0))  # Turn right
    rules_speed.append((w_inward_corner, 0.15))   # Slow speed

    # Rule 2: Too close to left wall -> Gentle RIGHT correction
    w_too_close = min(l_side["close"], l_diag["far"])
    rules_angle.append((w_too_close, -0.35))
    rules_speed.append((w_too_close, 0.45))

    # Rule 3: Ideal parallel distance -> Go STRAIGHT at fast cruise
    w_ideal = min(l_side["medium"], l_diag["far"], r_diag["far"])
    rules_angle.append((w_ideal, 0.0))
    rules_speed.append((w_ideal, 0.65))

    # Rule 4: Outward corner detected via rear sensor (ps4) -> Pivot LEFT tightly
    wall_still_at_rear = max(l_rear["medium"], l_rear["close"])
    w_outward_corner = min(l_side["far"], l_diag["far"], r_diag["far"], wall_still_at_rear)
    rules_angle.append((w_outward_corner, 0.75))
    rules_speed.append((w_outward_corner, 0.30))

    # Rule 5: Open space (all left sensors far) -> Gentle LEFT arc to search for wall
    w_open_space = min(l_side["far"], l_diag["far"], l_rear["far"], r_diag["far"])
    rules_angle.append((w_open_space, 0.35))
    rules_speed.append((w_open_space, 0.50))

    # Rule 6 (Right-side safety clearance):
    # Prevent rear/side right scraping when pivoting in narrow corridors
    w_right_safety = max(r_side["close"], r_rear["close"])
    rules_angle.append((w_right_safety, 0.40))
    rules_speed.append((w_right_safety, 0.35))

    #Defuzzification 
    total_weight_angle = sum(w for w, _ in rules_angle)
    delta_theta = (
        sum(w * val for w, val in rules_angle) / total_weight_angle
        if total_weight_angle > 0
        else 0.0
    )

    total_weight_speed = sum(w for w, _ in rules_speed)
    speed_ratio = (
        sum(w * val for w, val in rules_speed) / total_weight_speed
        if total_weight_speed > 0
        else 0.4
    )

    return speed_ratio, delta_theta