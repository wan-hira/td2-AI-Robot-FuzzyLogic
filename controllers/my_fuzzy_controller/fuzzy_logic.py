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
    Convert e-puck proximity sensor reading (0 to ~4000) into linguistic degrees.
    Higher sensor reading = closer obstacle.
    - Far: value < 250 (wall is far or absent)
    - Medium: peak around 350 (ideal wall-following distance ~3-4 cm)
    - Close: value > 550 (too close to the wall)
    """
    far = trapezoidal(sensor_val, -1.0, 0.0, 120.0, 300.0)
    medium = triangular(sensor_val, 150.0, 350.0, 600.0)
    close = trapezoidal(sensor_val, 450.0, 700.0, 4096.0, 5000.0)

    return {"far": far, "medium": medium, "close": close}


def compute_fuzzy_control(left_front_diag, left_side, right_front_diag, right_side):
    """
    Fuzzy Controller: Outputs desired linear speed (v) and steering angle (delta_theta).
    Assumes left-wall following behavior with right-side collision safety.
    """
    #Fuzzification
    l_diag = fuzzify_distance(left_front_diag)
    l_side = fuzzify_distance(left_side)
    r_diag = fuzzify_distance(right_front_diag)

    # Rule Base & Inference (using min for AND operator)
    # Positive delta_theta = turn LEFT | Negative delta_theta = turn RIGHT
    rules_angle = []
    rules_speed = []

    # Rule 1: If front-left or front-right sees an approaching wall (inward corner) -> Turn RIGHT sharply, slow down
    w_inward_corner = max(l_diag["close"], r_diag["close"], l_diag["medium"])
    rules_angle.append((w_inward_corner, -0.85))  # Turn right
    rules_speed.append((w_inward_corner, 0.25))   # Slow speed

    # Rule 2: If left side is TOO CLOSE to the wall -> Turn RIGHT gently, medium speed
    w_too_close = min(l_side["close"], l_diag["far"])
    rules_angle.append((w_too_close, -0.35))
    rules_speed.append((w_too_close, 0.45))

    # Rule 3: If left side is at IDEAL DISTANCE (medium) and front is clear -> Go STRAIGHT, fast cruise
    w_ideal = min(l_side["medium"], l_diag["far"], r_diag["far"])
    rules_angle.append((w_ideal, 0.0))
    rules_speed.append((w_ideal, 0.65))

    # Rule 4: If left side is TOO FAR from the wall -> Turn LEFT to find/rejoin the wall
    w_too_far = min(l_side["far"], l_diag["far"], r_diag["far"])
    rules_angle.append((w_too_far, 0.40))
    rules_speed.append((w_too_far, 0.45))

    #Defuzzification 
    total_weight_angle = sum(w for w, _ in rules_angle)
    if total_weight_angle > 0:
        delta_theta = sum(w * val for w, val in rules_angle) / total_weight_angle
    else:
        delta_theta = 0.0

    total_weight_speed = sum(w for w, _ in rules_speed)
    if total_weight_speed > 0:
        speed_ratio = sum(w * val for w, val in rules_speed) / total_weight_speed
    else:
        speed_ratio = 0.4

    return speed_ratio, delta_theta