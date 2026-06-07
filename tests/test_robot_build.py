import pytest
import math
import sickkinematix as sk

# 1 TEST FOR HAPPY PATH: Poprawne budowanie robota 
def test_build_ur5_robot_success():

    # Prawdziwe parametry UR5 (MDH)
    a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
    alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
    d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
    offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    robot = sk.build_robot(a, alpha, d, offsets)
    
    # sprawdzamy, czy funkcja C poprawnie zwrocila nam obiekt PyCapsule
    assert type(robot).__name__ == 'PyCapsule'
    assert robot is not None

# 2 TEST FOR WRONG VALUES: Bramkarz zle dlugosci - dajemy po 5 elementow zamiast 6 
def test_build_robot_invalid_length():

    a = [0.0] * 5
    alpha = [0.0] * 6
    d = [0.0] * 6
    offsets = [0.0] * 6

    # oczekujemy raise exception value error
    with pytest.raises(ValueError, match="length 6"):
        sk.build_robot(a, alpha, d, offsets)

# 3 TEST FOR WRONG TYPES : Bramkarz - zle typy danych (stringi zamiast doubli)
def test_build_robot_invalid_type():

    a = [0.0, "pijWojankaDoBialegoRanka", -0.425, -0.39225, 0.0, 0.0]
    alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
    d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
    offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # oczekujemy raise exception type error
    with pytest.raises(TypeError):
        sk.build_robot(a, alpha, d, offsets)