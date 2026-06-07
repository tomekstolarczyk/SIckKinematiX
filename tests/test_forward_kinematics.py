import pytest
import math
import sickkinematix as sk

# pytest zawsze sam to wywola i wstrzyknie wynik, jeśli dodamy 'ur5_robot' jako argument testu
@pytest.fixture
def ur5_robot():
    a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
    alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
    d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
    offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return sk.build_robot(a, alpha, d, offsets)


# 1 TEST FOR HAPPY PATH: FK dla pozycji zerowej
def test_fk_zero_position(ur5_robot):

    angles = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    results = sk.fk(ur5_robot, *angles)
    
    # Oczekujemy listy 7 elementów (baza + 6 przegubów)
    assert type(results) is list
    assert len(results) == 7
    
    baza = results[0]
    assert baza == (0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
    
    # Ostatni przegub (end-effector) też musi mieć 6 elementów [x, y, z, zx, zy, zz]
    end_effector = results[-1]
    assert len(end_effector) == 6


# 2 TEST FOR WRONG VALUES: zamiast obiektu PyCapsule dajemy zwyklego stringa
def test_fk_invalid_robot():

    fake_robot = "to_nie_jest_robot"

    with pytest.raises(TypeError, match="Expected input : RobotArm"):
        sk.fk(fake_robot, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


# 3 TEST FOR WRONG TYPES: pierwszy kat string zamiast doubla
def test_fk_invalid_angles(ur5_robot):
    with pytest.raises(TypeError):
        sk.fk(ur5_robot, "eooo", 0.0, 0.0, 0.0, 0.0, 0.0)