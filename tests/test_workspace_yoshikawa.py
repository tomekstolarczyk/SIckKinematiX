import pytest
import math
import sickkinematix as sk

@pytest.fixture
def ur5_robot():
    a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
    alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
    d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
    offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    return sk.build_robot(a, alpha, d, offsets)


####################### SEKCJA WORKSPACE ####################### 

# 1 TEST FOR HAPPY PATH: Generowanie chmury punktów
def test_workspace_generation(ur5_robot):
    num_points = 1000
    cloud = sk.workspace(ur5_robot, num_points)
    
    # powinnismy dostac krotke z 4 elementami - po jednym wskanziku na kazda tablice (num_points) punktow : X, Y, Z, Yoshikawa
    assert isinstance(cloud, tuple) 
    assert len(cloud) == 4 
    assert len(cloud[0]) == 1000 

# 2 TEST FOR WRONG VALUES: Ujemna liczba punktów
def test_workspace_invalid_points(ur5_robot):
    with pytest.raises(ValueError):
        sk.workspace(ur5_robot, -500)

####################### SEKCJA YOSHIKAWA ####################### 

# 3 TEST FOR HAPPY PATH: Zwinna pozycja robota
def test_yoshikawa_good_pose(ur5_robot):
    # mocno powyginane katy - robot powinien miec duzo luzu
    angles = (0.5, -1.0, 1.0, 0.5, 0.5, 0.0)
    measure = sk.yoshikawa(ur5_robot, *angles)
    
    assert isinstance(measure, float)
    # manipulability index powinien być w miare wysoki
    assert measure > 0.01 


# 4 TEST FOR EDGE CASE: singulatiries
def test_yoshikawa_singularity(ur5_robot):
    # Ramie maksymalnie wyprostowane 
    angles = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    measure = sk.yoshikawa(ur5_robot, *angles)
    
    # powinno stracic stopen swobody, index powinien zleciec blisko zera
    assert measure < 0.0001