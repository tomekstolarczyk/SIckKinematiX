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


####################### SEKCJA IK CCD ####################### 

# 1 TEST FOR HAPPY PATH: IK CCD (cel w zasiegu)
def test_ik_ccd_reachable(ur5_robot):
    tx, ty, tz = -0.5, -0.5, 0.2
    
    # przeliczamy
    katy = sk.ik_ccd(ur5_robot, tx, ty, tz)
    fk_res = sk.fk(ur5_robot, *katy)
    end_eff = fk_res[-1]

    # blad euklidesowy
    blad = math.sqrt((tx - end_eff[0])**2 + (ty - end_eff[1])**2 + (tz - end_eff[2])**2)
    assert blad < 0.01 


# 2 TEST FOR WRONG VALUES: ujemna liczba iteracji
def test_ik_ccd_invalid_params(ur5_robot):

    with pytest.raises(ValueError):
        sk.ik_ccd(ur5_robot, 0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -100, 0.001)


# 3 TEST FOR WRONG TYPES: string zamiast doubla dla celu X
def test_ik_ccd_invalid_type(ur5_robot):

    with pytest.raises(TypeError):
        sk.ik_ccd(ur5_robot, "eooo", 0.1, 0.1)

# 4 TEST FOR EDGE CASE: Cel poza zasięgiem, powinen nie trafic
def test_ik_ccd_unreachable(ur5_robot):

    tx, ty, tz = 100.0, 100.0, 100.0 # cel 100 metrów od robota 
    katy = sk.ik_ccd(ur5_robot, tx, ty, tz)
    
    assert len(katy) == 6
    
    fk_res = sk.fk(ur5_robot, *katy)
    end_eff = fk_res[-1]
    
    blad = math.sqrt((tx - end_eff[0])**2 + (ty - end_eff[1])**2 + (tz - end_eff[2])**2)
    
    # teraz chcemy zeby blad byl duzy
    assert blad > 50.0


#######################  SEKCJA IK DLS ####################### 

# 5 TEST FOR HAPPY PATH: IK DLS (cel w zasiegu)
def test_ik_dls_success(ur5_robot):
    
    tx, ty, tz = -0.5, -0.5, 0.2
    target_pose = [
        1.0, 0.0, 0.0, tx,
        0.0, 1.0, 0.0, ty,
        0.0, 0.0, 1.0, tz,
        0.0, 0.0, 0.0, 1.0
    ]
    
    katy = sk.ik_dls(ur5_robot, target_pose)
    
    # sprawdzamy czy zwrocil 6 katow
    assert len(katy) == 6
    assert isinstance(katy[0], float)

    # przeliczamy trafienie (dystans euklidesowy)
    fk_res = sk.fk(ur5_robot, *katy)
    end_eff = fk_res[-1]
    
    blad = math.sqrt((tx - end_eff[0])**2 + (ty - end_eff[1])**2 + (tz - end_eff[2])**2)
    assert blad < 0.01


# 6 TEST FOR WRONG VALUES: zle wymiary macierzy
def test_ik_dls_invalid_list(ur5_robot):

    target_pose_15 = [0.0] * 15  # 15 zamiast 16

    with pytest.raises(ValueError, match="16 elements"):
        sk.ik_dls(ur5_robot, target_pose_15)


# 7 TEST FOR WRONG VALUES: ujemna lambda
def test_ik_dls_invalid_lambda(ur5_robot):

    target_pose = [0.0] * 16

    with pytest.raises(ValueError, match="lambda"):
        sk.ik_dls(ur5_robot, target_pose, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100, 0.001, 0.1, -1.0)


# 8 TEST FOR WRONG TYPES: macierz pelna stringow
def test_ik_dls_invalid_type(ur5_robot):

    target_pose_strings = ["blablabla"] * 16

    with pytest.raises(TypeError):
        sk.ik_dls(ur5_robot, target_pose_strings)

# 9 TEST FOR EDGE CASE: DLS cel poza zasiegiem
def test_ik_dls_unreachable(ur5_robot):
    tx, ty, tz = 100.0, 100.0, 100.0  
    
    target_pose = [
        1.0, 0.0, 0.0, tx,
        0.0, 1.0, 0.0, ty,
        0.0, 0.0, 1.0, tz,
        0.0, 0.0, 0.0, 1.0
    ]
    
    katy = sk.ik_dls(ur5_robot, target_pose)
    
    assert len(katy) == 6
    
    fk_res = sk.fk(ur5_robot, *katy)
    end_eff = fk_res[-1]
    
    blad = math.sqrt((tx - end_eff[0])**2 + (ty - end_eff[1])**2 + (tz - end_eff[2])**2)
    
    # tu chcemy zeby błąd byl ogromny
    assert blad > 50.0