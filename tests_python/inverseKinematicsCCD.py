import math
import plotly.graph_objects as go
import sickkinematix  


# 1. test dla jednego konkretnego punktu

current_wygiecia = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0) 
target_x = 0.1349
target_y = -0.0383
target_z = 1.1811
katy_po_ik = sickkinematix.ik_ccd(target_x, target_y, target_z, *current_wygiecia)
fkResults = sickkinematix.fk(*katy_po_ik) 
pozycja_x = fkResults[6][0]
pozycja_y = fkResults[6][1]
pozycja_z = fkResults[6][2]

ex = target_x - pozycja_x
ey = target_y - pozycja_y
ez = target_z - pozycja_z
blad = math.sqrt(ex**2 + ey**2 + ez**2)
    
sukces = "SUKCES" if (blad < 0.05) else ":("
print(f"jeden konkretny punkt: {sukces}")

