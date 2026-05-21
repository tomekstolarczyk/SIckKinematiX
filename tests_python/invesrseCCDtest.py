import sickkinematix

# tu chcemy
cel_x, cel_y, cel_z = 0.1349, -0.0383, 1.1811
print(f"CEL :  X: {cel_x:.4f}, Y: {cel_y:.4f}, Z: {cel_z:.4f}")

# przeliczamy
katy = sickkinematix.ik_ccd(cel_x, cel_y, cel_z) 
fkResults = sickkinematix.fk(*katy) 

print(f"POZYCJA KONCOWA : X: {fkResults[0]:.4f}, Y: {fkResults[1]:.4f}, Z: {fkResults[2]:.4}")

bladx = cel_x - fkResults[0]
blady = cel_y - fkResults[1]
bladz = cel_z - fkResults[2]
bladCalk = (bladx**2 + blady**2 + bladz**2)**0.5

print(f"BLAD (odleglosc): {bladCalk:.6f}")
if bladCalk < 0.001:
    print("LETSGOOO!")
else:
    print(":(")