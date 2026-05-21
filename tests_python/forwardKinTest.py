import sickkinematix

katy = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)
wynik = sickkinematix.fk(*katy)

print(f"Otrzymana pozycja końcówki (X, Y, Z):")
print(f"X: {wynik[0]:.4f}")
print(f"Y: {wynik[1]:.4f}")
print(f"Z: {wynik[2]:.4f}")

