import math
import plotly.graph_objects as go
import sickkinematix  

# 1. generujemy trajektorie (kolo) ----------------------------------------------------
trajektoria = []
R = 0.1
X_Srodka = 0.2 
Y_Srodka = 0.0
Z_Srodka = 0.1  
liczba_punktow_na_kole = 80

for i in range(liczba_punktow_na_kole):
    kat = i * (360 / liczba_punktow_na_kole)
    kat = math.radians(kat)

    x = X_Srodka + R * math.cos(kat)
    y = Y_Srodka + R * math.sin(kat)
    trajektoria.append((x, y, Z_Srodka))

# 2. liczymy w c -----------------------------------------------------------

RobotAnimFrames = []
aktualne_katy = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

for punkt in trajektoria:
    aktualne_katy = sickkinematix.ik_ccd(punkt[0], punkt[1], punkt[2], *aktualne_katy) 
    fkResults = sickkinematix.fk(*aktualne_katy) 
    RobotAnimFrames.append(fkResults)


# 3. RYSOWANIE I ANIMACJA ------------------------------------------------------------

fig = go.Figure()

# Trajektoria
traj_x = [p[0] for p in trajektoria]
traj_y = [p[1] for p in trajektoria]
traj_z = [p[2] for p in trajektoria]

fig.add_trace(go.Scatter3d(
    x=traj_x, y=traj_y, z=traj_z,
    mode='lines',
    name='Trasa Celu',
    line=dict(color='#e74c3c', width=4, dash='dash')
))

# Robot (Startowa klatka)
start_x = [p[0] for p in RobotAnimFrames[0]]
start_y = [p[1] for p in RobotAnimFrames[0]]
start_z = [p[2] for p in RobotAnimFrames[0]]

fig.add_trace(go.Scatter3d(
    x=start_x, y=start_y, z=start_z,
    mode='lines+markers',
    name='Ramię SICK',
    marker=dict(size=8, color='#2c3e50'),  
    line=dict(color='#2980b9', width=14)   
))

# Animacja klatek
frames = []
for klatka in RobotAnimFrames:
    kx = [p[0] for p in klatka]
    ky = [p[1] for p in klatka]
    kz = [p[2] for p in klatka]
    frames.append(go.Frame(data=[go.Scatter3d(x=kx, y=ky, z=kz)], traces=[1]))

fig.frames = frames

fig.update_layout(
    title="Symulacja IK CCD - Robot 6 DoF",
    updatemenus=[dict(
        type="buttons",
        showactive=False,
        x=0.1, y=0.9,
        buttons=[dict(
            label="▶ START", 
            method="animate", 
            args=[None, dict(frame=dict(duration=80, redraw=True), fromcurrent=True)]
        )]
    )],
    scene=dict(
        xaxis=dict(range=[-1.5, 1.5], title='Oś X'),
        yaxis=dict(range=[-1.5, 1.5], title='Oś Y'),
        zaxis=dict(range=[0.0, 1.5], title='Oś Z'),
        aspectmode='cube',
        camera=dict(eye=dict(x=1.5, y=1.5, z=0.8))
    ),
    margin=dict(l=0, r=0, b=0, t=50)
)

pliky = "tests_python/wizuale.html"
fig.write_html(pliky)