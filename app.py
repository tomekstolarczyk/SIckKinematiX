import dash
from dash import dcc, html, Input, Output
import plotly.graph_objects as go
import numpy as np
import math

# --- IMPORT TWOJEGO PAKIETU Z C ---
import sickkinematix.c_kinematix as sk 

# --- KONFIGURACJA ROBOTA (UR5) ---
a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


# Budujemy kapsułę w C
robot = sk.build_robot(a, alpha, d, offsets)

##########################################################################
##########################################################################


bone_colors = ['#00f3ff', '#00d0ff', '#00adff', '#008aff', '#0066ff', '#0044ff']

app = dash.Dash(__name__)

# --- GENEROWANIE SUWAKÓW ---
sliders_list = []
for i in range(6):
    sliders_list.append(
        html.Div([
            html.Label(f"Joint {i+1} (\u03B8)", style={'color': '#00d0ff'}),
            dcc.Slider(
                id=f'slider-j{i+1}', min=-np.pi, max=np.pi, step=0.05, value=0.0,
                marks={-3.14: '-π', 0: '0', 3.14: 'π'},
                tooltip={"placement": "bottom", "always_visible": False},
                updatemode='drag' # PŁYNNE ODŚWIEŻANIE
            )
        ], style={'marginBottom': '20px'})
    )

# --- 1. LAYOUT ---
app.layout = html.Div(style={'backgroundColor': '#0f1115', 'color': '#32ff7e', 'height': '100vh', 'display': 'flex', 'fontFamily': 'Courier New'}, children=[
    
    # LEWA KOLUMNA
    html.Div(style={'width': '30%', 'padding': '20px', 'borderRight': '1px solid #2a2d35'}, children=[
        html.H2("SICK KINEMATIX", style={'color': '#ff007f'}),
        html.P("FORWARD KINEMATICS CONTROL"),
        html.Div(sliders_list, style={'marginTop': '30px'})
    ]),

    # PRAWA KOLUMNA
    html.Div(style={'width': '70%', 'padding': '0'}, children=[
        dcc.Graph(id='robot-3d-graph', style={'height': '100vh'})
    ])
])

# --- 2. CALLBACKS (LOGIKA) ---
@app.callback(
    Output('robot-3d-graph', 'figure'),
    [Input(f'slider-j{i+1}', 'value') for i in range(6)]
)
def update_robot(t1, t2, t3, t4, t5, t6):
    
    # PRAWDZIWA MATMA Z TWOJEGO C
    # fkResults to lista 7 elementów (Baza + 6 przegubów), każdy to krotka (x, y, z)
    fkResults = sk.fk(robot, t1, t2, t3, t4, t5, t6)
    
    # Rozpakowujemy X, Y, Z dla Plotly
    xs = [pt[0] for pt in fkResults]
    ys = [pt[1] for pt in fkResults]
    zs = [pt[2] for pt in fkResults]

    fig = go.Figure()

    # Rysujemy kości robota
    for j in range(6):
        fig.add_trace(go.Scatter3d(
            x=[xs[j], xs[j+1]], y=[ys[j], ys[j+1]], z=[zs[j], zs[j+1]],
            mode='lines', line=dict(width=10, color=bone_colors[j]), showlegend=False
        ))

    # Przeguby
    fig.add_trace(go.Scatter3d(
        x=xs, y=ys, z=zs, mode='markers',
        marker=dict(size=8, color='#ff007f', line=dict(color='white', width=1)), showlegend=False
    ))

    # Ustawienia sceny (Ciemny motyw)
    fig.update_layout(
        uirevision='blokada-kamery', # ZAPOBIEGA RESETOWANIU KAMERY
        paper_bgcolor='#0f1115', plot_bgcolor='#0f1115', margin=dict(l=0, r=0, t=0, b=0),
        scene=dict(
            xaxis=dict(range=[-1.0, 1.0], backgroundcolor='#171a21', gridcolor='#2a2d35', color='white'),
            yaxis=dict(range=[-1.0, 1.0], backgroundcolor='#171a21', gridcolor='#2a2d35', color='white'),
            zaxis=dict(range=[-1.0, 1.0], backgroundcolor='#171a21', gridcolor='#2a2d35', color='white'),
            aspectmode='cube' # Idealny sześcian (od -1m do +1m w każdą stronę)
        )
    )
    return fig

if __name__ == '__main__':
    app.run(debug=True)