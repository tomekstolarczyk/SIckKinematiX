# aplikacja webowa w dashu - do wizualizacji naszego pakietu

# importy
from dash import Dash, dcc, html, Input, Output 
import plotly.graph_objects as go 
import numpy as np  
import math 
import sickkinematix.c_kinematix as sk 

# konfigurujemy naszego robota (model UR5) 
a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot = sk.build_robot(a, alpha, d, offsets)


app = Dash() # tworzymy glowny obiekt naszej aplikacji
bone_colors = ['#00f3ff', '#00d0ff', '#00adff', '#008aff', '#0066ff', '#0044ff']


# lista sliderow
sliders_list = []
for i in range(6): 
    sliders_list.append(
        html.Div([ 
            html.Label(f"Joint {i+1} (\u03B8)", style={'color': '#00d0ff'}), # etyiketa, \u03B8 to kod znaku theta
            dcc.Slider(
                id=f'slider-j{i+1}', #unikalne id slidera
                min=-np.pi, max=np.pi, step=0.05, value=0.0, # limity suwakow
                # osie zakresy suwakow zaznaczone
                marks={ 
                    -np.pi: {'label': '-π', 'style': {'color': '#00d0ff'}},
                    0: {'label': '0', 'style': {'color': '#00d0ff'}},
                    np.pi: {'label': 'π', 'style': {'color': '#00d0ff'}}
                },
                tooltip={"placement": "bottom", "always_visible": False}, # dymek z wartoscia
                updatemode='drag') # wysylamy dane do pythona przy kazdym ruchu myszki
        ], 
        style={'marginBottom': '20px'})
    )

# 1 LAYOUT
app.layout = html.Div(
    style={'backgroundColor': '#0f1115', 'color': '#32ff7e', 'height': '100vh', 'display': 'flex', 'fontFamily': 'Courier New'}, 
    children=[

    # LEWA KOLUMNA
    html.Div(
        style={'width': '30%', 'padding': '20px', 'borderRight': '1px solid #2a2d35'}, 
        children=[
        html.H2("SICK KINEMATIX", style={'color': '#ff007f'}),
        html.P("FORWARD KINEMATICS CONTROL"),
        html.Div(sliders_list, style={'marginTop': '30px'})]),

    # PRAWA KOLUMNA
    html.Div(
        style={'width': '70%', 'padding': '0'}, 
        children=[dcc.Graph(id='robot-3d-graph', style={'height': '100vh'})])
])

# 2 CALLBACKS
@app.callback(
    Output('robot-3d-graph', 'figure'),
    [Input(f'slider-j{i+1}', 'value') for i in range(6)]
)
def update_robot(t1,t2,t3,t4,t5,t6):

    #wywolujemy nasze forward kinematics
    fkResults = sk.fk(robot, t1, t2, t3, t4, t5, t6)

    # rozpakowujemy listy do X, Y, Z dla Plotly
    Xs = [vector[0] for vector in fkResults]
    Ys = [vector[1] for vector in fkResults]
    Zs = [vector[2] for vector in fkResults]

    figure = go.Figure() # tworzymy obiekt do rysowania plotly

    # rysujemy linki robota
    for j in range(6):
        figure.add_trace(go.Scatter3d(
            x=[Xs[j], Xs[j+1]], 
            y=[Ys[j], Ys[j+1]], 
            z=[Zs[j], Zs[j+1]],
            mode='lines', 
            line=dict(width=10, color=bone_colors[j]), 
            showlegend=False))

    # rysujemy przeguby
    figure.add_trace(go.Scatter3d(
        x=Xs, 
        y=Ys, 
        z=Zs, 
        mode='markers',
        marker=dict(size=8, color='#ff007f', line=dict(color='white', width=1)), 
        showlegend=False))
    
    # Ustawienia sceny (Ciemny motyw)
    figure.update_layout(
        uirevision='blokada-kamery', # blokada kamery
        paper_bgcolor='#0f1115', plot_bgcolor='#0f1115', margin=dict(l=0, r=0, t=0, b=0),
        scene=dict(
            xaxis=dict(range=[-1.0, 1.0], backgroundcolor='#171a21', gridcolor='#2a2d35', color='white'),
            yaxis=dict(range=[-1.0, 1.0], backgroundcolor='#171a21', gridcolor='#2a2d35', color='white'),
            zaxis=dict(range=[-1.0, 1.0], backgroundcolor='#171a21', gridcolor='#2a2d35', color='white'),
            aspectmode='cube'
        )
    )
    
    return figure
    
if __name__ == '__main__':
    app.run(debug=True)