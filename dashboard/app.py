import dash
from dash import Dash, dcc, html, Input, Output, State
import plotly.graph_objects as go 
import numpy as np  
import math 
import sickkinematix.c_kinematix as sk 

#################################################### config ####################################################

# robot 
a = [0.0, 0.0, -0.425, -0.39225, 0.0, 0.0]
alpha = [0.0, math.pi/2.0, 0.0, 0.0, math.pi/2.0, -math.pi/2.0]
d = [0.089159, 0.0, 0.0, 0.10915, 0.09465, 0.0823]
offsets = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot = sk.build_robot(a, alpha, d, offsets)

# glowny obiekt dashowy
app = Dash(__name__) 

# pamiec stanu - Idealna startowa pozycja UR5 
ostatnie_katy_ik = [0.0, -math.pi/2, 0.0, -math.pi/2, 0.0, 0.0] 
chmura_x, chmura_y, chmura_z, chmura_w = None, None, None, None

BG_COLOR = '#0f172a'         
PANEL_COLOR = '#1e293b'      
GRID_COLOR = '#334155'       
TEXT_COLOR = '#f8fafc'       
TEXT_MUTED = '#94a3b8'       
ACCENT_COLOR = '#818cf8'     
WARN_COLOR = '#fbbf24'       

BONE_COLOR = '#e2e8f0'     
BASE_COLOR = '#334155'     
AXIS_COLOR = '#10b981'     
BONE_WIDTHS = [20, 16, 12, 10, 8, 6] 

#################################################### helper funcs ####################################################

# Euler do Macierzy (Dla DLS)
def euler_to_matrix(tx, ty, tz, rx, ry, rz):
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    
    return [
        cy*cz, sx*sy*cz - cx*sz, cx*sy*cz + sx*sz, tx,
        cy*sz, sx*sy*sz + cx*cz, cx*sy*sz - sx*cz, ty,
        -sy,   sx*cy,            cx*cy,            tz,
        0.0,   0.0,              0.0,              1.0
    ]

#################################################### layout ####################################################

tab_style = {
    'borderBottom': f'1px solid {GRID_COLOR}',
    'padding': '12px',
    'fontWeight': 'bold',
    'backgroundColor': PANEL_COLOR,
    'color': TEXT_MUTED,
    'borderTop': 'none',
    'borderLeft': 'none',
    'borderRight': 'none'
}

tab_selected_style = {
    'borderTop': 'none',
    'borderBottom': f'3px solid {ACCENT_COLOR}',
    'borderLeft': 'none',
    'borderRight': 'none',
    'backgroundColor': PANEL_COLOR,
    'color': TEXT_COLOR,
    'padding': '12px',
    'fontWeight': 'bold'
}

app.layout = html.Div(
    style={'backgroundColor': BG_COLOR, 'height': '100vh', 'display': 'flex', 'fontFamily': '"Roboto", "Segoe UI", sans-serif'}, 
    children=[

    # LEWA KOLUMNA 
    html.Div(
        style={'width': '28%', 'minWidth': '350px', 'padding': '30px', 'backgroundColor': PANEL_COLOR, 'borderRight': f'1px solid {GRID_COLOR}', 'boxShadow': '4px 0 20px rgba(0,0,0,0.3)', 'zIndex': 10, 'overflowY': 'auto'}, 
        children=[
            html.H2("SIck KinematiX", style={'color': TEXT_COLOR, 'marginBottom': '5px', 'fontWeight': '700', 'letterSpacing': '1px'}),
            html.P("6-DoF MANIPULATOR CONTROL", style={'color': ACCENT_COLOR, 'fontSize': '11px', 'fontWeight': 'bold', 'letterSpacing': '1.5px', 'marginBottom': '30px'}),
            
            dcc.Tabs(id='tabs-mode', value='tab-fk', children=[
                dcc.Tab(label='FK', value='tab-fk', style=tab_style, selected_style=tab_selected_style),
                dcc.Tab(label='CCD (Pos)', value='tab-ccd', style=tab_style, selected_style=tab_selected_style),
                dcc.Tab(label='DLS (Pose)', value='tab-dls', style=tab_style, selected_style=tab_selected_style),
                dcc.Tab(label='Yoshikawa', value='tab-ws', style=tab_style, selected_style=tab_selected_style),
            ], style={'marginBottom': '25px'}),
            
            # ZAKLADKA 1: FORWARD KINEMATICS 
            html.Div(id='panel-fk', children=[
                html.P("Live joint angle manipulation. Green vectors indicate axes of rotation.", style={'color': TEXT_MUTED, 'fontSize': '13px', 'marginBottom': '25px'})
            ] + [
                html.Div([
                    html.Label(f"Joint {i+1} (\u03B8)", style={'color': TEXT_COLOR, 'fontSize': '13px', 'fontWeight': '500'}),
                    dcc.Slider(id=f'slider-j{i+1}', min=-np.pi, max=np.pi, step=0.01, value=ostatnie_katy_ik[i],
                               marks={-3.14: {'label': '-π', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 3.14: {'label': 'π', 'style': {'color': TEXT_MUTED}}}, 
                               updatemode='drag', tooltip={"placement": "top", "always_visible": False})
                ], style={'marginBottom': '25px'}) for i in range(6)
            ]),
            
            # ZAKLADKA 2: IK CCD 
            html.Div(id='panel-ccd', children=[
                html.H4("Positional Inverse Kinematics", style={'color': TEXT_COLOR, 'fontSize': '15px', 'fontWeight': '600', 'marginBottom': '8px'}),
                html.P("CCD heuristic algorithm. Rapidly converges to target coordinates, ignoring end-effector orientation.", style={'color': TEXT_MUTED, 'fontSize': '13px', 'marginBottom': '25px'})
            ] + [
                html.Div([
                    html.Label(f"Target {axis.upper()} (m)", style={'color': TEXT_COLOR, 'fontSize': '13px', 'fontWeight': '500'}),
                    dcc.Slider(id=f'slider-ccd-{axis}', min=-0.8, max=0.8, step=0.01, value=0.0, 
                               marks={-0.8: {'label': '-0.8', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 0.8: {'label': '0.8', 'style': {'color': TEXT_MUTED}}}, 
                               updatemode='drag', tooltip={"placement": "top", "always_visible": False})
                ], style={'marginBottom': '25px'}) for axis in ['x', 'y', 'z']
            ]),

            # ZAKLADKA 3: IK DLS 
            html.Div(id='panel-dls', children=[
                html.H4("6-DoF Pose Tracking", style={'color': TEXT_COLOR, 'fontSize': '15px', 'fontWeight': '600', 'marginBottom': '8px'}),
                html.P("Damped Least Squares algorithm. Solves for exact position and orientation simultaneously.", style={'color': TEXT_MUTED, 'fontSize': '13px', 'marginBottom': '25px'}),
                
                html.H5("Position (XYZ)", style={'color': TEXT_COLOR, 'fontSize': '12px', 'textTransform': 'uppercase', 'letterSpacing': '1px', 'marginBottom': '10px'}),
                html.Div([
                    dcc.Slider(id='slider-dls-x', min=-0.8, max=0.8, step=0.01, value=0.3, marks={-0.8: {'label': '-0.8', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 0.8: {'label': '0.8', 'style': {'color': TEXT_MUTED}}}, updatemode='drag', tooltip={"placement": "top", "always_visible": False}),
                ], style={'marginBottom': '25px'}),
                html.Div([
                    dcc.Slider(id='slider-dls-y', min=-0.8, max=0.8, step=0.01, value=0.0, marks={-0.8: {'label': '-0.8', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 0.8: {'label': '0.8', 'style': {'color': TEXT_MUTED}}}, updatemode='drag', tooltip={"placement": "top", "always_visible": False}),
                ], style={'marginBottom': '25px'}),
                html.Div([
                    dcc.Slider(id='slider-dls-z', min=-0.1, max=1.0, step=0.01, value=0.3, marks={-0.1: {'label': '-0.1', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 1.0: {'label': '1.0', 'style': {'color': TEXT_MUTED}}}, updatemode='drag', tooltip={"placement": "top", "always_visible": False}),
                ], style={'marginBottom': '25px'}),
                
                html.H5("Rotation (Roll-Pitch-Yaw)", style={'color': TEXT_COLOR, 'fontSize': '12px', 'textTransform': 'uppercase', 'letterSpacing': '1px', 'marginTop': '10px', 'marginBottom': '10px'}),
                html.Div([
                    dcc.Slider(id='slider-dls-rx', min=-3.14, max=3.14, step=0.1, value=0.0, marks={-3.14: {'label': '-π', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 3.14: {'label': 'π', 'style': {'color': TEXT_MUTED}}}, updatemode='drag', tooltip={"placement": "top", "always_visible": False}),
                ], style={'marginBottom': '25px'}),
                html.Div([
                    dcc.Slider(id='slider-dls-ry', min=-3.14, max=3.14, step=0.1, value=1.57, marks={-3.14: {'label': '-π', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 3.14: {'label': 'π', 'style': {'color': TEXT_MUTED}}}, updatemode='drag', tooltip={"placement": "top", "always_visible": False}),
                ], style={'marginBottom': '25px'}),
                html.Div([
                    dcc.Slider(id='slider-dls-rz', min=-3.14, max=3.14, step=0.1, value=0.0, marks={-3.14: {'label': '-π', 'style': {'color': TEXT_MUTED}}, 0: {'label': '0', 'style': {'color': TEXT_MUTED}}, 3.14: {'label': 'π', 'style': {'color': TEXT_MUTED}}}, updatemode='drag', tooltip={"placement": "top", "always_visible": False}),
                ], style={'marginBottom': '25px'}),
            ]),

            # ZAKLADKA 4: WORKSPACE ANALYSIS 
            html.Div(id='panel-ws', children=[
                html.H4("Agility Analysis (Yoshikawa)", style={'color': TEXT_COLOR, 'fontSize': '15px', 'fontWeight': '600', 'marginBottom': '8px'}),
                html.P("Monte-Carlo point cloud reflecting full manipulator agility. Low-index regions indicate proximity to mathematical singularities.", style={'color': TEXT_MUTED, 'fontSize': '13px', 'lineHeight': '1.5'}),
                html.Hr(style={'borderColor': GRID_COLOR, 'margin': '25px 0'}),
                html.Label("Agility Threshold Filter", style={'color': TEXT_COLOR, 'fontSize': '13px', 'fontWeight': '500'}),
                html.P("Move right to hide outer boundaries and reveal the high-agility core workspace.", style={'color': TEXT_MUTED, 'fontSize': '11px', 'marginTop': '5px', 'marginBottom': '15px'}),
                html.Div(style={'paddingBottom': '10px'}, children=[
                    dcc.Slider(id='w-filter', min=0.0, max=0.1, step=0.005, value=0.02, 
                               marks={0: {'label': '0.0', 'style': {'color': TEXT_MUTED}}, 0.1: {'label': '0.1', 'style': {'color': TEXT_MUTED}}}, 
                               updatemode='drag', tooltip={"placement": "top", "always_visible": False})
                ]),
                html.Hr(style={'borderColor': GRID_COLOR, 'margin': '15px 0'}),
                html.Label("3D Slicer / Scanner", style={'color': TEXT_COLOR, 'fontSize': '13px', 'fontWeight': '500'}),
                html.P("Cut the cloud along the end-effector's plane to see inside.", style={'color': TEXT_MUTED, 'fontSize': '11px', 'marginTop': '5px', 'marginBottom': '15px'}),
                dcc.RadioItems(
                    id='ws-slice-mode',
                    options=[
                        {'label': ' Full 3D', 'value': 'full'},
                        {'label': ' Slice X', 'value': 'x'},
                        {'label': ' Slice Y', 'value': 'y'},
                        {'label': ' Slice Z', 'value': 'z'}
                    ],
                    value='full',
                    labelStyle={'display': 'inline-block', 'marginRight': '15px', 'color': TEXT_COLOR, 'fontSize': '13px', 'cursor': 'pointer'}
                )
            ])
        ]
    ),

    # PRAWA KOLUMNA 
    html.Div(style={'width': '72%', 'padding': '0', 'position': 'relative'}, children=[
        dcc.Graph(id='robot-3d-graph', style={'height': '100vh'}, config={'displayModeBar': False})
    ])
])


@app.callback(
    [Output('robot-3d-graph', 'figure'), 
     Output('panel-fk', 'style'), Output('panel-ccd', 'style'), Output('panel-dls', 'style'), Output('panel-ws', 'style')] +
    [Output(f'slider-j{i+1}', 'value') for i in range(6)] +
    [Output(f'slider-ccd-{axis}', 'value') for axis in ['x', 'y', 'z']],
    [Input('tabs-mode', 'value'), Input('w-filter', 'value'), Input('ws-slice-mode', 'value')] +
    [Input(f'slider-j{i+1}', 'value') for i in range(6)] +
    [Input(f'slider-ccd-{axis}', 'value') for axis in ['x', 'y', 'z']] +
    [Input('slider-dls-x', 'value'), Input('slider-dls-y', 'value'), Input('slider-dls-z', 'value'),
     Input('slider-dls-rx', 'value'), Input('slider-dls-ry', 'value'), Input('slider-dls-rz', 'value')]
)
def master_update(tab, w_filter, slice_mode, 
                  j1, j2, j3, j4, j5, j6, 
                  cx, cy, cz, 
                  dx, dy, dz, drx, dry, drz):
    global ostatnie_katy_ik, chmura_x, chmura_y, chmura_z, chmura_w
    
    ctx = dash.callback_context
    trigger = ctx.triggered[0]['prop_id'] if ctx.triggered else 'init'

    # 1. OBSLUGA LOGIKI NA PODSTAWIE ZAKŁADKI 
    if tab == 'tab-fk':
        ostatnie_katy_ik = [j1, j2, j3, j4, j5, j6]
        obecne_xyz = sk.fk(robot, *ostatnie_katy_ik)[-1]
        cx, cy, cz = obecne_xyz[0], obecne_xyz[1], obecne_xyz[2]

    elif tab == 'tab-ccd':
        ostatnie_katy_ik = sk.ik_ccd(robot, cx, cy, cz, *ostatnie_katy_ik, 1000, 0.005)
        j1, j2, j3, j4, j5, j6 = ostatnie_katy_ik

    elif tab == 'tab-dls' or tab == 'tab-ws':
        target_matrix = euler_to_matrix(dx, dy, dz, drx, dry, drz)
        ostatnie_katy_ik = sk.ik_dls(robot, target_matrix, *ostatnie_katy_ik, 300, 0.005, 0.1, 0.1)
        j1, j2, j3, j4, j5, j6 = ostatnie_katy_ik

    # 2. POBIERANIE DANYCH Z C API
    fkResults = sk.fk(robot, *ostatnie_katy_ik)
    Xs, Ys, Zs = [v[0] for v in fkResults], [v[1] for v in fkResults], [v[2] for v in fkResults]
    Zx, Zy, Zz = [v[3] for v in fkResults], [v[4] for v in fkResults], [v[5] for v in fkResults]

    fig = go.Figure()

    # PODŁOGA I CIEN
    fig.add_trace(go.Mesh3d(x=[-1, 1, 1, -1], y=[-1, -1, 1, 1], z=[0,0,0,0], color=GRID_COLOR, opacity=0.3, hoverinfo='skip'))
    fig.add_trace(go.Scatter3d(x=Xs, y=Ys, z=[0]*len(Zs), mode='lines', line=dict(width=8, color='rgba(0, 0, 0, 0.8)'), hoverinfo='skip'))

    # ZAKLADKA 4: WORKSPACE 
    if tab == 'tab-ws':
        if chmura_x is None:
            chmura_x, chmura_y, chmura_z, chmura_w = sk.workspace(robot, 150000) 
        
        # Wokselizacja
        GRID = 0.06
        cx_vox = np.round(np.array(chmura_x) / GRID) * GRID
        cy_vox = np.round(np.array(chmura_y) / GRID) * GRID
        cz_vox = np.round(np.array(chmura_z) / GRID) * GRID
        cw_arr = np.array(chmura_w)

        maska = (cw_arr >= w_filter)
        
        ee_x, ee_y, ee_z = Xs[-1], Ys[-1], Zs[-1] 

        SLICE_THICKNESS = 0.03 
        
        if slice_mode == 'x':
            maska = maska & (np.abs(cx_vox - dx) < SLICE_THICKNESS)
        elif slice_mode == 'y':
            maska = maska & (np.abs(cy_vox - dy) < SLICE_THICKNESS)
        elif slice_mode == 'z':
            maska = maska & (np.abs(cz_vox - dz) < SLICE_THICKNESS)
        
        fig.add_trace(go.Scatter3d(
            x=cx_vox[maska], y=cy_vox[maska], z=cz_vox[maska], mode='markers',
            marker=dict(
                size=6, opacity=0.8,     
                color=cw_arr[maska], 
                colorscale='Viridis',  
                cmin=0.0, cmax=0.1,    
                showscale=True, 
                colorbar=dict(title="w (Zwinność)", tickfont=dict(color=TEXT_COLOR), x=0)
            ), 
            hoverinfo='skip' 
        ))

    # RYSOWANIE ROBOTA 
    fig.add_trace(go.Scatter3d(x=[0, 0], y=[0, 0], z=[0, Zs[0]], mode='lines', line=dict(width=30, color=BASE_COLOR), hoverinfo='skip'))
    for j in range(6):
        fig.add_trace(go.Scatter3d(x=[Xs[j], Xs[j+1]], y=[Ys[j], Ys[j+1]], z=[Zs[j], Zs[j+1]], mode='lines', line=dict(width=BONE_WIDTHS[j], color=BONE_COLOR), hoverinfo='skip'))
        
    # Przeguby
    fig.add_trace(go.Scatter3d(x=Xs, y=Ys, z=Zs, mode='markers', marker=dict(size=[14, 12, 10, 10, 10, 10, 14], color=ACCENT_COLOR, line=dict(color=BG_COLOR, width=2)), hoverinfo='skip'))

    # OSIE OBROTU 
    for j in range(len(Xs)):  
        L = 0.06  
        fig.add_trace(go.Scatter3d(
            x=[Xs[j], Xs[j] + Zx[j]*L], 
            y=[Ys[j], Ys[j] + Zy[j]*L], 
            z=[Zs[j], Zs[j] + Zz[j]*L],
            mode='lines', line=dict(color=AXIS_COLOR, width=4), hoverinfo='skip'
        ))

    # CELE DLA IK 
    if tab == 'tab-ccd':
        fig.add_trace(go.Scatter3d(x=[cx], y=[cy], z=[cz], mode='markers', marker=dict(size=12, color=WARN_COLOR, symbol='cross'), name="Cel CCD", hoverinfo='skip'))
    
    elif tab == 'tab-dls' or tab == 'tab-ws':
        fig.add_trace(go.Scatter3d(x=[dx], y=[dy], z=[dz], mode='markers', marker=dict(size=8, color=WARN_COLOR, symbol='diamond'), hoverinfo='skip'))
        target_matrix = euler_to_matrix(dx, dy, dz, drx, dry, drz)
        tzx, tzy, tzz = target_matrix[2], target_matrix[6], target_matrix[10]
        fig.add_trace(go.Scatter3d(
            x=[dx, dx + tzx*0.2], y=[dy, dy + tzy*0.2], z=[dz, dz + tzz*0.2],
            mode='lines', line=dict(color=WARN_COLOR, width=4, dash='dash'), hoverinfo='skip'
        ))

    # USTAWIENIA SCENY
    fig.update_layout(
        showlegend=False, uirevision='const', paper_bgcolor=BG_COLOR, plot_bgcolor=BG_COLOR, margin=dict(l=0, r=0, t=0, b=0),
        scene=dict(
            xaxis=dict(range=[-1.0, 1.0], backgroundcolor=BG_COLOR, gridcolor=GRID_COLOR, showbackground=True, zerolinecolor=GRID_COLOR, tickfont=dict(color=TEXT_COLOR)),
            yaxis=dict(range=[-1.0, 1.0], backgroundcolor=BG_COLOR, gridcolor=GRID_COLOR, showbackground=True, zerolinecolor=GRID_COLOR, tickfont=dict(color=TEXT_COLOR)),
            zaxis=dict(range=[0.0, 1.2], backgroundcolor=BG_COLOR, gridcolor=GRID_COLOR, showbackground=True, zerolinecolor=GRID_COLOR, tickfont=dict(color=TEXT_COLOR)),
            aspectmode='manual', aspectratio=dict(x=1, y=1, z=0.6)
        )
    )

    # Przelaczanie widocznosci paneli bocznych
    s_fk = {'display': 'block'} if tab == 'tab-fk' else {'display': 'none'}
    s_ccd = {'display': 'block'} if tab == 'tab-ccd' else {'display': 'none'}
    s_dls = {'display': 'block'} if tab == 'tab-dls' else {'display': 'none'}
    s_ws = {'display': 'block'} if tab == 'tab-ws' else {'display': 'none'}
    
    return (fig, s_fk, s_ccd, s_dls, s_ws, j1, j2, j3, j4, j5, j6, cx, cy, cz)

###################################################################################################################################

if __name__ == '__main__':
    app.run(debug=True)