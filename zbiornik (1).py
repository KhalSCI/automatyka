from dash import Dash, dcc, html, Input, Output, State, ctx
import plotly.graph_objects as go

app = Dash(__name__)

control_signal_max = 10.0
control_signal_min = 0.0
flow_max = 0.05
flow_min = 0.0
integral_time = 0.5
tank_area = 1.5
flow_coefficient = 0.035
sampling_time = 0.1
total_time = 3600
num_samples = int(total_time / sampling_time) + 1

previous_graph_data = []

def simulate(desired_height, proportional_gain):
    time_points = [0.0]
    error = [0.0]
    height = [0.0]
    control_signal = [0.0]
    inflow_rate = [0.0]
    outflow_rate = [0.0]
    proportional_integral_control = [0.0]

    for _ in range(num_samples):
        time_points.append(time_points[-1] + sampling_time)
        error.append(desired_height - height[-1])
        proportional_integral_control.append(
            proportional_gain * error[-1] + proportional_gain * sampling_time * sum(error) / integral_time
        )
        control_signal.append(
            min(control_signal_max, max(control_signal_min, proportional_integral_control[-1]))
        )
        inflow_rate.append((flow_max - flow_min) / (control_signal_max - control_signal_min) *
                           (control_signal[-1] - control_signal_min) + flow_min)
        height.append((inflow_rate[-1] - outflow_rate[-1]) * sampling_time / tank_area + height[-1])
        outflow_rate.append(flow_coefficient * (height[-1]) ** 0.5)

    return time_points, height, inflow_rate, outflow_rate, control_signal, error


app.layout = html.Div([
    html.H4('Automatyka'),

    html.Label("Desired Height"),
    dcc.Slider(id="desired_height_slider", min=0, max=3, step=0.1, value=1.5,
               marks={i: f"{i:.1f}" for i in range(0, 4)}),

    html.Label("Proportional Gain"),
    dcc.Slider(id="proportional_gain_slider", min=0.01, max=0.1, step=0.01, value=0.02,
               marks={i / 100: f"{i / 100:.2f}" for i in range(1, 11)}),
    html.Button("Simulate", id="simulate_button", n_clicks=0),

    html.Button("Reset", id="reset_button", n_clicks=0),
    html.P("You need to reset graph data before changing the graph type."),
    html.P("Choose graph:"),
    dcc.Dropdown(
        id="dropdown",
        options=[
            {'label': 'Desired Height vs Actual Height', 'value': 'H_zad/H'},
            {'label': 'Inflow vs Outflow', 'value': 'Qd/Qo'},
            {'label': 'Control Signal', 'value': 'U_Pi/U'},
            {'label': 'Error', 'value': 'error'},
        ],
        value='H_zad/H',
        clearable=False,
    ),

    dcc.Graph(id="graph"),
])


@app.callback(
    Output("graph", "figure"),
    [Input("simulate_button", "n_clicks"),
     Input("reset_button", "n_clicks"),
     Input("dropdown", "value"),
     Input("desired_height_slider", "value"),
     Input("proportional_gain_slider", "value")],
    State("dropdown", "value")
)
def update_graph(n,x,selected_graph, desired_height, proportional_gain, previous_selected_graph):
    global previous_graph_data
    fig = go.Figure()

    if ctx.triggered_id == "reset_button":
        fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Value')
        previous_graph_data = []
        return fig

    if selected_graph != previous_selected_graph:
        previous_graph_data = []

    elif ctx.triggered_id == "simulate_button":

        time_points, height, inflow_rate, outflow_rate, control_signal, error = simulate(desired_height,
                                                                                         proportional_gain)

        for graph_data in previous_graph_data:
            for trace in graph_data:
                trace.update(line=dict(color='gray', dash='dash'))
                fig.add_trace(trace)

        new_graph_data = []
        if selected_graph == 'H_zad/H':
            new_graph_data.append(
                go.Scatter(x=time_points, y=[desired_height] * len(time_points), mode='lines', name='Desired Height'))
            new_graph_data.append(go.Scatter(x=time_points, y=height, mode='lines', name='Actual Height'))
            fig.update_layout(title='Desired Height vs Actual Height', xaxis_title='Time [s]', yaxis_title='Height [m]')

        elif selected_graph == 'Qd/Qo':
            new_graph_data.append(go.Scatter(x=time_points, y=inflow_rate, mode='lines', name='Inflow Rate'))
            new_graph_data.append(go.Scatter(x=time_points, y=outflow_rate, mode='lines', name='Outflow Rate'))
            fig.update_layout(title='Inflow Rate vs Outflow Rate', xaxis_title='Time [s]', yaxis_title='Flow [m^3/s]')

        elif selected_graph == 'U_Pi/U':
            new_graph_data.append(go.Scatter(x=time_points, y=control_signal, mode='lines', name='Control Signal'))
            fig.update_layout(title='Control Signal in Time', xaxis_title='Time [s]', yaxis_title='Signal')

        elif selected_graph == 'error':
            new_graph_data.append(go.Scatter(x=time_points, y=error, mode='lines', name='Error'))
            fig.update_layout(title='Error over Time', xaxis_title='Time [s]', yaxis_title='Error [m]')

        for trace in new_graph_data:
            fig.add_trace(trace)

        previous_graph_data.append(new_graph_data)
        if len(previous_graph_data) > 5:
            previous_graph_data.pop(0)

    return fig


if __name__ == '__main__':
    app.run_server(debug=True)


# TODO:
#  Documenation for the code
#  -------------------------------------
#  control_signal_max -> Maksymalna wartość sygnału sterującego -> Umax
#  control_signal_min -> Minimalna wartość sygnału sterującego -> Umin
#  flow_max -> Maksymalny przepływ -> Qdmax
#  flow_min -> Minimalny przepływ -> Qdmin
#  proportional_gain -> Wzmocnienie proporcjonalne -> Kp
#  integral_time -> Czas całkowania -> Ti
#  desired_height -> Żądana wysokość cieczy w zbiorniku -> H_zad
#  tank_area -> Powierzchnia zbiornika -> A
#  flow_coefficient -> Współczynnik przepływu -> beta
#  sampling_time -> Czas próbkowania -> Tp
#  total_time -> Całkowity czas symulacji -> time
#  num_samples -> int(total_time / sampling_time) + 1  -> Liczba próbek  -> N
#  outflow_rate -> Wypływ -> Qo
#  inflow_rate -> Wpływ -> Qd
#  control_signal -> Sygnał sterujący -> U
#  time_points -> Punkty czasowe -> t
#  error -> Błąd -> e
#  proportional_integral_control -> Sterowanie PI -> U_Pi
#  height -> Wysokość cieczy w zbiorniku -> H
#  simulate -> Funkcja symulująca działanie regulatora PI
#  update_graph -> Funkcja aktualizująca wykresy
