import plotly.graph_objects as go

controlsignal_max = 10.0  # Maksymalna wartość sygnału sterującego -> Umax
control_signal_min = 0.0  # Minimalna wartość sygnału sterującego -> Umin
flow_max = 0.05  # Maksymalny przepływ -> Qdmax
flow_min = 0.0  # Minimalny przepływ -> Qdmin
proportional_gain = 0.02  # Wzmocnienie proporcjonalne -> Kp
integral_time = 0.5  # Czas całkowania -> Ti
desired_height = 1.5  # Żądana wysokość cieczy w zbiorniku -> H_zad
tank_area = 1.5  # Powierzchnia zbiornika -> A
flow_coefficient = 0.035  # Współczynnik przepływu -> beta
sampling_time = 0.1  # Czas próbkowania -> Tp
total_time = 3600  # Całkowity czas symulacji -> time
num_samples = int(total_time / sampling_time) + 1  # Liczba próbek  -> N
outflow_rate = [0.0]  # Wypływ -> Qo
inflow_rate = [0.0]  # Wpływ -> Qd
control_signal = [0.0]  # Sygnał sterujący -> U
time_points = [0.0]  # Punkty czasowe -> t
error = [0.0]  # Błąd -> e
proportional_integral_control = [0.0]  # Sterowanie PI -> U_Pi
height = [0.0]  # Wysokość cieczy w zbiorniku -> H

for _ in range(num_samples):
    time_points.append(time_points[-1] + sampling_time)
    error.append(desired_height - height[-1])
    proportional_integral_control.append(proportional_gain * error[-1] + proportional_gain * sampling_time * sum(error) / integral_time)
    control_signal.append(min(controlsignal_max, max(control_signal_min, proportional_integral_control[-1])))
    inflow_rate.append((flow_max - flow_min) / (controlsignal_max - control_signal_min) * (control_signal[-1] - control_signal_min) + flow_min)
    height.append((inflow_rate[-1] - outflow_rate[-1]) * sampling_time / tank_area + height[-1])
    outflow_rate.append(flow_coefficient * (height[-1]) ** 0.5)

fig = go.Figure()
fig.add_trace(go.Scatter(x=time_points, y=height, mode='lines', name='Height'))
fig.update_layout(
    title='Tank height over time',
    xaxis_title='Time [s]',
    yaxis_title='Height [m]'
)
fig.show()