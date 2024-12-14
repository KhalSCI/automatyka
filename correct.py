from dash import Dash, dcc, html, Input, Output, State, ctx
import plotly.graph_objects as go
import numpy as np
import pandas as pd
# Constants
mass = 1000 # mass of the car in kg
drag_coefficient = 0.3  # drag coefficient
frontal_area = 2.2  # m^2
air_density = 1.225  # kg/m^3
rolling_resistance = 0.015
gravity = 9.81  # m/s^2
time_step = 0.05  # simulation time step in seconds
hill_start_time = 30  # Start time of the hill climb [s]
hill_duration = 20  # Duration of the hill climb [s]
rise = 5  # Vertical height in meters
run = 100  # Horizontal distance in meters
hill_slope = rise / run  # Slope of the hill as rise/run
hill_angle = np.arctan(hill_slope)  # Convert slope to angle in radians
previous_graph_data = {
    "speed": [],
    "throttle": [],
    "braking": [],
    "height": []
}
# PID Controller
class PIDController:
    def __init__(self, kp, ti, td):
        self.kp = kp
        self.ti = ti
        self.td = td
        self.prev_error = 0
        self.integral = 0
        self.first_cycle = True

    def control(self, target_speed, current_speed):
        error = target_speed - current_speed
        self.integral += error * time_step
        if self.first_cycle:
            derivative = 0
            self.first_cycle = False
        else:
            derivative = (error - self.prev_error) / time_step
        derivative = max(min(derivative, 100), 0)

        self.prev_error = error
        # PID output
        #output = self.kp * error + (self.kp / self.ti) * self.integral + self.kp * self.td * derivative
        #output = self.kp * error
        output = self.kp * error + self.kp * 1/self.ti * self.integral + self.kp * self.td * derivative
        #df = pd.DataFrame({'error': error, 'integral': self.integral, 'derivative': derivative, 'output': output}, index=[0])
        #df.to_csv('pid.csv', mode='a', header=False, index=False)
        return output

# Force calculations
def calculate_forces(speed, slope_angle):
    # Aerodynamic drag force
    drag_force = 0.5 * drag_coefficient * frontal_area * air_density * speed**2
    # Rolling resistance force
    rolling_force = rolling_resistance * mass * gravity
    # Gravitational force due to slope
    slope_force = mass * gravity * np.sin(slope_angle)
    return drag_force + rolling_force + slope_force

# Simulation
def simulate_cruise_control(target_speed, duration, kp, ti, td):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    throttle = 0  # Initial throttle

    speed = 0  # Initial speed
    height = 0  # Initial height
    for t in time:
        pid_output = pid.control(target_speed, speed)
        if pid_output < 0:
            pid2 = pid_output * -1
            hamulec = max(0, min(pid2 / 100, 1))
        else:
            hamulec = 0
        # Normalize and clamp throttle
        throttle = max(0, min(pid_output / 100, 1))  # Scale PID output to [0, 1]
        throttle = 0.9 * throttle + 0.1 * (throttles[-1] if throttles else 0)  # Smooth changes
        
        # Determine the slope based on the current time and height
        #if hill_start_time <= t <= hill_start_time + hill_duration:
        #    slope_angle = hill_angle  # Uphill slope
        #elif hill_start_time + hill_duration < t <= hill_start_time + 2 * hill_duration:
        #    slope_angle = -hill_angle  # Downhill slope
        #else:
        slope_angle = 0  # Flat ground
#
        ## Reset slope to 0 when height reaches 0
        #if height < 0:
        #    slope_angle = 0
        
        # Calculate forces
        multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * multiplier  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * 15000  # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step

        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change
        #height = max(0, height + height_change)  # Prevent height from going negative

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights

def simulate_cruise_control_up(target_speed, duration, kp, ti, td):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    throttle = 0  # Initial throttle

    speed = 0  # Initial speed
    height = 0  # Initial height
    for t in time:
        pid_output = pid.control(target_speed, speed)
        if pid_output < 0:
            pid2 = pid_output * -1
            hamulec = max(0, min(pid2 / 100, 1))
        else:
            hamulec = 0
        # Normalize and clamp throttle
        throttle = max(0, min(pid_output / 100, 1))  # Scale PID output to [0, 1]
        throttle = 0.9 * throttle + 0.1 * (throttles[-1] if throttles else 0)  # Smooth changes
        
        # Determine the slope based on the current time and height
        if hill_start_time <= t <= hill_start_time + hill_duration:
            slope_angle = hill_angle  # Uphill slope
        else:
            slope_angle = 0  # Flat ground

        # Reset slope to 0 when height reaches 0
        if height < 0:
            slope_angle = 0
        
        # Calculate forces
        multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * multiplier  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * 15000  # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step

        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change
        #height = max(0, height + height_change)  # Prevent height from going negative

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights

def simulate_cruise_control_down(target_speed, duration, kp, ti, td):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    throttle = 0  # Initial throttle

    speed = 0  # Initial speed
    height = 0  # Initial height
    for t in time:
        pid_output = pid.control(target_speed, speed)
        if pid_output < 0:
            pid2 = pid_output * -1
            hamulec = max(0, min(pid2 / 100, 1))
        else:
            hamulec = 0
        # Normalize and clamp throttle
        if pid_output > 0:
            throttle = max(0, min(pid_output / 100, 1))  # Scale PID output to [0, 1]
            throttle = 0.9 * throttle + 0.1 * (throttles[-1] if throttles else 0)  # Smooth changes
        else:
            throttle = 0
        
        # Determine the slope based on the current time and height
        if hill_start_time <= t <= hill_start_time + hill_duration:
            slope_angle = -hill_angle  # Uphill slope
        else:
            slope_angle = 0  # Flat ground

        
        # Calculate forces
        multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * multiplier  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * 15000  # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step

        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change
        #height = max(0, height + height_change)  # Prevent height from going negative

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights
def simulate_cruise_control_upandownrandom(target_speed, duration, kp, ti, td):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    throttle = 0  # Initial throttle

    speed = 0  # Initial speed
    height = 0  # Initial height
    slope_angle = 0  # Initial slope angle
    for t in time:
        pid_output = pid.control(target_speed, speed)
        if pid_output < 0:
            pid2 = pid_output * -1
            hamulec = max(0, min(pid2 / 100, 1))
        else:
            hamulec = 0
        # Normalize and clamp throttle
        throttle = max(0, min(pid_output / 100, 1))  # Scale PID output to [0, 1]
        throttle = 0.9 * throttle + 0.1 * (throttles[-1] if throttles else 0)  # Smooth changes
        
        # Randomly change the slope angle at each time step
        if np.random.rand() < 0.05:  # 5% chance to change slope
            slope_angle = np.random.uniform(-hill_angle, hill_angle)  # Random slope between -hill_angle and hill_angle

        # Calculate forces
        multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * multiplier  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * 15000  # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step

        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change
        #height = max(0, height + height_change)  # Prevent height from going negative

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights

# Dash application
app = Dash(__name__)

app.layout = html.Div([
    html.H4('Cruise Control with PID Controller'),

    html.Label("Simulation Type"),
    dcc.Dropdown(
        id="simulation_type_dropdown",
        options=[
            {'label': 'Cruise Control', 'value': 'simulate_cruise_control'},
            {'label': 'Cruise Control Up', 'value': 'simulate_cruise_control_up'},
            {'label': 'Cruise Control Down', 'value': 'simulate_cruise_control_down'},
            {'label': 'Cruise Control Up and Down Random', 'value': 'simulate_cruise_control_upandownrandom'}
        ],
        value='simulate_cruise_control'
    ),

    html.Label("Target Speed (m/s)"),
    dcc.Slider(id="target_speed_slider", min=0, max=50, step=1, value=20,
               marks={i: f"{i}" for i in range(0, 51, 5)}),

    html.Label("Proportional Gain (Kp)"),
    dcc.Slider(id="kp_slider", min=0.0, max=50, step=2.0, value=5,
               marks={i: f"{i:.1f}" for i in range(0, 51, 5)}),

    html.Label("Integral Gain (ti)"),
    dcc.Slider(id="ti_slider", min=0.0, max=100.0, step=5.0, value=50,
               marks={i: f"{i:.1f}" for i in range(0, 101, 5)}),

    html.Label("Derivative Gain (td)"),
    dcc.Slider(id="td_slider", min=0.0, max=10, step=0.5, value=0,
               marks={i: f"{i:.1f}" for i in range(0, 11, 1)}),

    html.Label("Hill Start Time (s)"),
    dcc.Slider(id="hill_start_time_slider", min=0, max=100, step=1, value=30,
               marks={i: f"{i}" for i in range(0, 101, 10)}),

    html.Label("Hill Duration (s)"),
    dcc.Slider(id="hill_duration_slider", min=0, max=50, step=1, value=20,
               marks={i: f"{i}" for i in range(0, 51, 5)}),

    html.Label("Hill Angle (degrees)"),
    dcc.Slider(id="hill_angle_slider", min=0, max=45, step=1, value=10,
               marks={i: f"{i}" for i in range(0, 46, 5)}),

    html.Button("Simulate", id="simulate_button", n_clicks=0),
    html.Button("Reset", id="reset_button", n_clicks=0),

    dcc.Graph(id="speed_graph"),
    dcc.Graph(id="throttle_graph"),
    dcc.Graph(id="braking_graph"),
    dcc.Graph(id="height_graph"),
        # Interval component to update the displayed time_step value
    html.Div("Czas próbkowania: 0.05 s", id="time_step_display", style={'position': 'absolute', 'top': '10px', 'right': '10px'})

])

@app.callback(
    [Output("speed_graph", "figure"),
     Output("throttle_graph", "figure"),
     Output("braking_graph", "figure"),
     Output("height_graph", "figure"),
     ],
    [Input("simulate_button", "n_clicks"),
     Input("reset_button", "n_clicks"),
     Input("simulation_type_dropdown", "value"),
     Input("target_speed_slider", "value"),
     Input("kp_slider", "value"),
     Input("ti_slider", "value"),
     Input("td_slider", "value"),
     Input("hill_start_time_slider", "value"),
     Input("hill_duration_slider", "value"),
     Input("hill_angle_slider", "value")]
)
def update_graph(n, reset_clicks, simulation_type, target_speed, kp, ti, td, hill_start_time_slider, hill_duration_slider, hill_angle_slider):
    global previous_graph_data, hill_start_time, hill_duration, hill_angle, hill_slope

    # Update global variables based on slider values
    hill_start_time = hill_start_time_slider
    hill_duration = hill_duration_slider
    hill_slope = hill_angle_slider/100  # Slope of the hill as rise/run
    hill_angle = np.arctan(hill_slope)  # Convert slope to angle in radians

    if ctx.triggered_id == "reset_button":
        previous_graph_data = {
            "speed": [],
            "throttle": [],
            "braking": [],
            "height": []
        }
        speed_fig = go.Figure()
        throttle_fig = go.Figure()
        braking_fig = go.Figure()
        height_fig = go.Figure()
        speed_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Speed [m/s]')
        throttle_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Throttle')
        braking_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Braking Power')
        height_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Height [m]')
        return speed_fig, throttle_fig, braking_fig, height_fig

    if ctx.triggered_id == "simulate_button":
        duration = 100  # duration in seconds

        if simulation_type == 'simulate_cruise_control':
            time, speeds, throttle_values, braking_values, heights = simulate_cruise_control(target_speed, duration, kp, ti, td)
        elif simulation_type == 'simulate_cruise_control_up':
            time, speeds, throttle_values, braking_values, heights = simulate_cruise_control_up(target_speed, duration, kp, ti, td)
        elif simulation_type == 'simulate_cruise_control_down':
            time, speeds, throttle_values, braking_values, heights = simulate_cruise_control_down(target_speed, duration, kp, ti, td)
        elif simulation_type == 'simulate_cruise_control_upandownrandom':
            time, speeds, throttle_values, braking_values, heights = simulate_cruise_control_upandownrandom(target_speed, duration, kp, ti, td)

        # Update previous graph data
        previous_graph_data["speed"].append((time, speeds))
        previous_graph_data["throttle"].append((time, throttle_values))
        previous_graph_data["braking"].append((time, braking_values))
        previous_graph_data["height"].append((time, heights))

        # Create new figures
        speed_fig = go.Figure()
        throttle_fig = go.Figure()
        braking_fig = go.Figure()
        height_fig = go.Figure()

        # Add previous data to figures
        for data in previous_graph_data["speed"]:
            speed_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["throttle"]:
            throttle_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["braking"]:
            braking_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["height"]:
            height_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))

        # Add new data to figures with a new color
        color = f"rgb({np.random.randint(0, 256)}, {np.random.randint(0, 256)}, {np.random.randint(0, 256)})"
        speed_fig.add_trace(go.Scatter(x=time, y=speeds, mode='lines', name='Speed', line=dict(color=color)))
        speed_fig.add_trace(go.Scatter(x=time, y=[target_speed] * len(time), mode='lines', name='Target Speed', line=dict(dash='dash', color='black')))
        speed_fig.update_layout(title='Speed vs Time', xaxis_title='Time [s]', yaxis_title='Speed [m/s]')

        throttle_fig.add_trace(go.Scatter(x=time, y=throttle_values, mode='lines', name='Throttle', line=dict(color=color)))
        throttle_fig.update_layout(title='Throttle vs Time', xaxis_title='Time [s]', yaxis_title='Throttle')

        braking_fig.add_trace(go.Scatter(x=time, y=braking_values, mode='lines', name='Braking Power', line=dict(color=color)))
        braking_fig.update_layout(title='Braking Power vs Time', xaxis_title='Time [s]', yaxis_title='Braking Power')

        height_fig.add_trace(go.Scatter(x=time, y=heights, mode='lines', name='Height', line=dict(color=color)))
        height_fig.update_layout(title='Height vs Time', xaxis_title='Time [s]', yaxis_title='Height [m]')

        return speed_fig, throttle_fig, braking_fig, height_fig

    # Ensure the function always returns a tuple or list of values
    return go.Figure(), go.Figure(), go.Figure(), go.Figure()

if __name__ == '__main__':
    app.run_server(debug=True)
