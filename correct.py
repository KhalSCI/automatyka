from dash import Dash, dcc, html, Input, Output, State, ctx
import plotly.graph_objects as go
import numpy as np
import pandas as pd
cars = {
    "Toyota Corolla": {
        "mass": 1375,  # in kg
        "drag_coefficient": 0.29,
        "frontal_area": 2.2,  # in m^2
        "multiplier": 70000,  # rescaled thrust force in N (140,000 * 0.5)
        "braking_power": 9000  # braking force in N
    },
    "Tesla Model 3 Performance": {
        "mass": 1610,  # in kg
        "drag_coefficient": 0.23,
        "frontal_area": 2.3,  # in m^2
        "multiplier": 175000,  # already rescaled
        "braking_power": 12000  # braking force in N
    },
    "Honda Civic": {
        "mass": 1325,  # in kg
        "drag_coefficient": 0.27,
        "frontal_area": 2.1,  # in m^2
        "multiplier": 67500,  # rescaled thrust force in N (135,000 * 0.5)
        "braking_power": 8500  # braking force in N
    },
    "Ford Mustang": {
        "mass": 1680,  # in kg
        "drag_coefficient": 0.38,
        "frontal_area": 2.4,  # in m^2
        "multiplier": 180000,  # rescaled thrust force in N (330,000 * 0.5)
        "braking_power": 15000  # braking force in N
    },
    "BMW 3 Series": {
        "mass": 1545,  # in kg
        "drag_coefficient": 0.26,
        "frontal_area": 2.2,  # in m^2
        "multiplier": 125000,  # rescaled thrust force in N (250,000 * 0.5)
        "braking_power": 10000  # braking force in N
    }
}

mass = 1000 # mass of the car in kg
drag_coefficient = 0.3  # drag coefficient
frontal_area = 2.2  # m^2
multiplier = 15000  # Engine force multiplier
braking_power = 15000  # Braking power in N
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
    "height": [],
    "slope_force": [],
    "drag_force": [],
    "rolling_force": [],
    "engine_force": [],
    "braking_force": []
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
def calculate_slope(speed, slope_angle):
    # Gravitational force due to slope
    slope_force = mass * gravity * np.sin(slope_angle)
    return slope_force
def calculate_drag(speed):
    # Aerodynamic drag force
    drag_force = 0.5 * drag_coefficient * frontal_area * air_density * speed**2
    return drag_force
def calculate_rolling(speed):
    # Rolling resistance force
    rolling_force = rolling_resistance * mass * gravity
    return rolling_force


# Simulation
def simulate_cruise_control(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    slope_forces = []
    drag_forces = []
    rolling_forces = []
    engine_forces = []
    braking_forces = []
    throttle = 0  # Initial throttle

    speed = 0  # Initial speed
    height = 0  # Initial height
    for t in time:
        pid_output = pid.control(target_speed, speed)
        if pid_output < 0:
            pid2 = pid_output * -1
            hamulec = max(0, min(pid2 / 100, 1))
            print(hamulec)
        else:
            hamulec = 0
        
        #test = pd.DataFrame({'pid_output': pid_output, 'hamulec': hamulec}, index=[0])
        #test.to_csv('test.csv', mode='a', header=False, index=False)
        
        throttle = max(0, min(pid_output / 100, 1))  # Scale PID output to [0, 1]
        throttle = 0.9 * throttle + 0.1 * (throttles[-1] if throttles else 0)  # Smooth changes
        

        slope_angle = 0  # Flat ground

        # Calculate forces
        engine_force_value = throttle * (multiplier/(1 + speed))  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * braking_power  # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force
        slope_force = calculate_slope(speed, slope_angle)
        drag_force = calculate_drag(speed)
        rolling_force = calculate_rolling(speed)
        
        # Log forces
        slope_forces.append(-slope_force)
        drag_forces.append(-drag_force)
        rolling_forces.append(-rolling_force)
        engine_forces.append(engine_force_value)
        braking_forces.append(-braking_force)

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step
        
        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(hamulec)
        heights.append(height)

    return time, speeds, throttles, brakings, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces

# Similarly update other simulation functions to log the forces

def simulate_cruise_control_up(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    slope_forces = []
    drag_forces = []
    rolling_forces = []
    engine_forces = []
    braking_forces = []
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
        #multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * (multiplier/(1 + speed))  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * braking_power # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force
        slope_force = calculate_slope(speed, slope_angle)
        drag_force = calculate_drag(speed)
        rolling_force = calculate_rolling(speed)
        
        # Log forces
        slope_forces.append(-slope_force)
        drag_forces.append(-drag_force)
        rolling_forces.append(-rolling_force)
        engine_forces.append(engine_force_value)
        braking_forces.append(-braking_force)

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step
        
        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces

def simulate_cruise_control_down(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    slope_forces = []
    drag_forces = []
    rolling_forces = []
    engine_forces = []
    braking_forces = []
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
        #multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * (multiplier/(1 + speed))  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * braking_power # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force
        slope_force = calculate_slope(speed, slope_angle)
        drag_force = calculate_drag(speed)
        rolling_force = calculate_rolling(speed)
        
        # Log forces
        slope_forces.append(-slope_force)
        drag_forces.append(-drag_force)
        rolling_forces.append(-rolling_force)
        engine_forces.append(engine_force_value)
        braking_forces.append(-braking_force)

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step
        
        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces
def simulate_cruise_control_upandownrandom(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power):
    pid = PIDController(kp=kp, ti=ti, td=td)
    time = np.arange(0, duration, time_step)
    speeds = []
    throttles = []
    brakings = []
    heights = []
    slope_forces = []
    drag_forces = []
    rolling_forces = []
    engine_forces = []
    braking_forces = []
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
       # multiplier = 15000 / (1+ speed * 0.2)
        engine_force_value = throttle * (multiplier/(1 + speed))  # Engine force in N
        resistance_force = calculate_forces(speed, slope_angle)
        braking_force = hamulec * braking_power # Braking force in N
        net_force = engine_force_value - resistance_force - braking_force
        slope_force = calculate_slope(speed, slope_angle)
        drag_force = calculate_drag(speed)
        rolling_force = calculate_rolling(speed)
        
        # Log forces
        slope_forces.append(-slope_force)
        drag_forces.append(-drag_force)
        rolling_forces.append(-rolling_force)
        engine_forces.append(engine_force_value)
        braking_forces.append(-braking_force)

        # Update speed
        acceleration = net_force / mass
        speed += acceleration * time_step
        
        # Update height
        height_change = speed * time_step * np.sin(slope_angle)
        height += height_change

        # Log values
        speeds.append(speed)
        throttles.append(throttle)
        brakings.append(braking_force)
        heights.append(height)

    return time, speeds, throttles, brakings, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces

# Dash application
app = Dash(__name__)

app.layout = html.Div([
    html.H4('Cruise Control with PID Controller'),

    html.Label("Car Type"),
    dcc.Dropdown(
        id="car_type_dropdown",
        options=[
            {'label': 'Toyota Corolla', 'value': 'Toyota Corolla'},
            {'label': 'Tesla Model 3 Performance', 'value': 'Tesla Model 3 Performance'},
            {'label': 'Honda Civic', 'value': 'Honda Civic'},
            {'label': 'Ford Mustang', 'value': 'Ford Mustang'},
            {'label': 'BMW 3 Series', 'value': 'BMW 3 Series'}
        ],
        value='Toyota Corolla'
    ),

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

    html.Label("Target Speed (km/h)"),
    dcc.Slider(id="target_speed_slider", min=0, max=300, step=5, value=100,
               marks={i: f"{i}" for i in range(0, 301, 5)}),

    html.Label("Kp"),
    dcc.Slider(id="kp_slider", min=0.0, max=50, step=2.0, value=15,
               marks={i: f"{i:.1f}" for i in range(0, 51, 5)}),

    html.Label("TI"),
    dcc.Slider(id="ti_slider", min=0.0, max=100.0, step=5.0, value=55,
               marks={i: f"{i:.1f}" for i in range(0, 101, 5)}),

    html.Label("TD"),
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
    dcc.Graph(id="height_graph"),
    dcc.Graph(id="other_forces_graph"),
    dcc.Graph(id="engine_force_graph"),
    dcc.Graph(id="throttle_graph"),
    dcc.Graph(id="braking_graph"),
    
    
    

    # Div to display the time_step value
    html.Div("Czas próbkowania: 0.05 s", id="time_step_display", style={'position': 'absolute', 'top': '10px', 'right': '10px'})
])

@app.callback(
    [Output("speed_graph", "figure"),
     Output("throttle_graph", "figure"),
     Output("braking_graph", "figure"),
     Output("height_graph", "figure"),
     Output("engine_force_graph", "figure"),
     Output("other_forces_graph", "figure")],
    [Input("simulate_button", "n_clicks"),
     Input("reset_button", "n_clicks"),
     Input("car_type_dropdown", "value"),
     Input("simulation_type_dropdown", "value"),
     Input("target_speed_slider", "value"),
     Input("kp_slider", "value"),
     Input("ti_slider", "value"),
     Input("td_slider", "value"),
     Input("hill_start_time_slider", "value"),
     Input("hill_duration_slider", "value"),
     Input("hill_angle_slider", "value")]
)
def update_graph(n, reset_clicks, car_type, simulation_type, target_speed_kmh, kp, ti, td, hill_start_time_slider, hill_duration_slider, hill_angle_slider):
    global previous_graph_data, hill_start_time, hill_duration, hill_angle, hill_slope

    # Update global variables based on slider values
    hill_start_time = hill_start_time_slider
    hill_duration = hill_duration_slider
    hill_slope = hill_angle_slider / 100  # Slope of the hill as rise/run
    hill_angle = np.arctan(hill_slope)  # Convert slope to angle in radians

    # Convert target speed from km/h to m/s
    target_speed = target_speed_kmh / 3.6

    # Get car parameters based on selected car type
    car_params = cars[car_type]
    mass = car_params["mass"]
    drag_coefficient = car_params["drag_coefficient"]
    frontal_area = car_params["frontal_area"]
    multiplier = car_params["multiplier"]
    braking_power = car_params["braking_power"]

    if ctx.triggered_id == "reset_button":
        previous_graph_data = {
            "speed": [],
            "throttle": [],
            "braking": [],
            "height": [],
            "slope_force": [],
            "drag_force": [],
            "rolling_force": [],
            "engine_force": [],
            "braking_force": []
        }
        speed_fig = go.Figure()
        throttle_fig = go.Figure()
        braking_fig = go.Figure()
        height_fig = go.Figure()
        engine_force_fig = go.Figure()
        other_forces_fig = go.Figure()
        speed_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Speed [km/h]')
        throttle_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Throttle')
        braking_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Braking Power')
        height_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Height [m]')
        engine_force_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Engine Force [N]')
        other_forces_fig.update_layout(title="Reset: No Data", xaxis_title='Time [s]', yaxis_title='Forces [N]')
        return speed_fig, throttle_fig, braking_fig, height_fig, engine_force_fig, other_forces_fig

    if ctx.triggered_id == "simulate_button":
        duration = 100  # duration in seconds

        if simulation_type == 'simulate_cruise_control':
            time, speeds, throttle_values, braking_values, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces = simulate_cruise_control(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power)
        elif simulation_type == 'simulate_cruise_control_up':
            time, speeds, throttle_values, braking_values, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces = simulate_cruise_control_up(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power)
        elif simulation_type == 'simulate_cruise_control_down':
            time, speeds, throttle_values, braking_values, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces = simulate_cruise_control_down(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power)
        elif simulation_type == 'simulate_cruise_control_upandownrandom':
            time, speeds, throttle_values, braking_values, heights, slope_forces, drag_forces, rolling_forces, engine_forces, braking_forces = simulate_cruise_control_upandownrandom(target_speed, duration, kp, ti, td, mass, drag_coefficient, frontal_area, multiplier, braking_power)

        # Convert speeds from m/s to km/h for display
        speeds_kmh = [round(speed * 3.6, 1) for speed in speeds]
        #throttle_values = [round(throttle, 1) for throttle in throttle_values]
        #braking_values = [round(braking, 1) for braking in braking_values]
        #heights = [round(height, 1) for height in heights]
        #slope_forces = [round(force, 1) for force in slope_forces]
        #drag_forces = [round(force, 1) for force in drag_forces]
        #rolling_forces = [round(force, 1) for force in rolling_forces]
        #engine_forces = [round(force, 1) for force in engine_forces]
        #braking_forces = [round(force, 1) for force in braking_forces]

        # Update previous graph data
        previous_graph_data["speed"].append((time, speeds_kmh))
        previous_graph_data["throttle"].append((time, throttle_values))
        previous_graph_data["braking"].append((time, braking_values))
        previous_graph_data["height"].append((time, heights))
        previous_graph_data["slope_force"].append((time, slope_forces))
        previous_graph_data["drag_force"].append((time, drag_forces))
        previous_graph_data["rolling_force"].append((time, rolling_forces))
        previous_graph_data["engine_force"].append((time, engine_forces))
        previous_graph_data["braking_force"].append((time, braking_forces))

        # Create new figures
        speed_fig = go.Figure()
        throttle_fig = go.Figure()
        braking_fig = go.Figure()
        height_fig = go.Figure()
        engine_force_fig = go.Figure()
        other_forces_fig = go.Figure()

        # Add previous data to figures
        for data in previous_graph_data["speed"]:
            speed_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["throttle"]:
            throttle_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["braking"]:
            braking_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["height"]:
            height_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["engine_force"]:
            engine_force_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', name='Engine Force', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["slope_force"]:
            other_forces_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', name='Gravitational force', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["drag_force"]:
            other_forces_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', name='Drag Force', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["rolling_force"]:
            other_forces_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', name='Rolling Force', line=dict(color='gray', dash='dash')))
        for data in previous_graph_data["braking_force"]:
            other_forces_fig.add_trace(go.Scatter(x=data[0], y=data[1], mode='lines', name='Braking Force', line=dict(color='gray', dash='dash')))

        # Add new data to figures with a new color
        colors = ["red", "blue", "green", "orange", "purple", "brown", "gray", "cyan", "magenta"]
        color = colors[len(previous_graph_data["speed"]) % len(colors)]
        speed_fig.add_trace(go.Scatter(x=time, y=speeds_kmh, mode='lines', name='Speed', line=dict(color=color)))
        speed_fig.add_trace(go.Scatter(x=time, y=[target_speed_kmh] * len(time), mode='lines', name='Target Speed', line=dict(dash='dash', color='black')))
        speed_fig.update_layout(title='Speed', xaxis_title='Time [s]', yaxis_title='Speed [km/h]')

        throttle_fig.add_trace(go.Scatter(x=time, y=throttle_values, mode='lines', name='Throttle', line=dict(color=color)))
        throttle_fig.update_layout(title='Throttle', xaxis_title='Time [s]', yaxis_title='Throttle')

        braking_fig.add_trace(go.Scatter(x=time, y=braking_values, mode='lines', name='Brake Force', line=dict(color=color)))
        braking_fig.update_layout(title='Brake Force', xaxis_title='Time [s]', yaxis_title='Brake Force')

        height_fig.add_trace(go.Scatter(x=time, y=heights, mode='lines', name='Height', line=dict(color=color)))
        height_fig.update_layout(title='Height', xaxis_title='Time [s]', yaxis_title='Height [m]')

        engine_force_fig.add_trace(go.Scatter(x=time, y=engine_forces, mode='lines', name='Engine Force', line=dict(color='orange')))
        engine_force_fig.update_layout(title='Engine Force', xaxis_title='Time [s]', yaxis_title='Engine Force [N]')

        other_forces_fig.add_trace(go.Scatter(x=time, y=slope_forces, mode='lines', name='Gravitational Force', line=dict(color='red')))
        other_forces_fig.add_trace(go.Scatter(x=time, y=drag_forces, mode='lines', name='Drag Force', line=dict(color='blue')))
        other_forces_fig.add_trace(go.Scatter(x=time, y=rolling_forces, mode='lines', name='Rolling Force', line=dict(color='green')))
        other_forces_fig.add_trace(go.Scatter(x=time, y=braking_forces, mode='lines', name='Braking Force', line=dict(color='purple')))
        other_forces_fig.update_layout(title='Forces', xaxis_title='Time [s]', yaxis_title='Forces [N]')

        return speed_fig, throttle_fig, braking_fig, height_fig, engine_force_fig, other_forces_fig

    # Ensure the function always returns a tuple or list of values
    return go.Figure(), go.Figure(), go.Figure(), go.Figure(), go.Figure(), go.Figure()

if __name__ == '__main__':
    app.run_server(debug=True)