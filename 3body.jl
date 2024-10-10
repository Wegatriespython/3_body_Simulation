using DifferentialEquations
using GLMakie
using LinearAlgebra  # Import norm function

# Constants
const G = 6.67430e-11  # Gravitational constant

# Acceleration function
function acceleration_on_body(r_body, r_other, m_other)
    r = r_other - r_body
    r_norm = norm(r) + 1e-10  # Avoid division by zero
    return G * m_other * r / r_norm^3
end

# Equations of motion
function three_body!(du, u, p, t)
    m1, m2, m3 = p
    r1, r2, r3 = u[1:2], u[3:4], u[5:6]
    v1, v2, v3 = u[7:8], u[9:10], u[11:12]

    # Update positions
    du[1:2] = v1
    du[3:4] = v2
    du[5:6] = v3

    # Compute accelerations
    du[7:8] = acceleration_on_body(r1, r2, m2) + acceleration_on_body(r1, r3, m3)
    du[9:10] = acceleration_on_body(r2, r1, m1) + acceleration_on_body(r2, r3, m3)
    du[11:12] = acceleration_on_body(r3, r1, m1) + acceleration_on_body(r3, r2, m2)
end

# Function to collect user input
function get_float(prompt::String, default::Float64)
    print(prompt, " [Default: ", default, "]: ")
    input = readline()
    return isempty(input) ? default : parse(Float64, input)
end

# Initialize simulation parameters
function initialize_simulation()
    println("=== Three-Body Problem Simulation Setup ===\n")

    u0 = zeros(12)
    for i in 1:3
        println("\n-- Body $i --")
        u0[2i-1] = get_float("Enter initial x-position", i == 1 ? 1.0 : i == 2 ? -1.0 : 0.0)
        u0[2i] = get_float("Enter initial y-position", 0.0)
        u0[2i+5] = get_float("Enter initial x-velocity", 0.0)
        u0[2i+6] = get_float("Enter initial y-velocity", i == 1 ? 1.0 : i == 2 ? -1.0 : 0.0)
    end

    masses = [get_float("\nEnter mass for Body $i (kg)", 1.0) for i in 1:3]
    p = Tuple(masses)

    tspan = (0.0, get_float("\nEnter simulation duration (seconds)", 10.0))

    return u0, p, tspan
end

# Main execution
u0, p, tspan = initialize_simulation()
prob = ODEProblem(three_body!, u0, tspan, p)
sol = solve(prob, Tsit5(), saveat=0.01)

# Set up the GLMakie plot
fig = Figure(resolution = (800, 600))
ax = Axis(fig[1, 1], aspect = DataAspect(), xlabel = "x", ylabel = "y", 
          title = "Three-Body Problem Simulation")

colors = [:red, :green, :blue]
labels = ["Body 1", "Body 2", "Body 3"]

# Initialize plots with initial positions
line_plots = []
point_plots = []

for i in 1:3
    xi = [u0[2i-1]]
    yi = [u0[2i]]
    push!(line_plots, lines!(ax, xi, yi; color=colors[i], label=labels[i]))
    push!(point_plots, scatter!(ax, xi, yi; color=colors[i], markersize=10))
end

# Set up legend
axislegend(ax)

# Animation loop
record(fig, "three_body_simulation.mp4", 1:length(sol.t); framerate=30) do frame
    for i in 1:3
        xi = [u[2i-1] for u in sol.u[1:frame]]
        yi = [u[2i] for u in sol.u[1:frame]]

        # Update line plot
        line_plots[i].visible = false  # Hide the previous line
        line_plots[i] = lines!(ax, xi, yi; color=colors[i], label=labels[i])

        # Update scatter plot
        point_plots[i].visible = false  # Hide the previous point
        point_plots[i] = scatter!(ax, [xi[end]], [yi[end]]; color=colors[i], markersize=10)
    end

    # Update axis limits
    autolimits!(ax)
end

println("Animation complete. The result has been saved as 'three_body_simulation.mp4'.")
