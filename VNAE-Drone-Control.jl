using Plots
using LinearAlgebra
using Random
using Statistics

# =============================================================================
# VNAE (Victoria-Nash Asymmetric Equilibrium) Drone Controller
# =============================================================================

mutable struct VNAEDroneController
    n_drones::Int
    theta::Vector{Float64}
    omega::Vector{Float64}
    beta::Float64
    A::Matrix{Float64}
    positions::Matrix{Float64}
    targets::Matrix{Float64}
    curvature_history::Vector{Float64}
    positions_history::Vector{Matrix{Float64}}

    function VNAEDroneController(n_drones=10)
        # Asymmetry parameters (theta_i)
        theta = [1.2, 2.8, 1.5, 1.0, 2.9, 1.3, 2.1, 3.7, 1.4, 1.0]
        
        # Strategic weights (omega_i)
        omega = ones(n_drones)
        
        # Global rigidity (beta)
        beta = 0.1
        
        # System setup
        Random.seed!(123)
        A_raw = randn(n_drones, n_drones)
        A = (A_raw + A_raw') / 2
        for i in 1:n_drones A[i, i] = 0 end
        
        # Initial positions
        positions = randn(n_drones, 3) .* 10.0
        
        # Target formation
        targets = [
            0.0 0.0 10.0; 5.0 5.0 10.0; 5.0 -5.0 10.0; -5.0 5.0 10.0; -5.0 -5.0 10.0;
            10.0 0.0 10.0; 0.0 10.0 10.0; -10.0 0.0 10.0; 0.0 -10.0 10.0; 0.0 0.0 15.0
        ]
        
        new(n_drones, theta, omega, beta, A, positions, targets, Float64[], [copy(positions)])
    end
end

# --- VNAE Computation Functions ---

function compute_vnae_equilibrium(c::VNAEDroneController)
    M = c.A + Diagonal(c.theta)
    
    # Manifold dimension using SVD
    S = svdvals(M)
    rank = sum(S .> 1e-10)
    manifold_dim = c.n_drones - rank
    
    # Scalar Curvature K
    K = 0.0
    for i in 1:(c.n_drones - 1)
        for j in (i + 1):c.n_drones
            K += abs(c.theta[i] - c.theta[j]) * abs(c.A[i, j]) / 
                 (1 + c.beta * (c.theta[i] + c.theta[j]))
        end
    end
    K = (2 * K) / (c.n_drones * (c.n_drones - 1))
    
    return (curvature=K, stable=K > 0, manifold_dim=manifold_dim)
end

function vnae_control_law!(c::VNAEDroneController, dt=0.1)
    grad_V = c.positions - c.targets
    control_force = zeros(c.n_drones, 3)
    
    for i in 1:c.n_drones
        # VNAE Dynamics
        vnae_force = -c.omega[i] .* grad_V[i, :] .- c.theta[i] .* (c.positions[i, :] .- c.targets[i, :])
        
        # Coupling
        coupling_force = zeros(3)
        for j in 1:c.n_drones
            if i != j
                coupling_force .+= c.A[i, j] .* (c.positions[j, :] .- c.positions[i, :])
            end
        end
        
        control_force[i, :] = vnae_force .+ c.beta .* coupling_force
    end
    
    c.positions += control_force .* dt
    
    # Store metrics
    res = compute_vnae_equilibrium(c)
    push!(c.curvature_history, res.curvature)
    push!(c.positions_history, copy(c.positions))
end

# --- Simulation and Plotting ---

function run_simulation!(c::VNAEDroneController, n_steps=100)
    println("-"^40)
    println("VNAE DRONE CONTROL")
    println("-"^40)
    
    for step in 0:n_steps
        if step > 0
            vnae_control_law!(c)
        end
        if step % 20 == 0
            res = compute_vnae_equilibrium(c)
            @printf("Step %d: K = %.4f, Stable = %s\n", step, res.curvature, res.stable)
        end
    end
end

using Printf # For formatted printing

# Execution
controller = VNAEDroneController(10)
run_simulation!(controller, 100)

# --- Visualization ---
# 3D Trajectories (Plotting Drone 1 and 10)
pos_hist = controller.positions_history
d1_traj = hcat([pos_hist[t][1, :] for t in 1:length(pos_hist)]...)'
d10_traj = hcat([pos_hist[t][10, :] for t in 1:length(pos_hist)]...)'

p1 = plot3d(d1_traj[:, 1], d1_traj[:, 2], d1_traj[:, 3], label="Drone 1", lw=2)
plot3d!(d10_traj[:, 1], d10_traj[:, 2], d10_traj[:, 3], label="Drone 10", lw=2)
scatter3d!(controller.targets[:, 1], controller.targets[:, 2], controller.targets[:, 3], 
           marker=:x, color=:black, label="Targets", title="3D Trajectories")

# Curvature
p2 = plot(controller.curvature_history, title="VNAE Curvature", color=:darkblue, lw=2, legend=false)
hline!([0], line=:dash, color=:red)

# Combine
plot(p1, p2, layout=(1, 2), size=(900, 450))
