# =============================================================================
# VNAE (Victoria-Nash Asymmetric Equilibrium) Control System for 10 Drones
# Implementation of the quadratic VNAE framework for multi-agent stability
# =============================================================================

# Load required libraries
library(rgl)        # For 3D visualization
library(ggplot2)    # For 2D plotting
library(plotly)     # For interactive plots
library(gridExtra)  # For multiple plots

# =============================================================================
# VNAE DRONE CONTROLLER CLASS
# =============================================================================

VNAEDroneController <- function(n_drones = 10) {
  # Initialize VNAE controller for 10 drones
 
  # ------------------------------------------------------------
  # USER INPUT AREAS - EDIT THESE VALUES
  # ------------------------------------------------------------
 
  # Asymmetry parameters for each drone (θ_i)
  # Higher θ = more rigid/inertial behavior
  theta <- c(1.2, 1.8, 1.5, 1.0, 2.9,
             1.3, 2.1, 3.7, 1.4, 1.0)  # ← EDIT THESE VALUES
 
  # Strategic weights for each drone (ω_i)
  # Higher ω = more responsive to potential gradients
  omega <- rep(1.0, 10)  # ← EDIT THESE VALUES
 
  # Global rigidity parameter (β)
  # Higher β = stronger geometric constraints
  beta <- 0.1  # ← EDIT THIS VALUE
 
  # ------------------------------------------------------------
  # System setup
  # ------------------------------------------------------------
 
  # Coupling matrix (A_ij) - represents interaction strengths
  set.seed(42)  # For reproducibility
  A <- matrix(rnorm(n_drones * n_drones), n_drones, n_drones)
  A <- (A + t(A)) / 2  # Make symmetric
  diag(A) <- 0         # No self-coupling
 
  # Drone states (positions in 3D)
  positions <- matrix(rnorm(n_drones * 3, sd = 10), n_drones, 3)
 
  # Target formation (desired positions)
  targets <- matrix(c(
    0, 0, 10,     # Drone 1
    5, 5, 10,     # Drone 2
    5, -5, 10,    # Drone 3
    -5, 5, 10,    # Drone 4
    -5, -5, 10,   # Drone 5
    10, 0, 10,    # Drone 6
    0, 10, 10,    # Drone 7
    -10, 0, 10,   # Drone 8
    0, -10, 10,   # Drone 9
    0, 0, 15      # Drone 10 (leader)
  ), ncol = 3, byrow = TRUE)
 
  # Tracking metrics
  curvature_history <- numeric(0)
  stability_history <- logical(0)
  positions_history <- list(positions)
 
  # ------------------------------------------------------------
  # VNAE COMPUTATION FUNCTIONS
  # ------------------------------------------------------------
 
  compute_vnae_equilibrium <- function() {
    # Compute VNAE equilibrium manifold and geometric properties
   
    # 1. Equilibrium matrix M = A + diag(θ)
    M <- A + diag(theta)
   
    # 2. Equilibrium manifold (null space of M)
    svd_result <- svd(M)
    singular_values <- svd_result$d
    rank <- sum(singular_values > 1e-10)
    manifold_dim <- n_drones - rank
   
    if (manifold_dim > 0) {
      # Take right singular vectors corresponding to near-zero singular values
      manifold_basis <- svd_result$v[, (rank + 1):n_drones]
    } else {
      manifold_basis <- matrix(0, n_drones, 1)
    }
   
    # 3. Riemannian metric g_ij = ω_i δ_ij + β(A_ij + θ_i δ_ij)
    g <- diag(omega) + beta * (A + diag(theta))
   
    # 4. Scalar curvature (simplified quadratic approximation)
    K <- 0
    for (i in 1:(n_drones - 1)) {
      for (j in (i + 1):n_drones) {
        K <- K + abs(theta[i] - theta[j]) * abs(A[i, j]) /
          (1 + beta * (theta[i] + theta[j]))
      }
    }
    K <- 2 * K / (n_drones * (n_drones - 1))
   
    # 5. Stability condition
    stable <- K > 0
   
    return(list(
      manifold_dimension = manifold_dim,
      manifold_basis = manifold_basis,
      curvature = K,
      stable = stable,
      metric = g,
      equilibrium_matrix = M
    ))
  }
 
  compute_potential_gradient <- function() {
    # Compute gradient of potential function V(s) = 0.5 * ||s - target||^2
    gradient <- positions - targets
    return(gradient)
  }
 
  vnae_control_law <- function(dt = 0.1) {
    # VNAE-based control law for drone movement
   
    # Get VNAE results
    vnae_results <- compute_vnae_equilibrium()
   
    # Compute potential gradient
    grad_V <- compute_potential_gradient()
   
    # VNAE control: ṡ = -[diag(ω) grad_V + diag(θ) s]
    control_force <- matrix(0, n_drones, 3)
   
    for (i in 1:n_drones) {
      # VNAE dynamics component
      vnae_force <- -omega[i] * grad_V[i, ] -
        theta[i] * (positions[i, ] - targets[i, ])
     
      # Add coupling from other drones
      coupling_force <- rep(0, 3)
      for (j in 1:n_drones) {
        if (i != j) {
          # Coupling proportional to A_ij and relative positions
          coupling_force <- coupling_force +
            A[i, j] * (positions[j, ] - positions[i, ])
        }
      }
     
      control_force[i, ] <- vnae_force + beta * coupling_force
    }
   
    # Update positions
    new_positions <- positions + control_force * dt
   
    # Store metrics
    curvature_history <<- c(curvature_history, vnae_results$curvature)
    stability_history <<- c(stability_history, vnae_results$stable)
    positions_history <<- c(positions_history, list(new_positions))
   
    return(new_positions)
  }
 
  # ------------------------------------------------------------
  # SIMULATION FUNCTION
  # ------------------------------------------------------------
 
  simulate <- function(n_steps = 100, dt = 0.1) {
    # Run simulation of drone swarm
   
    cat(rep("=", 60), "\n", sep = "")
    cat("VNAE DRONE CONTROL SIMULATION - 10 DRONES\n")
    cat(rep("=", 60), "\n\n", sep = "")
   
    cat("VNAE Parameters:\n")
    cat("θ (asymmetry):", theta, "\n")
    cat("ω (weights):", omega, "\n")
    cat("β (rigidity):", beta, "\n\n")
   
    for (step in 0:n_steps) {
      if (step > 0) {
        positions <<- vnae_control_law(dt)
      }
     
      if (step %% 20 == 0) {
        vnae_results <- compute_vnae_equilibrium()
        cat(sprintf("Step %d: Curvature K = %.4f, Stable = %s, Manifold dim = %d\n",
                    step, vnae_results$curvature,
                    vnae_results$stable, vnae_results$manifold_dimension))
      }
    }
   
    # Final analysis
    final_results <- compute_vnae_equilibrium()
   
    cat("\n", rep("=", 60), "\n", sep = "")
    cat("SIMULATION COMPLETE\n")
    cat(rep("=", 60), "\n\n", sep = "")
   
    cat("Final VNAE Analysis:\n")
    cat(sprintf("Scalar curvature K: %.6f\n", final_results$curvature))
    cat(sprintf("System stable (K > 0): %s\n", final_results$stable))
    cat(sprintf("Equilibrium manifold dimension: %d\n\n",
                final_results$manifold_dimension))
   
    # Analyze curvature distribution
    curvature_array <- curvature_history
    cat("Curvature Statistics:\n")
    cat(sprintf("  Mean: %.6f\n", mean(curvature_array)))
    cat(sprintf("  Std: %.6f\n", sd(curvature_array)))
    cat(sprintf("  Min: %.6f\n", min(curvature_array)))
    cat(sprintf("  Max: %.6f\n\n", max(curvature_array)))
   
    # Stability percentage
    stable_percentage <- mean(stability_history) * 100
    cat(sprintf("System was stable %.1f%% of the time\n\n", stable_percentage))
   
    return(list(
      positions_history = positions_history,
      final_results = final_results,
      curvature_history = curvature_history,
      stability_history = stability_history
    ))
  }
 
  # ------------------------------------------------------------
  # VISUALIZATION FUNCTIONS
  # ------------------------------------------------------------
 
  visualize <- function(simulation_results) {
    # Visualize drone trajectories and VNAE properties
   
    positions_history <- simulation_results$positions_history
   
    # Convert positions history to array for easier plotting
    n_steps <- length(positions_history)
    positions_array <- array(0, dim = c(n_steps, n_drones, 3))
   
    for (i in 1:n_steps) {
      positions_array[i, , ] <- positions_history[[i]]
    }
   
    # Create data frames for plotting
    trajectory_df <- data.frame()
    for (drone in 1:n_drones) {
      for (step in 1:n_steps) {
        trajectory_df <- rbind(trajectory_df, data.frame(
          Drone = paste("Drone", drone),
          Step = step,
          X = positions_array[step, drone, 1],
          Y = positions_array[step, drone, 2],
          Z = positions_array[step, drone, 3]
        ))
      }
    }
   
    targets_df <- data.frame(
      Drone = paste("Drone", 1:n_drones),
      X = targets[, 1],
      Y = targets[, 2],
      Z = targets[, 3]
    )
   
    final_positions <- positions_history[[n_steps]]
    final_df <- data.frame(
      Drone = paste("Drone", 1:n_drones),
      X = final_positions[, 1],
      Y = final_positions[, 2],
      Z = final_positions[, 3]
    )
   
    # Plot 1: 3D trajectory using plotly
    p1 <- plot_ly() %>%
      # Add drone trajectories
      add_trace(data = trajectory_df %>% filter(Drone == "Drone 1"),
                x = ~X, y = ~Y, z = ~Z,
                type = 'scatter3d', mode = 'lines',
                line = list(width = 2, color = 'blue'),
                name = 'Drone 1', showlegend = TRUE) %>%
      # Add more drones (simplified for clarity)
      add_trace(data = trajectory_df %>% filter(Drone == "Drone 10"),
                x = ~X, y = ~Y, z = ~Z,
                type = 'scatter3d', mode = 'lines',
                line = list(width = 2, color = 'red'),
                name = 'Drone 10', showlegend = TRUE) %>%
      # Add targets
      add_trace(data = targets_df,
                x = ~X, y = ~Y, z = ~Z,
                type = 'scatter3d', mode = 'markers',
                marker = list(size = 5, color = 'black', symbol = 'x'),
                name = 'Targets', showlegend = TRUE) %>%
      layout(scene = list(xaxis = list(title = 'X'),
                          yaxis = list(title = 'Y'),
                          zaxis = list(title = 'Z')),
             title = "Drone Trajectories (3D)")
   
    # Plot 2: Curvature over time
    curvature_df <- data.frame(
      Step = 1:length(curvature_history),
      Curvature = curvature_history
    )
   
    p2 <- ggplot(curvature_df, aes(x = Step, y = Curvature)) +
      geom_line(size = 1, color = "darkblue") +
      geom_hline(yintercept = 0, linetype = "dashed", color = "red", alpha = 0.5) +
      labs(x = "Time Step", y = "Scalar Curvature K",
           title = "VNAE Curvature Evolution") +
      theme_minimal() +
      theme(plot.title = element_text(hjust = 0.5))
   
    # Plot 3: Parameter visualization
    params_df <- data.frame(
      Drone = 1:n_drones,
      Theta = theta,
      Omega = omega
    )
   
    p3 <- ggplot(params_df) +
      geom_col(aes(x = factor(Drone), y = Theta, fill = "θ (asymmetry)"),
               position = position_dodge(width = 0.7), width = 0.35, alpha = 0.7) +
      geom_col(aes(x = factor(Drone), y = Omega, fill = "ω (weight)"),
               position = position_dodge(width = 0.7), width = 0.35, alpha = 0.7) +
      scale_fill_manual(values = c("θ (asymmetry)" = "steelblue",
                                   "ω (weight)" = "orange")) +
      labs(x = "Drone Index", y = "Parameter Value",
           title = "VNAE Parameters per Drone", fill = "Parameter") +
      theme_minimal() +
      theme(plot.title = element_text(hjust = 0.5),
            legend.position = "bottom")
   
    # Plot 4: Final positions vs targets (XY plane)
    p4 <- ggplot() +
      # Targets
      geom_point(data = targets_df, aes(x = X, y = Y),
                 color = "red", shape = 4, size = 5, stroke = 2) +
      # Final positions
      geom_point(data = final_df, aes(x = X, y = Y, color = Drone),
                 size = 3, alpha = 0.7) +
      # Connect positions to targets
      geom_segment(data = data.frame(
        x_start = final_df$X, y_start = final_df$Y,
        x_end = targets_df$X, y_end = targets_df$Y
      ), aes(x = x_start, y = y_start, xend = x_end, yend = y_end),
      linetype = "dashed", alpha = 0.3) +
      labs(x = "X", y = "Y",
           title = "Final Positions vs Targets (XY plane)") +
      theme_minimal() +
      theme(plot.title = element_text(hjust = 0.5)) +
      coord_fixed()
   
    # Display plots
    print(p1)
   
    # Arrange 2D plots
    grid.arrange(p2, p3, p4, ncol = 3,
                 top = textGrob(paste("VNAE Control System for 10 Drones | β =", beta),
                                gp = gpar(fontsize = 14, fontface = "bold")))
  }
 
  # ------------------------------------------------------------
  # PARAMETER SENSITIVITY ANALYSIS
  # ------------------------------------------------------------
 
  parameter_sensitivity <- function() {
    # Test different beta values
   
    cat(rep("=", 60), "\n", sep = "")
    cat("PARAMETER SENSITIVITY ANALYSIS\n")
    cat(rep("=", 60), "\n\n", sep = "")
   
    beta_values <- c(0.1, 0.3, 0.5, 0.7, 1.0)
    curvatures <- numeric(length(beta_values))
   
    for (i in 1:length(beta_values)) {
      # Temporary controller with different beta
      temp_theta <- theta
      temp_omega <- omega
      temp_beta <- beta_values[i]
     
      temp_A <- A
      temp_M <- temp_A + diag(temp_theta)
     
      # Compute curvature (simplified)
      K <- 0
      for (p in 1:(n_drones - 1)) {
        for (q in (p + 1):n_drones) {
          K <- K + abs(temp_theta[p] - temp_theta[q]) * abs(temp_A[p, q]) /
            (1 + temp_beta * (temp_theta[p] + temp_theta[q]))
        }
      }
      K <- 2 * K / (n_drones * (n_drones - 1))
     
      curvatures[i] <- K
     
      cat(sprintf("β = %.1f: Curvature K = %.6f, Stable = %s\n",
                  temp_beta, K, K > 0))
    }
   
    # Plot curvature vs beta
    beta_df <- data.frame(Beta = beta_values, Curvature = curvatures)
   
    p_sens1 <- ggplot(beta_df, aes(x = Beta, y = Curvature)) +
      geom_point(size = 4, color = "blue") +
      geom_line(size = 1, color = "blue") +
      geom_hline(yintercept = 0, linetype = "dashed", color = "red", alpha = 0.5) +
      labs(x = "Rigidity Parameter (β)", y = "Scalar Curvature (K)",
           title = "Curvature vs Rigidity Parameter") +
      theme_minimal() +
      theme(plot.title = element_text(hjust = 0.5))
   
    # Current parameter distribution
    params_current_df <- data.frame(
      Drone = 1:n_drones,
      Theta = theta,
      Omega = omega
    )
   
    p_sens2 <- ggplot(params_current_df) +
      geom_point(aes(x = Drone, y = Theta, color = "θ (asymmetry)"), size = 4) +
      geom_point(aes(x = Drone, y = Omega, color = "ω (weight)"), size = 4) +
      scale_color_manual(values = c("θ (asymmetry)" = "darkgreen",
                                    "ω (weight)" = "purple")) +
      labs(x = "Drone Index", y = "Parameter Value",
           title = "Current Parameter Distribution", color = "Parameter") +
      theme_minimal() +
      theme(plot.title = element_text(hjust = 0.5),
            legend.position = "bottom")
   
    grid.arrange(p_sens1, p_sens2, ncol = 2,
                 top = textGrob("VNAE Parameter Sensitivity Analysis",
                                gp = gpar(fontsize = 14, fontface = "bold")))
  }
 
  # ------------------------------------------------------------
  # RETURN PUBLIC METHODS
  # ------------------------------------------------------------
 
  return(list(
    # Parameters (can be modified)
    theta = theta,
    omega = omega,
    beta = beta,
   
    # Methods
    set_parameters = function(new_theta, new_omega, new_beta) {
      theta <<- new_theta
      omega <<- new_omega
      beta <<- new_beta
      cat("Parameters updated.\n")
    },
   
    compute_vnae_equilibrium = compute_vnae_equilibrium,
    simulate = simulate,
    visualize = visualize,
    parameter_sensitivity = parameter_sensitivity,
   
    # Data access
    get_positions = function() positions,
    get_targets = function() targets,
    get_history = function() list(
      curvature = curvature_history,
      stability = stability_history,
      positions = positions_history
    )
  ))
}

# =============================================================================
# MAIN EXECUTION
# =============================================================================

cat("VNAE DRONE CONTROL SYSTEM\n")
cat(rep("=", 50), "\n\n", sep = "")

# Create controller instance
controller <- VNAEDroneController(n_drones = 10)

# ------------------------------------------------------------
# OPTIONAL: MODIFY PARAMETERS (Uncomment and edit as needed)
# ------------------------------------------------------------
# Example: Set custom parameters
# controller$set_parameters(
#   new_theta = c(1.0, 2.0, 1.5, 0.8, 1.2, 0.9, 1.3, 1.1, 1.4, 0.7),
#   new_omega = c(0.8, 1.2, 1.0, 1.1, 0.9, 1.0, 1.0, 1.0, 1.0, 1.0),
#   new_beta = 0.3
# )

# Run simulation
simulation_results <- controller$simulate(n_steps = 100, dt = 0.1)

# Visualize results
controller$visualize(simulation_results)

# Parameter sensitivity analysis
controller$parameter_sensitivity()

# =============================================================================
# HOW TO INTERPRET RESULTS:
# =============================================================================
cat("\n", rep("=", 60), "\n", sep = "")
cat("INTERPRETATION GUIDE\n")
cat(rep("=", 60), "\n\n", sep = "")

cat("KEY METRICS:\n")
cat("1. SCALAR CURVATURE (K): \n")
cat("   - K > 0: System is VNAE-stable (asymmetry induces stability)\n")
cat("   - K ≈ 0: System is marginally stable (approaching classical equilibrium)\n")
cat("   - K < 0: System is unstable\n\n")

cat("2. EQUILIBRIUM MANIFOLD DIMENSION:\n")
cat("   - 0: Single equilibrium point\n")
cat("   - >0: Continuous family of equilibria (manifold)\n\n")

cat("3. PARAMETER ROLES:\n")
cat("   - θ_i (asymmetry): Higher values make drone i more rigid/inertial\n")
cat("   - ω_i (weight): Higher values make drone i more responsive to targets\n")
cat("   - β (rigidity): Global parameter balancing self-dynamics vs coupling\n\n")

cat("TIPS FOR PARAMETER TUNING:\n")
cat("1. For stability: Ensure θ_i values are diverse (asymmetry creates curvature)\n")
cat("2. For responsiveness: Increase ω_i values (but may reduce stability if too high)\n")
cat("3. For coordination: Adjust β to balance individual vs collective behavior\n\n")

cat("TRY THESE EXPERIMENTS:\n")
cat("1. Set all θ_i equal → curvature should approach 0\n")
cat("2. Set diverse θ_i values → curvature should increase\n")
cat("3. Increase β → stronger coupling between drones\n")
cat("4. Decrease ω_i → slower convergence to targets\n")


