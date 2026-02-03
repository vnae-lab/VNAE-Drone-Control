import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class VNAEDroneController:
    """
    VNAE Control System for a Swarm of Drones.
    Implements asymmetric equilibrium for multi-agent coordination.
    """
    def __init__(self, n_drones=10):
        self.n_drones = n_drones
        
        # --- USER-DEFINED PARAMETERS ---
        # Asymmetry parameters (theta_i)
        self.theta = np.array([1.2, 1.8, 1.5, 1.0, 2.9, 1.3, 2.1, 3.7, 1.4, 1.0])
        
        # Strategic weights (omega_i)
        self.omega = np.ones(n_drones)
        
        # Global rigidity parameter (beta)
        self.beta = 0.1
        
        # --- SYSTEM SETUP ---
        np.random.seed(42)
        # Coupling matrix (A) - Symmetrized
        A_raw = np.random.normal(0, 1, (n_drones, n_drones))
        self.A = (A_raw + A_raw.T) / 2
        np.fill_diagonal(self.A, 0)
        
        # Initial positions (random in 3D)
        self.positions = np.random.normal(0, 10, (n_drones, 3))
        
        # Target formation (desired coordinates)
        self.targets = np.array([
            [0, 0, 10], [5, 5, 10], [5, -5, 10], [-5, 5, 10], [-5, -5, 10],
            [10, 0, 10], [0, 10, 10], [-10, 0, 10], [0, -10, 10], [0, 0, 15]
        ])
        
        # Metrics History
        self.curvature_history = []
        self.positions_history = [self.positions.copy()]

    def compute_vnae_equilibrium(self):
        """Computes geometric stability and manifold properties."""
        M = self.A + np.diag(self.theta)
        
        # Manifold dimension using SVD
        U, S, Vh = np.linalg.svd(M)
        rank = np.sum(S > 1e-10)
        manifold_dim = self.n_drones - rank
        
        # Simplified Scalar Curvature K
        K = 0.0
        for i in range(self.n_drones):
            for j in range(i + 1, self.n_drones):
                diff_theta = abs(self.theta[i] - self.theta[j])
                coupling = abs(self.A[i, j])
                rigidity = 1 + self.beta * (self.theta[i] + self.theta[j])
                K += diff_theta * coupling / rigidity
        
        K = (2 * K) / (self.n_drones * (self.n_drones - 1))
        return {"curvature": K, "stable": K > 0, "manifold_dim": manifold_dim}

    def vnae_control_law(self, dt=0.1):
        """VNAE-based control law: s_dot = -[diag(w)*grad_V + diag(theta)*s] + coupling"""
        # Potential gradient (distance to target)
        grad_V = self.positions - self.targets
        control_force = np.zeros((self.n_drones, 3))
        
        for i in range(self.n_drones):
            # Individual VNAE dynamics
            vnae_force = -self.omega[i] * grad_V[i, :] - self.theta[i] * (self.positions[i, :] - self.targets[i, :])
            
            # Coordination coupling from other drones
            coupling_force = np.zeros(3)
            for j in range(self.n_drones):
                if i != j:
                    coupling_force += self.A[i, j] * (self.positions[j, :] - self.positions[i, :])
            
            control_force[i, :] = vnae_force + self.beta * coupling_force
            
        # Update positions
        self.positions = self.positions + control_force * dt
        
        # Store metrics
        results = self.compute_vnae_equilibrium()
        self.curvature_history.append(results["curvature"])
        self.positions_history.append(self.positions.copy())

    def simulate(self, n_steps=100, dt=0.1):
        print("="*40)
        print("VNAE DRONE CONTROL SIMULATION")
        print("="*40)
        
        for step in range(n_steps + 1):
            if step > 0:
                self.vnae_control_law(dt)
            
            if step % 20 == 0:
                res = self.compute_vnae_equilibrium()
                print(f"Step {step}: K = {res['curvature']:.4f}, Stable = {res['stable']}")

    def visualize(self):
        # Prepare data
        pos_hist = np.array(self.positions_history)
        
        fig = plt.figure(figsize=(15, 10))
        
        # Plot 1: 3D Trajectories
        ax1 = fig.add_subplot(2, 2, 1, projection='3d')
        for i in [0, 9]: # Plotting drone 1 and 10 for clarity
            ax1.plot(pos_hist[:, i, 0], pos_hist[:, i, 1], pos_hist[:, i, 2], label=f'Drone {i+1}')
        ax1.scatter(self.targets[:, 0], self.targets[:, 1], self.targets[:, 2], color='black', marker='x', label='Targets')
        ax1.set_title("Drone Trajectories (3D)")
        ax1.legend()

        # Plot 2: Curvature Evolution
        ax2 = fig.add_subplot(2, 2, 2)
        ax2.plot(self.curvature_history, color='darkblue')
        ax2.axhline(0, color='red', linestyle='--', alpha=0.5)
        ax2.set_title("VNAE Curvature Evolution")
        ax2.set_xlabel("Time Step")
        ax2.set_ylabel("Scalar Curvature K")

        # Plot 3: Parameters
        ax3 = fig.add_subplot(2, 2, 3)
        indices = np.arange(self.n_drones)
        ax3.bar(indices - 0.2, self.theta, width=0.4, label='θ (asymmetry)', color='steelblue')
        ax3.bar(indices + 0.2, self.omega, width=0.4, label='ω (weight)', color='orange')
        ax3.set_title("VNAE Parameters per Drone")
        ax3.legend()

        # Plot 4: XY Plane Final Positions
        ax4 = fig.add_subplot(2, 2, 4)
        ax4.scatter(self.targets[:, 0], self.targets[:, 1], color='red', marker='x', s=100, label='Targets')
        ax4.scatter(self.positions[:, 0], self.positions[:, 1], color='blue', alpha=0.6, label='Final Pos')
        ax4.set_title("Final Positions (XY Plane)")
        ax4.legend()

        plt.tight_layout()
        plt.show()

# Execution
controller = VNAEDroneController(n_drones=10)
controller.simulate(n_steps=100)
controller.visualize()
