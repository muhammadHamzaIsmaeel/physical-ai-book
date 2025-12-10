# code-examples/sim-to-real/domain_randomization_example.py

import omni.isaac.core.utils.nucleus as nucleus_utils
from omni.isaac.kit import SimulationApp

# Start the simulation app
# Note: Ensure Isaac Sim is installed and configured.
# This script is meant to be run within the Isaac Sim environment or through `isaac-sim.sh python.sh`
simulation_app = SimulationApp({"headless": False})

import omni.isaac.core as ic
from omni.isaac.core.simulation_context import SimulationContext
import random
import numpy as np
import carb

class DomainRandomizationExample:
    def __init__(self):
        self._simulation_context = SimulationContext()
        self._scene = self._simulation_context.scene
        self._world = ic.World(stage_units_in_meters=1.0)
        self._world.scene.add_default_ground_plane()

        self._current_randomization_iteration = 0
        self._max_randomization_iterations = 100 # How many times to randomize and reset

    async def setup_scene(self):
        # Add a simple cube to the scene to observe physics
        self.cube = self._scene.add(
            ic.objects.DynamicCuboid(
                prim_path="/World/Cube",
                name="my_cube",
                position=np.array([0.0, 0.0, 1.0]),
                size=0.2,
                density=1000, # kg/m^3
                mass=1.0,
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        self._scene.add(self.cube)

        # Get the ground plane for friction randomization
        self.ground_prim = self._world.get_ground_plane().prim
        if not self.ground_prim:
            carb.log_error("Could not find ground plane prim.")

        # Get the default light for randomization (if exists)
        self.default_light_prim = self._scene.get_light() # Assumes a default light exists
        if not self.default_light_prim:
            carb.log_warn("No default light found, adding one.")
            # Add a default light if none exists
            self.default_light_prim = self._scene.add(
                ic.objects.SphereLight(
                    prim_path="/World/DefaultSphereLight",
                    name="default_sphere_light",
                    position=np.array([500.0, 200.0, 1000.0]),
                    radius=50.0,
                    intensity=3000000.0,
                    color=np.array([1.0, 1.0, 1.0]),
                )
            )

        await self._world.reset_async()

    def randomize_parameters(self):
        # Randomize ground friction
        static_friction = random.uniform(0.1, 1.0)
        dynamic_friction = random.uniform(0.1, 1.0)
        # Apply physics material properties directly to the ground plane
        if self.ground_prim:
            ic.core.utils.prims.set_prim_collision_properties(
                self.ground_prim,
                static_friction=static_friction,
                dynamic_friction=dynamic_friction,
                restitution=0.01 # Keep restitution low for stability
            )
            print(f"Randomized ground friction: Static={static_friction:.2f}, Dynamic={dynamic_friction:.2f}")

        # Randomize light intensity and color
        if self.default_light_prim:
            intensity = random.uniform(1_000_000.0, 5_000_000.0)
            color = np.array([random.uniform(0.5, 1.0), random.uniform(0.5, 1.0), random.uniform(0.5, 1.0)])
            self.default_light_prim.set_attribute("intensity", intensity)
            self.default_light_prim.set_attribute("color", carb.Float3(color[0], color[1], color[2]))
            print(f"Randomized light: Intensity={intensity:.2e}, Color={color}")

        # Randomize cube's mass
        new_mass = random.uniform(0.5, 2.0) # kg
        self.cube.set_mass(new_mass)
        self.cube.set_color(np.array([random.uniform(0.1, 1.0), random.uniform(0.1, 1.0), random.uniform(0.1, 1.0)]))
        print(f"Randomized cube: Mass={new_mass:.2f} kg, Color={self.cube.get_color()}")

    async def run_randomized_sim(self):
        await self._world.reset_async()
        self.randomize_parameters()
        print(f"--- Running iteration {self._current_randomization_iteration + 1} ---")
        # Simulate for a few steps to observe the effect
        for _ in range(200): # Simulate for ~3 seconds at 60 FPS
            await self._simulation_context.render_async()
            await self._simulation_context.step_async()
            if not self._simulation_context.is_playing():
                break

    def start_simulation(self):
        self._world.run_on_startup()
        self._simulation_context.play()

        while self._simulation_context.is_playing() and self._current_randomization_iteration < self._max_randomization_iterations:
            self.start_simulation_loop()
            self._current_randomization_iteration += 1

        self._simulation_context.stop()
        simulation_app.close()

    def start_simulation_loop(self):
        async def _loop():
            await self.run_randomized_sim()
            # After each randomized run, reset the cube's position to observe a fresh drop
            self.cube.set_world_pose(position=np.array([0.0, 0.0, 1.0]))
            await self._simulation_context.step_async()
            await self._simulation_context.render_async()

        self._simulation_context.app.update()
        self._simulation_context.app.add_update_callback(_loop)


async def main():
    dr_example = DomainRandomizationExample()
    await dr_example.setup_scene()
    dr_example.start_simulation()

if __name__ == "__main__":
    import asyncio
    asyncio.run(main())

