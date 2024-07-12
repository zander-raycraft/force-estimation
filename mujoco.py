import mujoco_py
import matplotlib.pyplot as plt

# Load the model from the XML file
model = mujoco_py.load_model_from_path('/2DOFmodel.xml')
sim = mujoco_py.MjSim(model)
times = []
forces_lbs = []
newton_2_pound = 0.224809

"""
    @Class: CustomView: personalized view for mujoco modeling to allow for differing sensors then default
"""
class CustomView(mujoco_py.MjViewer):
    def __init__(self, sim):
        super().__init__(sim)
        self.joint1_control = 0
        self.joint2_control = 0

    def keyboard(self, key, action, mods):
        if action == mujoco_py.MJ_VIEW_ACT_KEY:
            if key == mujoco_py.GLFW_KEY_LEFT:
                self.joint1_control -= 0.1
            elif key == mujoco_py.GLFW_KEY_RIGHT:
                self.joint1_control += 0.1
            elif key == mujoco_py.GLFW_KEY_A:
                self.joint2_control -= 0.1
            elif key == mujoco_py.GLFW_KEY_D:
                self.joint2_control += 0.1

            sim.data.ctrl[0] = self.joint1_control
            sim.data.ctrl[1] = self.joint2_control

viewer = CustomView(sim)
newton_to_pound = 0.224809


def convert_to_pounds(force_newton):
    return force_newton * newton_2_pound

for i in range(1000):
    sim.step()
    viewer.render()
    time = i * sim.model.opt.timestep
    force_data_newton = sim.data.sensordata  # This is in Newtons
    force_data_pounds = convert_to_pounds(force_data_newton)
    times.append(time)
    forces_lbs.append(force_data_pounds)
    print(f"Time: {time:.2f} s, Force (lbs): {force_data_pounds}")


plt.plot(times, forces_lbs)
plt.xlabel('Time (s)')
plt.ylabel('Force (lbs)')
plt.title('Force Sensor Readings')
plt.show()
