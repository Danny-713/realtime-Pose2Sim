import time
import opensim as osim

model_path = "/Users/danny/.codex/worktrees/6c16/pose2sim/Pose2Sim/OpenSim_Setup/Model_Pose2Sim_simple.osim"

model = osim.Model(model_path)
model.setUseVisualizer(True)
state = model.initSystem()

viz = model.getVisualizer()
model.realizePosition(state)
viz.show(state)

coord = model.getCoordinateSet().get("knee_angle_r")

for i in range(120):
    angle = 0.005 * i
    coord.setValue(state, angle)
    model.realizePosition(state)
    viz.show(state)
    time.sleep(1 / 30)
