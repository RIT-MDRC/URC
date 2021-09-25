import klampt
from klampt.model import ik
world = klampt.WorldModel()
world.loadElement("data/robots/planar3R.rob")

robot= world.robot(0)
link = robot.link(2)
obj = ik.objective(link,local=[1,0,0],world=[1.5,0,1])
solver = ik.solver(obj)
solver.setJointLimits(*robot.getJointLimits())  #the values [],[] tell the solver to turn off joint limits
robot.setConfig([0,0,0])
print(solver.solve())
print(robot.getConfig())
print(solver.getResidual())

solver.sampleInitial()   # the initial configuration didn't let the solver find a solution, sample a new one
print(solver.solve())
print(robot.getConfig())
print(solver.getResidual())

input()

from klampt import vis
vis.add("world",world)    #shows the robot in the solved configuration
vis.add("local point",link.getWorldPosition([1,0,0]))
vis.setAttribute("local point","type","Vector3")  #usually the vis module identifies a Config vs a Vector3, but this robot has exactly 3 links
vis.add("target point",[1.5,0,1])
vis.setAttribute("target point","type","Vector3")
vis.setColor("target point",1,0,0)  #turns the target point red
vis.show()  #this will pop up the visualization window until you close it

input()