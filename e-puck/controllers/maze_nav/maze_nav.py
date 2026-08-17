from controller import Robot


class MazeNavigator(Robot):
    def __init__(self):
        super().__init__()
        self.timestep = int(self.getBasicTimeStep())

    def __del__(self):
        pass

    def stepBegin(self, duration):
        pass

    def stepEnd(self):
        pass


if __name__ == "__main__":
    robot = MazeNavigator()
    while robot.step(robot.timestep) != -1:
        robot.stepBegin(robot.timestep)
        robot.stepEnd()
