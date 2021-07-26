from preliminary_competition import PreliminaryCompetitionStrategy
from locomotion_control import ActionGroups
import time


class PuppyPi(PreliminaryCompetitionStrategy):
    def __init__(self):
        super().__init__()


if __name__ == '__main__':
    with PuppyPi() as puppypi:
        puppypi.start()

    # ActionGroups('actionGroups/calm.csv').do()
    # time.sleep(3)
    # ActionGroups().unload()
