from puppypi_pro.preliminary_competition import PreliminaryCompetitionStrategy


class PuppyPi(PreliminaryCompetitionStrategy):
    def __init__(self):
        super().__init__()


with PuppyPi() as puppypi:
    puppypi.setup()
    puppypi.start()
    puppypi.finished.wait()
