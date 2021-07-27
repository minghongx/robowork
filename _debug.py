from preliminary_competition import PreliminaryCompetitionStrategy


class PuppyPi(PreliminaryCompetitionStrategy):
    def __init__(self):
        super().__init__()


if __name__ == '__main__':
    with PuppyPi() as puppypi:
        puppypi.start()
        puppypi.finished.wait()
