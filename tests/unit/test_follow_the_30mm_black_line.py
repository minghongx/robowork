from puppypi_pro.preliminary_competition import PreliminaryCompetitionStrategy
from time import sleep

with PreliminaryCompetitionStrategy() as test:
    test.setup()
    test.follow_the_30mm_black_line()
    sleep(10)  # following the line for n seconds
