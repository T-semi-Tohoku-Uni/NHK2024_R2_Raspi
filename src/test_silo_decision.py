from silo_decision import ScoreingSilos
from NHK2024_Camera_Library import Silo
import typing

class TestSilo(Silo):
    def __init__(self, x, y, z, w):
        super().__init__(x, y, z, w)
        self._my_team_ball_cnt = 0
        self._opponent_team_cnt = 0

    def set_my_team_ball(self, n):
        self._my_team_ball_cnt = n

    def set_opponent_team_ball_cnt(self, n):
        self._opponent_team_cnt = n

    def get_my_team_ball_cnt(self):
        return self._my_team_ball_cnt

    def get_opponent_team_cnt(self):
        return self._opponent_team_cnt

import random as rd

silos = []
for i in range(5):
    silos.append(TestSilo(rd.randint(0, 100), rd.randint(0, 100), rd.randint(0, 100), rd.randint(0, 100)))



for silo in silos:
    balls = rd.randint(0, 3)
    print(f'ball num:{balls}')
    my_ball_num = rd.randint(0, balls)
    print(f'may ball num:{my_ball_num}')
    silo.set_my_team_ball(my_ball_num)
    silo.set_opponent_team_ball_cnt(balls - my_ball_num)
    # s.update_position()

# 生成したSiloを表示
for s in silos:
    print(s.__str__)
    print('generated silo object')
    print(s.get_my_team_ball_cnt())
    print(s.get_opponent_team_cnt())
    print("")

# スコアリング
ss = ScoreingSilos(silos)
sorted_silos = ss.sort_by_score()

# スコアリング後のSiloを表示
for s in sorted_silos:
    print(s.__str__)
    print(f'my balls:{s.get_my_team_ball_cnt()}')
    print(f'opponent balls:{s.get_opponent_team_cnt()}')
    print(ss.get_score(s))
    print("")  