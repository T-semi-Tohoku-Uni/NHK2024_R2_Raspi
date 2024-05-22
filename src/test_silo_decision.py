from silo_decision import ScoreingSilos
from NHK2024_Camera_Library import Silo

import random as rd
silos = []
for i in range(rd.randint(0, 5)):
    silos.append(Silo(rd.randint(0, 100), rd.randint(0, 100), rd.randint(0, 100), rd.randint(0, 100)))

for s in silos:
    balls = rd.randint(0, 3)
    s.__my_team_ball_cnt = balls - rd.randint(0, balls)
    s.__opponent_team_cnt = balls - s.__my_team_ball_cnt

# 生成したSiloを表示
for s in silos:
    print(s.__str__)
    print(s.get_position())
    print(s.get_my_team_ball_cnt())
    print(s.get_opponent_team_cnt())
    print("")

# スコアリング
ss = ScoreingSilos(silos)
sorted_silo = ss.sort_by_score()

# スコアリング後のSiloを表示
for s in sorted_silo:
    print(s.__str__)
    print(s.get_position())
    print(s.get_my_team_ball_cnt())
    print(s.get_opponent_team_cnt())
    print(ss.get_score(s))
    print(ss.get_silo_full())
    print("")  